#!/usr/bin/env python3
"""
MJCF 场景生成器（EVT 机器人版, 位于 xmujoco/terrain/）

参考 bitbot-ovinf terrain/terrain_generator.py（原脚本把地形写进机器人 xml），
本脚本生成独立的场景 xml，通过 <include> 引用机器人模型：
  - 默认引用 urdf/dex_evt_depth.xml（带深度相机的 EVT 模型）
  - 输出 urdf/scene_test.xml（与 scene_parkour.xml 同构）
依赖: numpy, pillow（无需 cv2 / noise）。

用法:
    python terrain_generator.py                        # 生成 scene_test.xml (dex 布局)
    python terrain_generator.py -o scene_test -l depth_test --seed 3
    python terrain_generator.py --robot dex_evt.xml    # 不带相机的裸 EVT
"""

import argparse
import os
import xml.etree.ElementTree as xml_et

import numpy as np
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_DIR = os.path.join(SCRIPT_DIR, "..", "urdf")
HFIELD_DIR = os.path.join(SCRIPT_DIR, "hfields")


def euler_to_quat(roll, pitch, yaw):
    cx, sx = np.cos(roll / 2), np.sin(roll / 2)
    cy, sy = np.cos(pitch / 2), np.sin(pitch / 2)
    cz, sz = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cx * cy * cz + sx * sy * sz,
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * cz,
        cx * cy * sz - sx * sy * cz,
    ], dtype=np.float64)


def euler_to_rot(roll, pitch, yaw):
    rot_x = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)],
                      [0, np.sin(roll), np.cos(roll)]])
    rot_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0],
                      [-np.sin(pitch), 0, np.cos(pitch)]])
    rot_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return rot_z @ rot_y @ rot_x


def rot2d(x, y, yaw):
    return (x * np.cos(yaw) - y * np.sin(yaw), x * np.sin(yaw) + y * np.cos(yaw))


def rot3d(pos, euler):
    return euler_to_rot(*euler) @ np.asarray(pos, dtype=np.float64)


def list_to_str(vec):
    return " ".join(str(s) for s in vec)


def fractal_noise(shape, octaves=6, persistence=0.5, lacunarity=2.0, seed=0):
    """numpy 实现的分形值噪声（替代 noise.pnoise2），返回 0..1"""
    rng = np.random.default_rng(seed)
    h, w = shape
    total = np.zeros((h, w))
    norm, amp, freq = 0.0, 1.0, 4.0
    for _ in range(octaves):
        gh, gw = max(2, int(freq)), max(2, int(freq))
        grid = rng.random((gh + 1, gw + 1))
        ys = np.linspace(0, gh, h)
        xs = np.linspace(0, gw, w)
        yi = np.clip(ys.astype(int), 0, gh - 1)
        xi = np.clip(xs.astype(int), 0, gw - 1)
        yf, xf = ys - yi, xs - xi
        v00 = grid[np.ix_(yi, xi)]
        v01 = grid[np.ix_(yi, xi + 1)]
        v10 = grid[np.ix_(yi + 1, xi)]
        v11 = grid[np.ix_(yi + 1, xi + 1)]
        total += amp * (v00 * (1 - xf) * (1 - yf) + v01 * xf * (1 - yf) +
                        v10 * (1 - xf) * yf + v11 * xf * yf)
        norm += amp
        amp *= persistence
        freq *= lacunarity
    return total / norm


class SceneGenerator:
    """生成引用机器人 xml 的独立场景文件（不改动机器人 xml 本身）"""

    def __init__(self, robot_xml="dex_evt_depth.xml", model_name="test_scene"):
        self.robot_xml = robot_xml
        self.root = xml_et.Element("mujoco", {"model": model_name})
        xml_et.SubElement(self.root, "include", {"file": robot_xml})
        self.asset = xml_et.SubElement(self.root, "asset")
        self.worldbody = xml_et.SubElement(self.root, "worldbody")
        self._geom_count = 0
        self._hfield_count = 0

    def _next_geom_name(self, prefix):
        self._geom_count += 1
        return f"{prefix}_{self._geom_count}"

    def AddBox(self, position=(1.0, 0.0, 0.0), euler=(0.0, 0.0, 0.0),
               size=(0.1, 0.1, 0.1), rgba=None):
        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["name"] = self._next_geom_name("box")
        geo.attrib["pos"] = list_to_str(position)
        geo.attrib["type"] = "box"
        geo.attrib["size"] = list_to_str(
            0.5 * np.array(size))  # mujoco box 为半尺寸
        geo.attrib["quat"] = list_to_str(euler_to_quat(*euler))
        if rgba is None:
            r = np.random.uniform(0, 1)
            g = np.random.uniform(0, 1 - r)
            b = 1 - r - g
            rgba = (r, g, b)
        # Alpha 必须为 1.0：alpha < 0.995 的 geom 在 MuJoCo 透明渲染通道中
        # 不写深度缓冲，深度相机(mjr_readPixels)看不到它们。
        geo.attrib["rgba"] = list_to_str(list(rgba[:3]) + [1.0])

    def AddGeometry(self, position=(1.0, 0.0, 0.0), euler=(0.0, 0.0, 0.0),
                    size=(0.1, 0.1), geo_type="cylinder"):
        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["name"] = self._next_geom_name(geo_type)
        geo.attrib["pos"] = list_to_str(position)
        geo.attrib["type"] = geo_type
        geo.attrib["size"] = list_to_str(
            0.5 * np.array(size[:2]) if geo_type in ("cylinder", "capsule")
            else 0.5 * np.array(size))
        geo.attrib["quat"] = list_to_str(euler_to_quat(*euler))

    def AddStairs(self, init_pos=(1.0, 0.0, 0.0), yaw=0.0, width=0.2,
                  height=0.15, length=1.5, stair_nums=10):
        local_pos = [0.0, 0.0, -0.5 * height]
        for _ in range(stair_nums):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.AddBox((x + init_pos[0], y + init_pos[1], local_pos[2]),
                        (0.0, 0.0, yaw), (width, length, height))

    def AddSuspendStairs(self, init_pos=(1.0, 0.0, 0.0), yaw=0.0, width=0.2,
                         height=0.15, length=1.5, gap=0.1, stair_nums=10):
        local_pos = [0.0, 0.0, -0.5 * height]
        for _ in range(stair_nums):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.AddBox((x + init_pos[0], y + init_pos[1], local_pos[2]),
                        (0.0, 0.0, yaw), (width, length, abs(height - gap)))

    def AddRoughGround(self, init_pos=(1.0, 0.0, 0.0), euler=(0.0, 0.0, 0.0),
                       nums=(10, 10), box_size=(0.5, 0.5, 0.5),
                       box_euler=(0.0, 0.0, 0.0), separation=(0.2, 0.2),
                       box_size_rand=(0.05, 0.05, 0.05),
                       box_euler_rand=(0.2, 0.2, 0.2),
                       separation_rand=(0.05, 0.05)):
        local_pos = [0.0, 0.0, -0.5 * box_size[2]]
        new_sep = np.array(separation, dtype=np.float64)
        for _ in range(nums[0]):
            local_pos[0] += separation[0]
            local_pos[1] = 0.0
            for _ in range(nums[1]):
                new_size = np.array(box_size) + \
                    np.array(box_size_rand) * np.random.uniform(-1, 1, 3)
                new_euler = np.array(box_euler) + \
                    np.array(box_euler_rand) * np.random.uniform(-1, 1, 3)
                new_sep = np.array(separation) + \
                    np.array(separation_rand) * np.random.uniform(-1, 1, 2)
                local_pos[1] += new_sep[1]
                pos = rot3d(local_pos, euler) + np.array(init_pos)
                self.AddBox(pos.tolist(), new_euler.tolist(),
                            new_size.tolist())
            local_pos[0] += new_sep[0] - separation[0]

    def AddFractalHeightField(self, position=(1.0, 0.0, 0.0),
                              euler=(0.0, 0.0, 0.0), size=(4.0, 4.0),
                              height_scale=0.2, negative_height=0.05,
                              image_width=128, image_height=128,
                              octaves=6, persistence=0.5, seed=0,
                              hfield_name=None):
        self._hfield_count += 1
        name = hfield_name or f"fractal_hf_{self._hfield_count}"
        img_path = os.path.join(HFIELD_DIR, f"{name}.png")
        noise_img = fractal_noise((image_height, image_width), octaves=octaves,
                                  persistence=persistence, seed=seed)
        os.makedirs(HFIELD_DIR, exist_ok=True)
        Image.fromarray((noise_img * 255).astype(np.uint8), mode="L").save(
            img_path)

        hfield = xml_et.SubElement(self.asset, "hfield")
        hfield.attrib["name"] = name
        hfield.attrib["size"] = list_to_str(
            [size[0] / 2, size[1] / 2, height_scale, negative_height])
        # 场景 xml 位于 urdf/，高度图路径相对其解析
        hfield.attrib["file"] = f"../terrain/hfields/{name}.png"

        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["name"] = self._next_geom_name("hfield")
        geo.attrib["type"] = "hfield"
        geo.attrib["hfield"] = name
        geo.attrib["pos"] = list_to_str(position)
        geo.attrib["quat"] = list_to_str(euler_to_quat(*euler))

    def AddHeightFieldFromImage(self, input_img, position=(1.0, 0.0, 0.0),
                                euler=(0.0, 0.0, 0.0), size=(2.0, 1.6),
                                height_scale=0.05, negative_height=0.1,
                                image_scale=(1.0, 1.0), invert_gray=False,
                                hfield_name=None):
        self._hfield_count += 1
        name = hfield_name or f"image_hf_{self._hfield_count}"
        img = Image.open(input_img).convert("L")
        w = int(img.width * image_scale[0])
        h = int(img.height * image_scale[1])
        if (w, h) != img.size:
            img = img.resize((w, h), Image.Resampling.BILINEAR)
        arr = np.asarray(img, dtype=np.uint8)
        if invert_gray:
            arr = 255 - arr
        out_path = os.path.join(HFIELD_DIR, f"{name}.png")
        os.makedirs(HFIELD_DIR, exist_ok=True)
        Image.fromarray(arr, mode="L").save(out_path)

        hfield = xml_et.SubElement(self.asset, "hfield")
        hfield.attrib["name"] = name
        hfield.attrib["size"] = list_to_str(
            [size[0] / 2, size[1] / 2, height_scale, negative_height])
        hfield.attrib["file"] = f"../terrain/hfields/{name}.png"

        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["name"] = self._next_geom_name("hfield")
        geo.attrib["type"] = "hfield"
        geo.attrib["hfield"] = name
        geo.attrib["pos"] = list_to_str(position)
        geo.attrib["quat"] = list_to_str(euler_to_quat(*euler))

    def Save(self, output_path):
        xml_et.ElementTree(self.root).write(output_path, encoding="utf-8",
                                            xml_declaration=True)
        print(f"scene written: {output_path}")


def Gap(sg: SceneGenerator):
    """上坡接平台，平台间留缺口（bitbot-ovinf terrain_generator.py 的移植）"""
    init_pos = np.array([1.0, -3.0, 0.0])
    bias_x = 3.0
    platform_l = 1.0
    gap_size = [0.6, 0.6, 0.6, 0.6]
    height = 0.3
    slope_angle = 0.12  # rad (~7 deg)
    slope_width = 2.0
    slope_thickness = 0.005

    slope_length = height / np.sin(slope_angle)
    slope_horiz = slope_length * np.cos(slope_angle)
    slope_x_center = bias_x - slope_horiz / 2

    sg.AddBox(
        position=(init_pos + np.array([slope_x_center, 0.0, height / 2])).tolist(),
        euler=[0.0, -slope_angle, 0.0],
        size=[slope_length, slope_width, slope_thickness],
    )
    box_pos = init_pos + np.array([bias_x + platform_l / 2.0, 0.0, height / 2])
    sg.AddBox(position=box_pos.tolist(), size=[platform_l, 2.0, height])
    for _ in gap_size:
        box_pos = box_pos + np.array([platform_l + 0.6, 0.0, 0.0])
        sg.AddBox(position=box_pos.tolist(), size=[platform_l, 2.0, height])


def Stairs(sg: SceneGenerator):
    """金字塔台阶：10 级上、平台、10 级下（bitbot-ovinf 布局）"""
    init_pos = np.array([1.0, 2.0, 0.0])
    height = 0.15
    width = 0.35
    length = 4.5
    stair_nums = 10
    platform_length = 1.0

    sg.AddStairs(
        init_pos=init_pos.tolist(),
        yaw=0.0,
        width=width,
        height=height,
        length=length,
        stair_nums=stair_nums,
    )
    sg.AddBox(
        position=[
            init_pos[0] + width * stair_nums + (width + platform_length) / 2,
            init_pos[1],
            height * stair_nums + init_pos[2] - 0.05,
        ],
        euler=[0.0, 0.0, 0.0],
        size=[platform_length, length, 0.1],
    )
    sg.AddStairs(
        init_pos=(
            init_pos
            + np.array([width * (stair_nums * 2 + 1) + platform_length, 0.0, 0.0])
        ).tolist(),
        yaw=3.14,
        width=width,
        height=height,
        length=length,
        stair_nums=stair_nums,
    )


def DexTerrain(sg: SceneGenerator):
    """与 bitbot-ovinf terrain_generator.py 的 DexTerrain 完全一致的地形"""
    Gap(sg)
    Stairs(sg)
    sg.AddBox(position=[20.0, 0.0, 0.0], euler=[0.0, 0.0, 0.0], size=[1.0, 20.0, 5])
    sg.AddBox(position=[0.0, -10.0, 0.0], euler=[0.0, 0.0, 0.0], size=[20.0, 1.0, 5])
    sg.AddBox(position=[2.0, 7.0, 0.0], euler=[0.0, 0.0, 0.0], size=[2.0, 3.0, 0.6])


def DepthTestScene(sg: SceneGenerator):
    """深度渲染测试场景：出生点前方 (+X) 依次布置各类障碍"""
    sg.AddBox((1.5, 0.0, 0.15), size=(0.3, 0.3, 0.3))
    sg.AddBox((2.5, -0.8, 0.2), size=(0.4, 0.4, 0.4))
    sg.AddBox((2.5, 0.8, 0.1), size=(0.2, 0.2, 0.2))
    sg.AddGeometry((3.5, 0.0, 0.25), size=(0.5, 0.5), geo_type="cylinder")
    sg.AddStairs(init_pos=(4.5, 2.0, 0.0), width=0.3, height=0.1,
                 length=2.0, stair_nums=6)
    sg.AddSuspendStairs(init_pos=(4.5, -2.5, 0.0), width=0.3, height=0.12,
                        length=2.0, gap=0.05, stair_nums=6)
    sg.AddRoughGround(init_pos=(9.0, -2.0, 0.0), nums=(6, 8),
                      box_size=(0.25, 0.25, 0.25),
                      separation=(0.4, 0.4))
    sg.AddFractalHeightField(position=(13.0, 0.0, 0.0), size=(6.0, 6.0),
                             height_scale=0.25, negative_height=0.1,
                             seed=7)
    sg.AddBox((17.0, 0.0, 1.0), size=(0.5, 12.0, 2.0), rgba=(0.6, 0.6, 0.6))
    sg.AddBox((0.0, 6.0, 1.0), size=(20.0, 0.5, 2.0), rgba=(0.6, 0.6, 0.6))
    sg.AddBox((0.0, -6.0, 1.0), size=(20.0, 0.5, 2.0), rgba=(0.6, 0.6, 0.6))


LAYOUTS = {
    "dex": DexTerrain,
    "depth_test": DepthTestScene,
}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MJCF 场景生成器 (EVT)")
    parser.add_argument("-o", "--output", default="scene_test",
                        help="输出场景文件名（不含路径，默认 scene_test）")
    parser.add_argument("-l", "--layout", default="dex",
                        choices=sorted(LAYOUTS),
                        help="地形布局（默认 dex，与 bitbot-ovinf 一致）")
    parser.add_argument("--seed", type=int, default=None,
                        help="随机种子（颜色与粗糙地面）")
    parser.add_argument("--robot", default="dex_evt_depth.xml",
                        help="被引用的机器人 xml（默认带深度相机的 EVT）")
    args = parser.parse_args()

    if args.seed is not None:
        np.random.seed(args.seed)

    sg = SceneGenerator(robot_xml=args.robot,
                        model_name=f"{args.output}_scene")
    LAYOUTS[args.layout](sg)
    out = os.path.join(URDF_DIR, f"{args.output}.xml")
    sg.Save(out)
