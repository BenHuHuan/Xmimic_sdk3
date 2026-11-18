"""Convert Tiangong MJCF scene XML + model29 STL assets into a RaiSim-compatible URDF."""
from pathlib import Path
import math
import struct
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape
import numpy as np


def _parse_vec(text, default):
    if not text:
        return list(default)
    return [float(x) for x in text.strip().split()]


def _quat_wxyz_to_mat(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    w, x, y, z = w / n, x / n, y / n, z / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def quat_wxyz_to_rpy(q):
    """Convert MuJoCo (w, x, y, z) quaternion to URDF (roll, pitch, yaw) extrinsic XYZ."""
    R = _quat_wxyz_to_mat(q)
    sy = -R[2][0]
    sy_clamped = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy_clamped)
    if abs(abs(sy_clamped) - 1.0) > 1e-9:
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-R[0][1], R[1][1])
    return [roll, pitch, yaw]


def _fmt_vec(vals):
    return " ".join(f"{v:.12g}" for v in vals)


def _resolve_mesh_path(mesh_elem, mesh_dir):
    file_name = mesh_elem.attrib["file"]
    candidate = (mesh_dir / file_name).resolve()
    stl_candidate = candidate.with_suffix(".STL")
    if candidate.suffix.lower() != ".stl" and stl_candidate.is_file():
        return stl_candidate
    stl_lower = candidate.with_suffix(".stl")
    if candidate.suffix.lower() != ".stl" and stl_lower.is_file():
        return stl_lower
    return candidate


def _stl_bbox(mesh_path):
    """Return (center_xyz, size_xyz) bounding box of a binary/ASCII STL mesh."""
    path = Path(mesh_path)
    if not path.is_file() or path.suffix.lower() != ".stl":
        return None
    data = path.read_bytes()
    if len(data) >= 84 and not data[:5].lower().startswith(b"solid"):
        n_tris = struct.unpack("<I", data[80:84])[0]
        expected = 84 + 50 * n_tris
        if n_tris > 0 and len(data) >= expected:
            dtype = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("attr", "<u2")])
            pts = np.frombuffer(data[84:expected], dtype=dtype)["v"].reshape(-1, 3)
            mn = pts.min(axis=0).astype(float)
            mx = pts.max(axis=0).astype(float)
            return ((0.5 * (mn + mx)).tolist(), np.maximum(mx - mn, 1e-3).tolist())
    # ASCII STL fallback (or binary file starting with 'solid')
    pts = []
    for line in data.decode("utf-8", errors="ignore").splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0] == "vertex":
            pts.append((float(parts[1]), float(parts[2]), float(parts[3])))
    if not pts:
        return None
    arr = np.asarray(pts, dtype=np.float64)
    mn = arr.min(axis=0)
    mx = arr.max(axis=0)
    return ((0.5 * (mn + mx)).tolist(), np.maximum(mx - mn, 1e-3).tolist())


def _mat_mul_vec(R, v):
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


def mjcf_to_raisim_urdf(mjcf_path, output_urdf_path):
    """Generate a RaiSim URDF from an MJCF scene and return model metadata."""
    mjcf_path = Path(mjcf_path).resolve()
    output_urdf_path = Path(output_urdf_path).resolve()
    root = ET.parse(mjcf_path).getroot()
    robot_name = root.attrib.get("model", "robot")

    compiler = root.find("compiler")
    meshdir_rel = compiler.attrib.get("meshdir", ".") if compiler is not None else "."
    mesh_dir = (mjcf_path.parent / meshdir_rel).resolve()

    meshes = {}
    for mesh_elem in root.findall("./asset/mesh"):
        name = mesh_elem.attrib.get("name") or Path(mesh_elem.attrib["file"]).stem
        meshes[name] = _resolve_mesh_path(mesh_elem, mesh_dir)

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"MJCF missing <worldbody>: {mjcf_path}")
    root_body = worldbody.find("body")
    if root_body is None:
        raise ValueError(f"MJCF missing root <body>: {mjcf_path}")

    actuator_limits = {}
    for act in root.findall("./actuator/*"):
        jname = act.attrib.get("joint")
        if jname and "ctrlrange" in act.attrib:
            lo, hi = _parse_vec(act.attrib["ctrlrange"], (-100.0, 100.0))
            actuator_limits[jname] = max(abs(lo), abs(hi))

    excluded_pairs = [
        (ex.attrib["body1"], ex.attrib["body2"])
        for ex in root.findall("./contact/exclude")
        if "body1" in ex.attrib and "body2" in ex.attrib
    ]

    links_xml = []
    joints_xml = []
    joint_info = {}
    link_names = []
    total_mass = 0.0

    def walk_body(body_elem, parent_link=None, grandparent_link=None):
        nonlocal total_mass
        bname = body_elem.attrib["name"]
        link_names.append(bname)
        if grandparent_link is not None:
            pair = (grandparent_link, bname)
            if pair not in excluded_pairs and (bname, grandparent_link) not in excluded_pairs:
                excluded_pairs.append(pair)
        bpos = _parse_vec(body_elem.attrib.get("pos"), (0.0, 0.0, 0.0))
        bquat = _parse_vec(body_elem.attrib.get("quat"), (1.0, 0.0, 0.0, 0.0))
        brpy = quat_wxyz_to_rpy(bquat)

        inertial = body_elem.find("inertial")
        if inertial is not None:
            ipos = _parse_vec(inertial.attrib.get("pos"), (0.0, 0.0, 0.0))
            iquat = _parse_vec(inertial.attrib.get("quat"), (1.0, 0.0, 0.0, 0.0))
            irpy = quat_wxyz_to_rpy(iquat)
            mass = float(inertial.attrib.get("mass", "1e-4"))
            diag = _parse_vec(inertial.attrib.get("diaginertia"), (1e-6, 1e-6, 1e-6))
        else:
            ipos = [0.0, 0.0, 0.0]
            irpy = [0.0, 0.0, 0.0]
            mass = 1e-4
            diag = [1e-6, 1e-6, 1e-6]
        total_mass += mass

        visuals = []
        collisions = []
        for geom in body_elem.findall("geom"):
            if geom.attrib.get("type", "mesh") != "mesh" or "mesh" not in geom.attrib:
                continue
            mesh_path = meshes.get(geom.attrib["mesh"])
            if mesh_path is None:
                continue
            gpos = _parse_vec(geom.attrib.get("pos"), (0.0, 0.0, 0.0))
            gquat = _parse_vec(geom.attrib.get("quat"), (1.0, 0.0, 0.0, 0.0))
            grpy = quat_wxyz_to_rpy(gquat)
            rgba = _parse_vec(geom.attrib.get("rgba"), (0.75, 0.75, 0.75, 1.0))
            is_visual_only = (
                geom.attrib.get("contype") == "0"
                and geom.attrib.get("conaffinity") == "0"
            ) or geom.attrib.get("group") == "2"
            origin_tag = f'<origin xyz="{_fmt_vec(gpos)}" rpy="{_fmt_vec(grpy)}"/>'
            geom_tag = f'<geometry><mesh filename="{escape(str(mesh_path))}"/></geometry>'
            if is_visual_only:
                visuals.append(
                    f'    <visual>\n      {origin_tag}\n      {geom_tag}\n'
                    f'      <material name=""><color rgba="{_fmt_vec(rgba)}"/></material>\n'
                    f'    </visual>'
                )
            else:
                bbox = _stl_bbox(mesh_path)
                if bbox is not None:
                    local_center, box_size = bbox
                    rc = _mat_mul_vec(_quat_wxyz_to_mat(gquat), local_center)
                    cpos = [gpos[i] + rc[i] for i in range(3)]
                    corigin = f'<origin xyz="{_fmt_vec(cpos)}" rpy="{_fmt_vec(grpy)}"/>'
                    cgeom = f'<geometry><box size="{_fmt_vec(box_size)}"/></geometry>'
                    collisions.append(
                        f'    <collision>\n      {corigin}\n      {cgeom}\n    </collision>'
                    )
                else:
                    collisions.append(
                        f'    <collision>\n      {origin_tag}\n      {geom_tag}\n    </collision>'
                    )

        link_parts = [
            f'  <link name="{escape(bname)}">',
            '    <inertial>',
            f'      <origin xyz="{_fmt_vec(ipos)}" rpy="{_fmt_vec(irpy)}"/>',
            f'      <mass value="{mass:.12g}"/>',
            f'      <inertia ixx="{diag[0]:.12g}" ixy="0" ixz="0" iyy="{diag[1]:.12g}" iyz="0" izz="{diag[2]:.12g}"/>',
            '    </inertial>',
            *visuals,
            *collisions,
            '  </link>',
        ]
        links_xml.append("\n".join(link_parts))

        if parent_link is not None:
            j_elem = body_elem.find("joint")
            if j_elem is not None and j_elem.tag == "joint":
                jname = j_elem.attrib["name"]
                axis = _parse_vec(j_elem.attrib.get("axis"), (0.0, 0.0, 1.0))
                jrange = _parse_vec(j_elem.attrib.get("range"), (-3.14159, 3.14159))
                frcrange = _parse_vec(j_elem.attrib.get("actuatorfrcrange"), (-100.0, 100.0))
                effort = actuator_limits.get(jname, max(abs(frcrange[0]), abs(frcrange[1])))
                armature = float(j_elem.attrib.get("armature", "0.0"))
                damping = float(j_elem.attrib.get("damping", "0.0"))
                friction = float(j_elem.attrib.get("frictionloss", "0.0"))
                joint_info[jname] = {
                    "effort": effort,
                    "armature": armature,
                    "damping": damping,
                    "friction": friction,
                    "range": jrange,
                }
                joints_xml.append(
                    "\n".join([
                        f'  <joint name="{escape(jname)}" type="revolute">',
                        f'    <origin xyz="{_fmt_vec(bpos)}" rpy="{_fmt_vec(brpy)}"/>',
                        f'    <parent link="{escape(parent_link)}"/>',
                        f'    <child link="{escape(bname)}"/>',
                        f'    <axis xyz="{_fmt_vec(axis)}"/>',
                        f'    <limit lower="{jrange[0]:.12g}" upper="{jrange[1]:.12g}" effort="{effort:.12g}" velocity="100.0"/>',
                        f'    <dynamics damping="{damping:.12g}" friction="{friction:.12g}"/>',
                        '  </joint>',
                    ])
                )
            else:
                joints_xml.append(
                    "\n".join([
                        f'  <joint name="{escape(bname)}_fixed_joint" type="fixed">',
                        f'    <origin xyz="{_fmt_vec(bpos)}" rpy="{_fmt_vec(brpy)}"/>',
                        f'    <parent link="{escape(parent_link)}"/>',
                        f'    <child link="{escape(bname)}"/>',
                        '  </joint>',
                    ])
                )

        for child in body_elem.findall("body"):
            walk_body(child, bname, parent_link)

    walk_body(root_body, parent_link=None, grandparent_link=None)
    urdf_text = "\n".join([
        '<?xml version="1.0" encoding="utf-8"?>',
        f'<robot name="{escape(robot_name)}">',
        *links_xml,
        *joints_xml,
        '</robot>',
        '',
    ])
    output_urdf_path.parent.mkdir(parents=True, exist_ok=True)
    output_urdf_path.write_text(urdf_text, encoding="utf-8")
    return {
        "urdf_path": str(output_urdf_path),
        "mesh_dir": str(mesh_dir),
        "joint_names": list(joint_info.keys()),
        "joint_info": joint_info,
        "link_names": link_names,
        "excluded_pairs": excluded_pairs,
        "total_mass": total_mass,
    }
