import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
import yaml

import signal
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from rl_control_node import XMIGCSControlNode


class XMIGCSControlNode_sim(XMIGCSControlNode):
    def __init__(self, debug=False):
        # 默认回退到 sdk_a_sim.yaml 配置文件
        if 'XMIGCS_CONFIG' not in os.environ:
            sim_cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config', 'sdk_a_sim.yaml')
            if os.path.isfile(sim_cfg):
                os.environ['XMIGCS_CONFIG'] = sim_cfg
        # sim=True 经基类构造函数在控制线程启动前生效: 跳过真机并联踝关节求解
        # (MuJoCo 仿真中踝关节为串联表达，跳过并联解算与驱动器电流转换)
        super().__init__(debug, sim=True)

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = None
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        node = XMIGCSControlNode_sim(debug=False)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # SIGTERM 关闭 context 后, spin 中未完成的订阅 take 会抛
        # "Unable to convert call argument"; 上下文仍在则属真实错误, 放行
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception as e:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
