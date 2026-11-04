"""离线单元测试：carla_common 与桥接节点（无 ROS 运行环境、无 CARLA 也可跑）。

说明：bridge_node 顶层 import rclpy，宿主机（无 ROS）不能直接 import，
因此对节点行为采用“源码守卫”测试（与 carla_perception_control 同一惯例）；
纯逻辑部分（carla_common 的惰性导入与错误提示、帧距离）直接测试。
"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import carla_aeb_braking.carla_common as cc  # noqa: E402


def _node_source():
    return (ROOT / "carla_aeb_braking" / "bridge_node.py").read_text(encoding="utf-8")


class TestCarlaCommon(unittest.TestCase):
    def test_imports_without_carla(self):
        # carla 未安装时模块仍可导入（惰性导入）
        self.assertTrue(hasattr(cc, "connect"))
        self.assertTrue(hasattr(cc, "restore_async"))
        self.assertTrue(hasattr(cc, "make_depth_camera"))

    def test_check_carla_raises_friendly_error(self):
        saved = cc.carla
        cc.carla = None
        try:
            with self.assertRaises(RuntimeError) as ctx:
                cc._check_carla()
            self.assertIn("carla", str(ctx.exception).lower())
        finally:
            cc.carla = saved

    def test_connect_raises_without_carla(self):
        saved = cc.carla
        cc.carla = None
        try:
            with self.assertRaises(RuntimeError):
                cc.connect("127.0.0.1", 2000)
        finally:
            cc.carla = saved


class TestBridgeNodeSourceGuards(unittest.TestCase):
    """桥接节点关键行为守卫（源码级，避免依赖 ROS 运行环境）。"""

    def test_publishes_obstacle_distance(self):
        src = _node_source()
        self.assertIn('"/aeb/obstacle_distance"', src)
        self.assertIn("lidar_obstacle_distance", src)

    def test_subscribes_control_and_applies(self):
        src = _node_source()
        self.assertIn('"/aeb/control"', src)
        self.assertIn("cc.apply_control", src)
        self.assertIn("world.tick()", src)

    def test_restores_async_on_destroy(self):
        src = _node_source()
        self.assertIn("cc.restore_async(self.world)", src)

    def test_lidar_frame_is_copied(self):
        # CARLA 复用 raw_data 底层缓冲，必须 bytes() 复制，否则下一帧覆盖
        src = _node_source()
        self.assertIn("bytes(pc.raw_data)", src)


if __name__ == "__main__":
    unittest.main()
