"""技能宏的节点级测试：真实 KeyboardController + 假位姿/假时钟/假终端.

不接 ROS、不接仿真器：`step_once` 的位姿由测试注入，用一个最小的"积分器"当被控对象，
检查真正发到 `/uav/cmd_vel` 的指令能否把飞机带到目标（高度 1.5 m / 转 360° / 前进 3 m）。

覆盖：
  * 3 键前进 3 m：加速 -> 刹车 -> 收油，不冲过头
  * 1 键升到 1.5 m：收敛后结束，回到全零
  * 2 键转 360°：按累计偏航判断停止
  * 没有 /uav/odom 或位姿超时：宏拒绝启动，只发零速
  * 手动方向键中途接管：宏被取消，速度斜坡切到手动值
  * stop()：取消宏并补发零速
"""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

PKG = Path(__file__).resolve().parents[1]
SCRIPT = PKG / 'scripts' / 'keyboard_control.py'
sys.path.insert(0, str(PKG / 'scripts'))

import skill_macros as sm  # noqa: E402

DT = 1.0 / 30.0

PARAMS = {
    "control/horiz_speed": 2.0, "control/vert_speed": 1.0, "control/yaw_rate": 1.0,
    "control/key_hold_timeout": 0.15,
    "control/accel_horiz": 4.0, "control/accel_vert": 2.0, "control/accel_yaw": 4.0,
    "control/odom_timeout": 0.6,
    "macro/takeoff_altitude": 1.5, "macro/spin_degrees": 360.0,
    "macro/forward_distance": 3.0, "macro/forward_speed": 1.5,
    "macro/brake_accel": 2.0, "macro/alt_kp": 2.0,
    "topic/cmd_vel": "/uav/cmd_vel", "topic/odom": "/uav/odom", "rate/publish_hz": 30.0,
}


class _Vec3(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Twist(object):
    def __init__(self):
        self.linear = _Vec3()
        self.angular = _Vec3()


class _Odom(object):
    pass


class MacrosLoopTests(unittest.TestCase):
    def setUp(self):
        ros = types.ModuleType('rospy')
        ros.is_shutdown = Mock(return_value=False)
        ros.get_param = lambda name, default=None: PARAMS.get(name, default)
        ros.Publisher = lambda *a, **k: Mock()
        ros.Rate = lambda *a, **k: Mock()
        ros.Subscriber = lambda *a, **k: None
        ros.logwarn = Mock()

        geometry = types.ModuleType('geometry_msgs')
        messages = types.ModuleType('geometry_msgs.msg')
        messages.Twist = _Twist
        nav = types.ModuleType('nav_msgs')
        nav_msg = types.ModuleType('nav_msgs.msg')
        nav_msg.Odometry = _Odom

        with patch.dict(sys.modules, {'rospy': ros, 'geometry_msgs': geometry,
                                      'geometry_msgs.msg': messages,
                                      'nav_msgs': nav, 'nav_msgs.msg': nav_msg}):
            spec = importlib.util.spec_from_file_location('kb_macros_under_test', SCRIPT)
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)

        self.node = self.module.KeyboardController()
        self.published = []
        self.node.pub = Mock()
        self.node.pub.publish.side_effect = self.published.append

    # ------------------------------------------------------------------ 工具
    def drive(self, key0, plant, max_ticks=800, settle=12):
        """发一次 key0，之后每帧只喂位姿；宏结束后再跑几帧让平滑器收尾."""
        pose = plant.pose
        i = 0
        for i in range(max_ticks):
            key = key0 if i == 0 else None
            cmd = self.node.step_once(key, now=i * DT, dt=DT, pose=pose)
            self.published.append(cmd)
            pose = plant.step(cmd, DT)
            if i > 5 and not self.node.skills.active:
                break
        for j in range(settle):
            cmd = self.node.step_once(None, now=(i + 1 + j) * DT, dt=DT, pose=pose)
            self.published.append(cmd)
            pose = plant.step(cmd, DT)
        plant.pose = pose
        return pose

    # ------------------------------------------------------------------ 用例
    def test_forward_macro_stops_at_3m(self):
        plant = _Plant(sm.Pose(0.0, 0.0, 1.5, 0.0))
        pose = self.drive('3', plant)
        speeds = [m.linear.x for m in self.published]
        self.assertAlmostEqual(pose.x, 3.0, delta=0.1, msg='应停在 3 m 处')
        self.assertLessEqual(max(speeds), 1.5 + 1e-6, '不超过 macro/forward_speed')
        self.assertLess(speeds[-1], 1e-9, '收尾后指令回到全零')
        self.assertFalse(self.node.skills.active)
        self.assertGreater(len(self.published), 60, '全程持续发布指令')

    def test_takeoff_macro_reaches_altitude(self):
        plant = _Plant(sm.Pose(0.0, 0.0, 1.0, 0.0))
        pose = self.drive('1', plant)
        self.assertAlmostEqual(pose.z, 1.5, delta=0.07)
        self.assertAlmostEqual(self.published[-1].linear.z, 0.0, places=9)
        self.assertFalse(self.node.skills.active)

    def test_spin_macro_turns_360(self):
        plant = _Plant(sm.Pose(0.0, 0.0, 1.5, 0.0))
        pose = self.drive('2', plant)
        turned = abs(sm.wrap_pi(pose.yaw - 0.0))
        self.assertGreater(len(self.published), 100, '360° 需要持续几十帧')
        self.assertAlmostEqual(turned, 0.0, delta=0.15, msg='转满一圈后偏航回到原方向附近')
        self.assertFalse(self.node.skills.active)

    def test_odom_callback_updates_pose(self):
        """cb_odom 把 nav_msgs/Odometry 转成 Pose（位置 + 四元数解偏航）."""
        q = 0.7071067811865476            # 90°
        msg = types.SimpleNamespace(
            pose=types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=1.0, y=2.0, z=3.0),
                    orientation=types.SimpleNamespace(x=0.0, y=0.0, z=q, w=q),
                )
            )
        )
        self.node.cb_odom(msg)
        self.assertAlmostEqual(self.node.pose.x, 1.0, places=9)
        self.assertAlmostEqual(self.node.pose.y, 2.0, places=9)
        self.assertAlmostEqual(self.node.pose.z, 3.0, places=9)
        self.assertAlmostEqual(self.node.pose.yaw, 3.141592653589793 / 2, places=6)
        self.assertGreater(self.node.pose_time, 0.0)

    def test_macro_refused_without_odom(self):
        self.assertIsNone(self.node.pose)
        cmd = self.node.step_once('3', now=0.0, dt=DT)
        self.assertAlmostEqual(cmd.linear.x, 0.0, places=9)
        self.assertFalse(self.node.skills.active, '没有位姿不该启动宏')

    def test_stale_odom_refused(self):
        self.node.pose = sm.Pose(0.0, 0.0, 1.0, 0.0)
        self.node.pose_time = 0.0
        cmd = self.node.step_once('1', now=5.0, dt=DT)   # 位姿已过期 5 s
        self.assertAlmostEqual(cmd.linear.z, 0.0, places=9)
        self.assertFalse(self.node.skills.active)

    def test_manual_key_preempts_macro(self):
        pose = sm.Pose(0.0, 0.0, 1.5, 0.0)
        self.node.step_once('3', now=0.0, dt=DT, pose=pose)
        self.assertTrue(self.node.skills.active)

        # 第 2 帧按 W：宏应立即取消，改由手动键控制
        self.node.step_once('w', now=DT, dt=DT, pose=pose)
        self.assertFalse(self.node.skills.active, '手动键应取消宏')

        last = None
        for i in range(2, 40):
            last = self.node.step_once('w', now=i * DT, dt=DT, pose=pose)
        self.assertGreater(last.linear.x, 1.9, '接管后应加速到手动 horiz_speed')

    def test_stop_cancels_macro_and_publishes_zero(self):
        pose = sm.Pose(0.0, 0.0, 1.5, 0.0)
        self.node.step_once('3', now=0.0, dt=DT, pose=pose)
        self.assertTrue(self.node.skills.active)
        self.node.stop()
        self.assertFalse(self.node.skills.active)
        self.assertAlmostEqual(self.published[-1].linear.x, 0.0, places=9)


class _Plant(object):
    """最小被控对象：把机体系速度指令积分成位姿（只覆盖本测试需要的两轴 + 偏航）."""

    def __init__(self, pose):
        self.pose = pose

    def step(self, cmd, dt):
        p = self.pose
        vx, _, vz, wz = (cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z)
        self.pose = sm.Pose(p.x + vx * dt * 1.0,
                            p.y,
                            p.z + vz * dt,
                            sm.wrap_pi(p.yaw + wz * dt))
        return self.pose


if __name__ == '__main__':
    unittest.main()
