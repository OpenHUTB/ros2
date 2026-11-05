"""技能宏的纯逻辑测试：不需要 ROS / 终端 / 仿真器.

覆盖：
  * 四元数 -> 偏航、角度 wrap
  * 起飞宏：P 律收敛到目标高度、进入死区要稳定若干帧才结束
  * 旋转宏：累计偏航到 360° 就停，不会转两圈
  * 前进宏：刹车曲线到点收油，不冲过头
  * MacroRunner：一次只跑一个、未知键/无位姿不启动、cancel、位姿丢失自动取消
"""
import math
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

import skill_macros as sm  # noqa: E402

DT = 1.0 / 30.0


class GeometryTests(unittest.TestCase):
    def test_quat_to_yaw(self):
        self.assertAlmostEqual(sm.quat_to_yaw(0, 0, 0, 1), 0.0, places=9)
        self.assertAlmostEqual(sm.quat_to_yaw(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)),
                               math.pi / 2, places=9)
        self.assertAlmostEqual(sm.quat_to_yaw(0, 0, 1, 0), math.pi, places=9)

    def test_wrap_pi(self):
        self.assertAlmostEqual(sm.wrap_pi(3 * math.pi), math.pi, places=9)
        self.assertAlmostEqual(sm.wrap_pi(-3 * math.pi), math.pi, places=9)
        self.assertAlmostEqual(sm.wrap_pi(0.5), 0.5, places=9)


class TakeoffTests(unittest.TestCase):
    def test_converges_to_target_altitude(self):
        macro = sm.Takeoff(target_z=1.5, kp=2.0, max_speed=1.0)
        pose = sm.Pose(0.0, 0.0, 1.0, 0.0)
        peak = 0.0
        for _ in range(600):
            cmd = macro.update(pose, DT)
            peak = max(peak, cmd[2])
            pose = sm.Pose(pose.x, pose.y, pose.z + cmd[2] * DT, pose.yaw)
            if macro.done:
                break
        self.assertTrue(macro.done, '应能收敛并结束')
        self.assertAlmostEqual(pose.z, 1.5, delta=0.06)
        self.assertLessEqual(peak, 1.0 + 1e-9, '垂直速度不超过 max_speed')

    def test_requires_settle_ticks(self):
        macro = sm.Takeoff(target_z=1.0, settle_ticks=6)
        pose = sm.Pose(0.0, 0.0, 1.0, 0.0)          # 一开始就在目标高度
        for i in range(5):
            macro.update(pose, DT)
            self.assertFalse(macro.done, '死区内也要连续 %d 帧才结束' % 6)
        macro.update(pose, DT)
        self.assertTrue(macro.done)


class SpinTests(unittest.TestCase):
    def test_full_turn_and_stop(self):
        macro = sm.Spin(degrees=360.0, rate=1.0)
        yaw = 0.0
        for _ in range(600):
            cmd = macro.update(sm.Pose(0, 0, 1.5, yaw), DT)
            yaw = sm.wrap_pi(yaw + cmd[3] * DT)
            if macro.done:
                break
        self.assertTrue(macro.done, '360° 应能在 600 帧内转完')
        self.assertAlmostEqual(abs(macro.accumulated), 2 * math.pi, delta=math.radians(4))
        self.assertLess(abs(macro.accumulated), 2 * math.pi + math.radians(10), '不应转过两圈')

    def test_reverse_spin(self):
        macro = sm.Spin(degrees=-180.0, rate=1.0)
        yaw = 0.0
        signs = set()
        for _ in range(600):
            cmd = macro.update(sm.Pose(0, 0, 1.5, yaw), DT)
            if cmd[3]:
                signs.add(math.copysign(1, cmd[3]))
            yaw = sm.wrap_pi(yaw + cmd[3] * DT)
            if macro.done:
                break
        self.assertTrue(macro.done)
        self.assertEqual(signs, {-1.0}, '负角度应朝反方向转')
        self.assertAlmostEqual(abs(macro.accumulated), math.pi, delta=math.radians(4))


class ForwardTests(unittest.TestCase):
    def test_brakes_at_distance(self):
        macro = sm.Forward(distance=3.0, speed=1.5, accel=2.0)
        pose = sm.Pose(0.0, 0.0, 1.5, 0.0)
        speeds = []
        for _ in range(600):
            cmd = macro.update(pose, DT)
            speeds.append(cmd[0])
            pose = sm.Pose(pose.x + cmd[0] * DT, pose.y, pose.z, pose.yaw)
            if macro.done:
                break
        self.assertTrue(macro.done)
        self.assertAlmostEqual(pose.x, 3.0, delta=0.05)
        self.assertLessEqual(max(speeds), 1.5 + 1e-9)
        self.assertLess(speeds[-2], 0.5, '到点前已在减速')

    def test_follows_heading(self):
        macro = sm.Forward(distance=2.0, speed=1.0, accel=1.0)
        pose = sm.Pose(0.0, 0.0, 1.5, math.pi / 2)      # 机头朝 +y
        for _ in range(600):
            cmd = macro.update(pose, DT)
            pose = sm.Pose(pose.x + cmd[0] * math.cos(pose.yaw) * DT,
                           pose.y + cmd[0] * math.sin(pose.yaw) * DT,
                           pose.z, pose.yaw)
            if macro.done:
                break
        self.assertAlmostEqual(pose.y, 2.0, delta=0.05)
        self.assertAlmostEqual(pose.x, 0.0, delta=0.01)


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.runner = sm.MacroRunner()

    def test_trigger_keys(self):
        pose = sm.Pose(0, 0, 1.0, 0.0)
        self.assertTrue(self.runner.trigger('1', pose))
        self.assertEqual(self.runner.name, 'TAKEOFF')
        self.assertTrue(self.runner.trigger('3', pose))
        self.assertEqual(self.runner.name, 'FORWARD', '一次只能跑一个宏')
        self.assertTrue(self.runner.trigger('2', pose))
        self.assertEqual(self.runner.name, 'SPIN')

    def test_unknown_key_and_missing_pose(self):
        self.assertFalse(self.runner.trigger('9', sm.Pose(0, 0, 1, 0)))
        self.assertFalse(self.runner.trigger('1', None))
        self.assertFalse(self.runner.active)

    def test_cancel(self):
        self.runner.trigger('1', sm.Pose(0, 0, 1, 0))
        self.runner.cancel()
        self.assertFalse(self.runner.active)
        self.assertEqual(self.runner.update(sm.Pose(0, 0, 1, 0), DT), (0.0, 0.0, 0.0, 0.0))

    def test_pose_lost_cancels(self):
        self.runner.trigger('1', sm.Pose(0, 0, 1, 0))
        self.assertEqual(self.runner.update(None, DT), (0.0, 0.0, 0.0, 0.0))
        self.assertFalse(self.runner.active, '位姿丢失应自动取消')

    def test_zero_command_when_idle(self):
        self.assertEqual(self.runner.update(sm.Pose(0, 0, 1, 0), DT), (0.0, 0.0, 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
