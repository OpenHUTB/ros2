# -*- coding: utf-8 -*-
"""场景与射线感知测试（需要 MuJoCo；未安装时自动跳过并说明原因）。

覆盖：
    1. 场景能加载，关键对象齐备；
    2. 射线不会打到自车（geom group 屏蔽 + bodyexclude 是否生效）；
    3. "本车道判定"确实把路侧目标滤掉了（原始最近命中 vs 本车道最近命中）；
    4. 射线测距与真值间距在同一基准上（site 位于前保险杠）；
    5. 给定轮速后车辆确实前进；
    6. 等轮速直行不跑偏（接触断续/偏航的回归）；
    7. 射线对车身俯仰免疫、且不会打到地面/路面；
    8. 驱动轮有足够牵引力（满指令下车速应接近指令速度）；
    9. 擦着路侧干扰物开过去时，本车道判定不会把它误判进来；
    10. 驱动轮出力有限幅（阶跃指令不会把车弹飞/掀翻）。
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoco_car_avoidance import config                      # noqa: E402
from mujoco_car_avoidance.car_env import CarEnv, MujocoUnavailable  # noqa: E402

try:
    import mujoco  # noqa: F401

    MUJOCO_OK = True
    MUJOCO_HINT = ""
except Exception as exc:  # pragma: no cover
    MUJOCO_OK = False
    MUJOCO_HINT = "未安装 MuJoCo（{}），已跳过场景测试".format(exc)


@unittest.skipUnless(MUJOCO_OK, MUJOCO_HINT)
class TestSceneAndSensors(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        scene = config.resolve_scene_path()
        if not scene.is_file():
            raise unittest.SkipTest("找不到场景文件：{}".format(scene))
        cls.env = CarEnv(scene)

    def setUp(self):
        # 每个用例前复位：用例之间不共享车辆位置，避免测试顺序影响结果
        self.env.reset()

    def test_scene_objects_exist(self):
        # 构造 CarEnv 时已按名称解析所有对象，能走到这里即说明名称齐备
        self.assertGreater(self.env.model.ngeom, 5)
        self.assertGreater(self.env.model.nu, 0)

    def test_static_ray_does_not_hit_ego(self):
        distance = self.env.sensed_distance()
        self.assertIsNotNone(distance, "静止时应能测到前方障碍物")
        self.assertGreater(distance, 6.0, "射线疑似打到了自车（自车应位于 geom group 3 并被屏蔽）")

    def test_lane_filter_rejects_roadside_object(self):
        ranges = self.env.scan()
        raw_min = float(ranges.min())
        in_lane_min = self.env.lidar.min_distance(ranges)

        self.assertLess(raw_min, 6.0, "路侧干扰物应能被某束射线打到")
        self.assertIsNotNone(in_lane_min)
        self.assertGreater(in_lane_min - raw_min, 1.0,
                           "本车道判定未生效：原始最近命中与车道内最近命中过于接近")

    def test_sensor_distance_matches_truth(self):
        """锁定 mj_ray 的量纲约定：射线测距与真值间距应在同一基准上（单位为米）。

        这条断言同时是回归测试：若以后又把 `方向 × max_range` 传给 mj_ray，
        返回值会变成 0~1 的参数，这里会立刻失败。
        """
        sensed = self.env.sensed_distance()
        truth = self.env.truth_clearance()
        self.assertIsNotNone(sensed)
        self.assertAlmostEqual(sensed, truth, delta=0.05,
                               msg="site 应位于前保险杠，射线测距与真值间距基准一致（米）")

    def test_wheel_command_moves_car_forward(self):
        self.env.reset()
        start = self.env.state().x
        self.env.set_wheel_speeds(10.0, 10.0)
        self.env.step(int(1.0 / self.env.timestep))
        moved = self.env.state().x - start
        self.assertGreater(moved, 0.1)

    def test_straight_driving_keeps_lane(self):
        """等轮速直行 3 s：横向漂移与偏航必须很小。

        回归背景：出生时轮胎与地面恰好相切，接触会断续（ncon 在 0/3/6 之间跳），
        车轮空转（滑移率 55%+）并由左右不对称摩擦引起约 10° 偏航；
        偏航使"本车道"跟着旋转，路侧干扰物被误判为本车道障碍物而提前刹停。
        """
        self.env.reset()
        # 先静置：等接触建立（等价于 runner.settle() 的作用）
        self.env.set_wheel_speeds(0.0, 0.0)
        self.env.step(int(0.6 / self.env.timestep))

        self.env.set_wheel_speeds(20.0, 20.0)          # 指令 1.0 m/s
        self.env.step(int(3.0 / self.env.timestep))

        state = self.env.state()
        # 阈值按"能抓住 10° 级偏航"来定：3 m 行程内偏航 <5°、横漂 <0.25 m
        self.assertLess(abs(state.y), 0.25,
                        "横向漂移过大：y={:.3f} m".format(state.y))
        self.assertLess(abs(state.yaw), math.radians(5.0),
                        "偏航过大：{:.2f}°（直行时航向应基本不变）".format(
                            math.degrees(state.yaw)))

    def test_drive_wheels_have_traction(self):
        """满指令直行 3 s 后，车速应接近指令速度：驱动轮必须有足够的法向力。

        回归背景：四点支撑（前后支撑轮同时着地）是**静不定**的，求解器可能把重量
        几乎全给支撑轮，驱动轮几乎没有法向力 —— 实测车轮空转（滑移 75%）、
        车速只有指令的 25%（指令 2.0 m/s 时实测 0.5 m/s）。
        把车体质心前移到轮轴前方、后支撑轮抬离地面后恢复正常。
        """
        self.env.reset()
        self.env.set_wheel_speeds(0.0, 0.0)
        self.env.step(int(0.6 / self.env.timestep))     # 静置

        self.env.set_wheel_speeds(40.0, 40.0)           # 指令 2.0 m/s
        self.env.step(int(3.0 / self.env.timestep))

        state = self.env.state()
        self.assertGreater(
            state.v, 1.6,
            "车速只有 {:.2f} m/s（指令 2.0 m/s）：驱动轮疑似被架空、牵引力不足".format(state.v))
        self.assertGreater(state.upright, 0.9,
                           "车体姿态异常（{:.3f}）：驱动力未限幅会把车掀翻".format(state.upright))
        self.assertLess(state.z, 0.09,
                        "车身高度 {:.3f} m 偏高：车被驱动指令弹飞了".format(state.z))

    def test_wheel_actuator_force_is_limited(self):
        """驱动轮出力必须限幅（XML 的 forcerange）。

        回归背景：不限幅时，40 rad/s 的阶跃指令会让速度伺服瞬间输出几百牛的力，
        实测 0.05 s 内把 4.6 kg 的车加速到 2.77 m/s、车身冲到 z=0.27 m 并掀翻
        （upright=-1），此后车轮悬空、滑移率 100%。
        """
        limits = self.env.model.actuator_forcerange
        for index, name in ((self.env.actuator_left, "left_motor"),
                            (self.env.actuator_right, "right_motor")):
            upper = float(limits[index][1])
            self.assertGreater(upper, 0.0, "{} 未设置 forcerange".format(name))
            self.assertLessEqual(upper, 10.0,
                                 "{} 的出力上限 {:.1f} N 过大".format(name, upper))

    def test_step_command_does_not_launch_car(self):
        """阶跃给满轮速：车身不应被弹飞，也不应翻车。

        回归背景：v1.6.0 实测 0.05 s 内 z 冲到 0.13 m、0.15 s 到 0.26 m，
        速度在 +2.8 m/s 与 -2.6 m/s 之间反号（车翻了），最终倒扣静止、滑移率 100%。
        """
        self.env.reset()
        self.env.set_wheel_speeds(0.0, 0.0)
        self.env.step(int(0.6 / self.env.timestep))

        self.env.set_wheel_speeds(40.0, 40.0)           # 阶跃给满
        max_z = 0.0
        min_upright = 1.0
        for _ in range(int(1.0 / self.env.timestep)):
            self.env.step(1)
            state = self.env.state()
            max_z = max(max_z, state.z)
            min_upright = min(min_upright, state.upright)

        self.assertLess(max_z, 0.09, "车身最高到 {:.3f} m：被驱动力弹飞了".format(max_z))
        self.assertGreater(min_upright, 0.9, "最低姿态 {:.3f}：车被掀翻了".format(min_upright))

    def test_lane_filter_holds_while_passing_roadside_object(self):
        """贴着路侧干扰物开过去时，不应把它判进本车道。

        回归背景：路侧物早期放在 y=1.4 m、车道半宽取 0.9 m 时，擦着开过去出现过
        横向偏移 0.898 m 的擦线命中（与门限只差 0.002），本车道测距从 4.8 m 突降到
        1.3 m 并触发提前刹停。
        判据：全程"本车道最近距离"应与"到主障碍物的真实间距"一致（相差 < 0.3 m）。
        """
        self.env.reset()
        self.env.set_wheel_speeds(0.0, 0.0)
        self.env.step(int(0.6 / self.env.timestep))

        self.env.set_wheel_speeds(12.0, 12.0)           # 0.6 m/s，开 7.5 s ≈ 4.5 m
        for index in range(int(7.5 / self.env.timestep)):
            self.env.step(1)
            if index % 25:                              # 每 0.05 s 检查一次
                continue
            sensed = self.env.sensed_distance()
            truth = self.env.truth_clearance()
            self.assertIsNotNone(sensed, "本车道测距丢失")
            self.assertGreater(
                sensed, truth - 0.3,
                "t={:.2f}s x={:.2f}m：本车道测距 {:.2f} m 明显小于到主障碍物的真实间距 "
                "{:.2f} m，说明路侧目标被误判进了本车道".format(
                    self.env.time, self.env.state().x, sensed, truth))

    def test_ray_fan_is_immune_to_body_pitch(self):
        """车身俯仰不应让前方障碍物从视场中消失。

        回归背景：射线方向若直接用车身坐标系，实测上仰约 3.7° 时
        射线在 7.5 m 处抬高 0.48 m，正好越过 0.6 m 高的障碍物顶端，
        测距变成 None，决策随即急刹。
        """
        self.env.set_pitch_deg(0.0)
        level = self.env.sensed_distance()
        self.assertIsNotNone(level)

        for angle in (3.0, 4.0, 6.0, -3.0, -6.0):
            self.env.set_pitch_deg(angle)
            value = self.env.sensed_distance()
            self.assertIsNotNone(value, "俯仰 {:+.1f}° 时丢失了前方障碍物".format(angle))
            self.assertAlmostEqual(value, level, delta=0.2,
                                   msg="俯仰 {:+.1f}° 时测距偏差过大".format(angle))

    def test_ground_and_road_are_not_detected(self):
        """下俯时不应把地面/路面当成障碍物。

        回归背景：路面若与障碍物同组，实测下俯约 2.3° 时射线提前打到路面，
        报出 0.11/tan(2.33°) ≈ 2.68 m 的假目标。
        """
        for angle in (-6.0, -3.0, 0.0, 3.0, 6.0):
            self.env.set_pitch_deg(angle)
            ranges = self.env.scan()
            self.assertGreater(float(ranges.min()), 1.0,
                               "俯仰 {:+.1f}° 时测到 {:.2f} m 的目标，疑似打到地面/路面".format(
                                   angle, float(ranges.min())))
            self.assertGreater(self.env.sensed_distance() or 0.0, 6.0)

    def test_car_stays_upright_under_braking(self):
        self.env.reset()
        self.env.set_wheel_speeds(30.0, 30.0)
        self.env.step(int(1.0 / self.env.timestep))
        # 突然给零速：模拟急刹，观察是否点头翻车
        self.env.set_wheel_speeds(0.0, 0.0)
        self.env.step(int(1.0 / self.env.timestep))
        self.assertGreater(self.env.state().upright, 0.8, "急刹时车体姿态异常")


if __name__ == "__main__":
    if not MUJOCO_OK:
        print(MUJOCO_HINT)
        print("安装方式：python3 -m pip install mujoco==3.12.0")
    unittest.main(verbosity=2)
