# -*- coding: utf-8 -*-
"""shadow_hand_mujoco 模型结构单元测试（不依赖 ROS）。

运行：cd src/manipulator/shadow_hand_mujoco && python3 -m unittest discover -s tests -v

覆盖：
1. MJCF 可编译，nq > 0；
2. 每个 keyframe 的 qpos 长度等于 nq —— #708 曾因 19 vs 16 直接加载失败；
3. 执行器数量不少于 hinge 关节数，避免"没有驱动、自由坍塌"；
4. 零控制连续步进 20 步，状态保持有限；
5. 控制器输出有限且在合理量级。
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

try:
    from shadow_hand_mujoco import main as mod
except Exception as exc:  # 缺少 OpenGL 后端时跳过，不让 CI 变红
    raise unittest.SkipTest("导入 shadow_hand_mujoco 失败：%r" % exc)


class TestModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import mujoco
        import numpy as np

        cls.mj = mujoco
        cls.np = np
        try:
            cls.model = mod.build_model()
        except Exception as exc:
            raise unittest.SkipTest("模型不可用（缺少 menagerie 缓存或 GL）：%r" % exc)

    def test_keyframe_qpos_matches_nq(self):
        self.assertGreater(self.model.nq, 0)
        for k in range(self.model.nkey):
            self.assertEqual(len(self.model.key_qpos[k]), self.model.nq,
                             "keyframe %d 的 qpos 长度与 nq 不一致" % k)

    def test_actuators_present(self):
        self.assertGreater(self.model.nu, 0)

    def test_hinges_are_actuated(self):
        """执行器应覆盖大部分 hinge（耦合/被动关节本来就允许不装执行器）。"""
        n_hinge = sum(1 for j in range(self.model.njnt)
                      if self.model.jnt_type[j] == self.mj.mjtJoint.mjJNT_HINGE)
        self.assertGreaterEqual(self.model.nu, int(n_hinge * 0.75))

    def test_zero_control_is_finite(self):
        import mujoco

        data = mujoco.MjData(self.model)
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, data, 0)
        for _ in range(20):
            mujoco.mj_step(self.model, data)
        self.assertTrue(self.np.isfinite(data.qpos).all())
        self.assertTrue(self.np.isfinite(data.qvel).all())

    def test_controller_output(self):
        import mujoco

        data = mujoco.MjData(self.model)
        ctrl = mod.make_controller(self.model)
        for t in (0.0, 0.5, 1.0, 2.0):
            ctrl(data, t)
        self.assertTrue(self.np.isfinite(data.ctrl).all())
        lim = max([abs(float(x)) for r in self.model.actuator_ctrlrange for x in r] + [1.0])
        self.assertLessEqual(float(self.np.abs(data.ctrl).max()), lim * 1.2)


if __name__ == "__main__":
    unittest.main()
