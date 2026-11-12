"""mujoco_multi_goal_nav 的最小回归测试（不依赖 ROS 运行时，只需要 mujoco + numpy）。

运行方式（二选一）：

    cd src/ground/mujoco_multi_goal_nav
    python3 -m unittest discover -s test -v

    # 或
    python3 -m pytest test/test_main.py -v
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import main as nav  # noqa: E402


class TestConstants(unittest.TestCase):
    def test_goals_count(self):
        self.assertEqual(len(nav.GOALS), 3)

    def test_goals_coordinates(self):
        self.assertEqual(nav.GOALS[0], (1.5, 1.0))
        self.assertEqual(nav.GOALS[1], (-1.5, 1.0))
        self.assertEqual(nav.GOALS[2], (-1.5, -1.0))

    def test_params(self):
        self.assertTrue(0 < nav.SPEED <= 1.0)
        self.assertTrue(0 < nav.STOP_DIST < 0.5)
        self.assertGreater(nav.SLOW_RADIUS, nav.STOP_DIST)
        self.assertGreater(nav.Z0, 0)


class TestScenePath(unittest.TestCase):
    def test_default_path_exists(self):
        path = nav.resolve_scene_path()
        self.assertTrue(path.endswith("scene.xml"))
        self.assertTrue(os.path.isfile(path))

    def test_env_var_wins(self):
        import tempfile

        with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as f:
            f.write("<mujoco><worldbody/></mujoco>")
            alt = f.name
        old = os.environ.get("MUJOCO_MULTI_GOAL_SCENE")
        try:
            os.environ["MUJOCO_MULTI_GOAL_SCENE"] = alt
            self.assertEqual(nav.resolve_scene_path(), os.path.abspath(alt))
        finally:
            if old is None:
                os.environ.pop("MUJOCO_MULTI_GOAL_SCENE", None)
            else:
                os.environ["MUJOCO_MULTI_GOAL_SCENE"] = old
            os.unlink(alt)


class TestScene(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import mujoco

        cls.mujoco = mujoco
        cls.model = mujoco.MjModel.from_xml_path(nav.resolve_scene_path())
        cls.data = mujoco.MjData(cls.model)

    def test_geom_names(self):
        names = [
            self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_GEOM, i)
            for i in range(self.model.ngeom)
        ]
        for g in ("ground", "goal1", "goal2", "goal3", "chassis"):
            self.assertIn(g, names)

    def test_goal_markers_are_non_colliding(self):
        for i in range(self.model.ngeom):
            name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_GEOM, i)
            if name and name.startswith("goal"):
                self.assertEqual(self.model.geom_contype[i], 0)
                self.assertEqual(self.model.geom_conaffinity[i], 0)

    def test_lock_pose_resets_state(self):
        self.data.qpos[2] = 0.9
        self.data.qpos[3:7] = (0.5, 0.5, 0.5, 0.5)
        self.data.qvel[2] = 3.0
        self.data.qvel[3:6] = 1.0
        nav.lock_pose(self.data)
        self.assertAlmostEqual(self.data.qpos[2], nav.Z0, places=9)
        np.testing.assert_allclose(self.data.qpos[3:7], (1.0, 0.0, 0.0, 0.0))
        self.assertEqual(self.data.qvel[2], 0.0)
        np.testing.assert_allclose(self.data.qvel[3:6], (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
