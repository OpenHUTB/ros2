#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""配置与参数接线的回归测试（不需要 ROS / CARLA）。

这三类问题都不会让程序报错，只会让结论悄悄失真，所以专门用测试钉住：

1. `tools/run_on_carla.py` 里车道宽度的尺度。标定模式下米/像素约 0.0080，
   而 `LaneFitConfig.meters_per_pixel_x` 的 dataclass 默认值是 0.05；
   用后者去算会把 3.7 m 的车道算成 23 m，合理性门限把每一帧都拒掉，
   "通过合理性检查的帧数"就恒为 0。
2. `launch/main.launch` 必须把控制参数载入控制器节点的私有命名空间。
   只载入 `/lane_detector` 的话，控制器读不到 `~kp`，改 yaml 不生效。
3. 检测段与控制段必须分属两个 yaml：混在一份里就又会退化成只载一次。

运行::

    python -m unittest discover -s tests -v
"""

import os
import re
import sys
import unittest
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)
PKG_SCRIPTS = os.path.join(PKG_ROOT, "scripts")
PKG_TOOLS = os.path.join(PKG_ROOT, "tools")
LAUNCH = os.path.join(PKG_ROOT, "launch", "main.launch")
DETECTOR_YAML = os.path.join(PKG_ROOT, "config", "lane_params.yaml")
CONTROL_YAML = os.path.join(PKG_ROOT, "config", "control_params.yaml")


def yaml_top_keys(path):
    """取 YAML 的顶层键（不引入 PyYAML：这些文件只有一层键值）。"""
    keys = set()
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.rstrip("\n")
            if not line or line.lstrip().startswith("#") or line.startswith(" "):
                continue
            m = re.match(r"^([A-Za-z_][A-Za-z0-9_]*):", line)
            if m:
                keys.add(m.group(1))
    return keys


class _Region:
    def __init__(self, mpp_x):
        self.mpp_x = mpp_x


class _Detector:
    def __init__(self, mpp_x=None):
        self.region = None if mpp_x is None else _Region(mpp_x)


class _Result:
    def __init__(self, lane_width_m=None, left_fit=None, right_fit=None, v_eval=10.0):
        self.lane_width_m = lane_width_m
        self.left_fit = left_fit
        self.right_fit = right_fit
        self.v_eval = v_eval


class TestLaneWidthScale(unittest.TestCase):
    """tools/run_on_carla.py 算车道宽度时必须用检测器的那套尺度。"""

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, PKG_SCRIPTS)
        sys.path.insert(0, PKG_TOOLS)
        import run_on_carla  # noqa: E402

        cls.mod = run_on_carla
        cls.cfg = run_on_carla.LaneFitConfig()

    def test_prefers_detector_result(self):
        res = _Result(lane_width_m=3.72)
        self.assertAlmostEqual(self.mod.lane_width_m(res, _Detector(0.0080), self.cfg), 3.72)

    def test_fallback_uses_region_mpp_not_dataclass_default(self):
        """没有 lane_width_m 时退回 region.mpp_x，而不是 cfg 的 0.05。"""
        res = _Result(lane_width_m=None, left_fit=[0.0, 100.0], right_fit=[0.0, 560.0])
        got = self.mod.lane_width_m(res, _Detector(0.0080), self.cfg)
        # 460 px * 0.0080 = 3.68 m；若误用默认 0.05 会得到 23 m
        self.assertAlmostEqual(got, 460.0 * 0.0080)
        self.assertLess(got, 8.0)

    def test_returns_none_without_fits(self):
        res = _Result(lane_width_m=None, left_fit=None, right_fit=None)
        self.assertIsNone(self.mod.lane_width_m(res, _Detector(0.0080), self.cfg))


class TestLaunchWiring(unittest.TestCase):
    """launch 必须让控制器真正拿到它的私有参数。"""

    def setUp(self):
        self.tree = ET.parse(LAUNCH)
        self.root = self.tree.getroot()

    def _node(self, type_name):
        for node in self.root.findall("node"):
            if node.get("type") == type_name:
                return node
        self.fail("launch 里没有 %s" % type_name)

    def _resolve(self, ref):
        """把 $(arg xxx) 换成该 arg 的默认值，便于断言最终指向哪个文件。"""
        m = re.match(r"^\$\(arg ([A-Za-z0-9_]+)\)$", ref or "")
        if not m:
            return ref or ""
        for a in self.root.findall("arg"):
            if a.get("name") == m.group(1):
                return a.get("default", "")
        return ""

    def test_controller_node_loads_control_params(self):
        node = self._node("lane_keeping_controller.py")
        files = [
            self._resolve(p.get("file"))
            for p in node.findall("rosparam")
            if p.get("command") == "load"
        ]
        self.assertTrue(
            any("control_params.yaml" in f for f in files),
            "控制器节点内没有载入 control_params.yaml，~kp 等参数会读不到默认值以外的配置",
        )

    def test_detector_params_loaded_into_detector_ns(self):
        found = False
        for p in self.root.findall("rosparam"):
            if p.get("command") != "load":
                continue
            if "lane_params.yaml" in self._resolve(p.get("file")) and p.get("ns") == "lane_detector":
                found = True
        self.assertTrue(found, "顶层没有把 lane_params.yaml 载入 /lane_detector")


class TestParamFiles(unittest.TestCase):
    """两份 yaml 的职责边界。"""

    def test_control_params_cover_controller_get_param(self):
        with open(os.path.join(PKG_SCRIPTS, "lane_keeping_controller.py"), encoding="utf-8") as fh:
            src = fh.read()
        wanted = set(re.findall(r'get_param\(\s*"~([A-Za-z_][A-Za-z0-9_]*)"', src))
        self.assertTrue(wanted, "没有从控制器源码里解析到 ~ 参数，正则可能失效了")
        missing = sorted(wanted - yaml_top_keys(CONTROL_YAML))
        self.assertEqual(missing, [], "control_params.yaml 缺少控制器要读的参数：%s" % missing)

    # 两个节点各有一个自己的置信度门限，同名但含义独立，属已知的合理重叠
    SHARED_OK = {"min_confidence"}

    def test_detector_and_control_yaml_are_disjoint(self):
        det = yaml_top_keys(DETECTOR_YAML)
        ctl = yaml_top_keys(CONTROL_YAML)
        both = sorted((det & ctl) - self.SHARED_OK)
        self.assertEqual(
            both, [], "检测段与控制段出现同名键 %s，说明两份文件又混在一起了" % both
        )

    def test_control_gains_not_in_detector_yaml(self):
        det = yaml_top_keys(DETECTOR_YAML)
        for key in ("kp", "ki", "kd", "k_psi", "target_speed"):
            self.assertNotIn(key, det, "控制参数 %s 又跑回检测段了" % key)


if __name__ == "__main__":
    unittest.main(verbosity=2)
