"""launch 参数文件与节点声明的匹配校验（不依赖 ROS 与 CARLA）。

防止 yaml 顶层键与节点名不匹配导致参数静默回落默认值
（ROS 2 会忽略不匹配节点名的参数段且无任何报错）。
"""

import types
import unittest
from pathlib import Path

import yaml

_PKG = Path(__file__).resolve().parent.parent

# 两个节点的 ROS 参数声明（与 bridge_node.py / ldw_node.py 的 declare_parameter 保持一致）
_DECLARED = {
    "host", "port", "town", "blueprint",               # bridge
    "warn_enter", "warn_exit", "depart_enter",         # alert（决策阈值）
    "depart_exit", "period",
}


class TestParamsFile(unittest.TestCase):

    def setUp(self):
        cfg = yaml.safe_load((_PKG / "config" / "ldw_params.yaml").read_text())
        self.params = cfg["/**"]["ros__parameters"]

    def test_wildcard_top_level_key(self):
        """顶层键必须是 /**（通配两节点）；按单节点名写会使另一节点参数失效。"""
        cfg = yaml.safe_load((_PKG / "config" / "ldw_params.yaml").read_text())
        self.assertIn("/**", cfg)
        self.assertEqual(list(cfg.keys()), ["/**"])

    def test_all_keys_declared_by_nodes(self):
        """yaml 里每个键都必须有节点声明过，否则静默无效。"""
        unknown = set(self.params) - _DECLARED
        self.assertEqual(unknown, set(), f"yaml 中存在无人声明的参数: {unknown}")

    def test_thresholds_match_core_defaults(self):
        """yaml 阈值与决策核心默认值一致（改一处需同步另一处）。"""
        from carla_lane_departure_warning.lane_departure_core import LDWParams
        p = LDWParams()
        for k in ("warn_enter", "warn_exit", "depart_enter", "depart_exit"):
            self.assertEqual(self.params[k], getattr(p, k), k)


if __name__ == "__main__":
    unittest.main()
