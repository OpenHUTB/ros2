# -*- coding: utf-8 -*-
"""HoloOcean ROS2 文档与源码的一致性检查。

这个测试不 import ROS 2 / HoloOcean，只用 AST 读 ``holoocean_node.py``，
检查文档是否漏掉节点实际声明的话题、服务和参数。改代码或改文档后运行：

    python tests/test_holoocean_docs.py
"""

from __future__ import annotations

import ast
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "src/water/holoocean_main/holoocean_main/holoocean_node.py"
DOC = ROOT / "docs/water/HoloOcean.md"
WATER = ROOT / "src/water"


def _const_str(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _call_name(node):
    func = node.func
    if isinstance(func, ast.Attribute):
        return func.attr
    if isinstance(func, ast.Name):
        return func.id
    return None


def extract_node_api():
    """返回 (parameters, subscriptions, publishers, services)。"""
    tree = ast.parse(NODE.read_text(encoding="utf-8"))
    params, subs, pubs, srvs = [], [], [], []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = _call_name(node)
        if name == "declare_parameter" and node.args:
            value = _const_str(node.args[0])
            if value:
                params.append(value)
        elif name in ("create_subscription", "create_publisher",
                      "create_service") and len(node.args) >= 2:
            value = _const_str(node.args[1])
            if value:
                (subs if name == "create_subscription" else
                 pubs if name == "create_publisher" else srvs).append(value)
    return params, subs, pubs, srvs


class HoloOceanDocsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # 允许在只检出 docs/ 或只检出 src/water/ 的环境里跳过，而不是报
        # FileNotFoundError；完整仓库里才做一致性检查。
        if not NODE.exists() or not DOC.exists():
            raise unittest.SkipTest("need docs/water/HoloOcean.md and "
                                    "src/water/holoocean_main")
        cls.doc = DOC.read_text(encoding="utf-8")
        cls.params, cls.subs, cls.pubs, cls.srvs = extract_node_api()

    def test_node_parameters_documented(self):
        for name in self.params:
            self.assertIn("`%s`" % name, self.doc,
                          "文档缺少参数 %s" % name)

    def test_topics_documented(self):
        for topic in self.subs + self.pubs:
            self.assertIn("`%s`" % topic, self.doc,
                          "文档缺少话题 %s" % topic)

    def test_services_documented(self):
        for service in self.srvs:
            self.assertIn("`%s`" % service, self.doc,
                          "文档缺少服务 %s" % service)

    def test_packages_documented(self):
        names = []
        for package_xml in sorted(WATER.glob("*/package.xml")):
            text = package_xml.read_text(encoding="utf-8")
            match = re.search(r"<name>([^<]+)</name>", text)
            if match:
                names.append(match.group(1))
        self.assertTrue(names)
        for name in names:
            if name.startswith("holoocean_"):
                self.assertIn("`%s`" % name, self.doc,
                              "文档缺少软件包 %s" % name)

    def test_launch_files_documented_and_exist(self):
        expected = {
            "holoocean_launch.py": WATER / "holoocean_main/launch",
            "joy_launch.py": WATER / "holoocean_examples/launch",
            "waypoint_launch.py": WATER / "holoocean_examples/launch",
            "command_launch.py": WATER / "holoocean_examples/launch",
        }
        for name, directory in expected.items():
            self.assertTrue((directory / name).exists(), name)
            self.assertIn(name, self.doc, "文档缺少 launch 文件 %s" % name)

    def test_joy_linux_package_name(self):
        # joy_launch.py / package.xml 用的是 joy_linux，不是 joy
        self.assertIn("ros-humble-joy-linux", self.doc)
        self.assertNotIn("ros-humble-joy ", self.doc)
        self.assertIn("`joy_linux`", self.doc)

    def test_agent_command_frame_id_is_agent_name(self):
        # holoocean_node.py 把 header.frame_id 直接当代理名；
        # Fossen 代理只有代理名命中 fossen_agents 才调用 set_u_control。
        self.assertIn("`frame_id` 必须是代理名", self.doc)
        self.assertNotIn("`frame_id` 设为 `body`", self.doc)

    def test_draw_waypoints_marked_unused(self):
        line = next(ln for ln in self.doc.splitlines()
                    if "`draw_waypoints`" in ln)
        self.assertIn("预留参数，当前未使用", line)


if __name__ == "__main__":
    unittest.main(verbosity=2)
