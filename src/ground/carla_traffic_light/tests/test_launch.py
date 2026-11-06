# -*- coding: utf-8 -*-
"""launch / package.xml 与代码是否对得上的检查。

这两个文件不参与 Python 逻辑，但错了会让「配置看起来生效、实际不生效」：
``scripts/main.py`` 是纯 argparse 脚本，不 import rospy，因此 launch 里写
``<param>`` / ``<rosparam>`` 没有任何代码去读，参数只能经 ``args`` 传给命令行。
"""

from __future__ import annotations

import os
import re
import unittest
import xml.etree.ElementTree as ET

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_DIR = os.path.join(PKG, "launch")
SCRIPTS_DIR = os.path.join(PKG, "scripts")

# $(arg xxx) —— 只匹配 arg 引用，$(find ...) 属于包路径不在此列
_ARG_REF = re.compile(r"\$\(arg\s+([A-Za-z_][A-Za-z0-9_]*)\)")


def _parse(name):
    return ET.parse(os.path.join(LAUNCH_DIR, name)).getroot()


def _nodes(root):
    return root.findall("node")


class LaunchWiringTest(unittest.TestCase):

    def test_main_launch_passes_carla(self):
        """main.launch 必须真的以 --carla 启动，否则跑的是离线分支。"""
        nodes = _nodes(_parse("main.launch"))
        self.assertEqual(len(nodes), 1)
        args = nodes[0].get("args") or ""
        self.assertIn("--carla", args.split(),
                      "main.launch 没传 --carla，roslaunch 起来会走离线分支")

    def test_launch_args_all_defined(self):
        """args 里引用的每个 $(arg x) 都得有对应的 <arg> 声明，否则 roslaunch 直接报错。"""
        for name in ("main.launch", "offline.launch"):
            root = _parse(name)
            declared = {a.get("name") for a in root.findall("arg")}
            for node in _nodes(root):
                for ref in _ARG_REF.findall(node.get("args") or ""):
                    self.assertIn(ref, declared,
                                  "%s: 引用了未声明的 $(arg %s)" % (name, ref))

    def test_no_unread_ros_params(self):
        """代码不读 ROS param，launch 里就不该有 <param> / <rosparam>。"""
        for name in ("main.launch", "offline.launch"):
            root = _parse(name)
            self.assertEqual(root.findall(".//param"), [],
                             "%s: <param> 没有代码读取，应改为 args" % name)
            self.assertEqual(root.findall(".//rosparam"), [],
                             "%s: <rosparam> 没有代码读取，应改为 --params" % name)


class PackageXmlTest(unittest.TestCase):

    def test_no_unused_ros_dependencies(self):
        """声明的 ROS 依赖必须在代码里真的用到。

        脚本里没有 import rospy，也没有任何消息类型，所以 rospy / std_msgs /
        geometry_msgs 都不该出现在 package.xml 里。
        """
        root = ET.parse(os.path.join(PKG, "package.xml")).getroot()
        declared = {e.text.strip() for e in root
                    if e.tag.endswith("depend") and e.text}

        used = set()
        for fn in os.listdir(SCRIPTS_DIR):
            if not fn.endswith(".py"):
                continue
            with open(os.path.join(SCRIPTS_DIR, fn), encoding="utf-8") as f:
                src = f.read()
            for mod in ("rospy", "std_msgs", "geometry_msgs"):
                if re.search(r"^\s*(?:import|from)\s+%s\b" % mod, src, re.M):
                    used.add(mod)

        for mod in ("rospy", "std_msgs", "geometry_msgs"):
            if mod in declared:
                self.assertIn(mod, used,
                              "package.xml 声明了 %s 但 scripts/ 里没用到" % mod)


if __name__ == "__main__":
    unittest.main()
