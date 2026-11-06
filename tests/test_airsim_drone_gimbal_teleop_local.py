#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""airsim_drone_gimbal_teleop 的本地测试（不需要 AirSim / pygame / ROS）.

覆盖：
    A. 按键映射 KEYMAP 完整性
    B. 按键 -> NED 速度指令（含组合键、可配置速度）
    C. 云台俯仰/偏航更新与限位
    D. 按下键集合 -> 动作集合（含方向键别名、大小写）
    E. 打包与文档：入口函数、console_script、package.xml、README 图片与代码围栏
    F. 命令行：--help 正常退出；模块导入不依赖 pygame/airsim

用法: python3 tests/test_airsim_drone_gimbal_teleop_local.py
"""
from __future__ import annotations

import contextlib
import io
import os
import re
import sys

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "airsim_drone_gimbal_teleop")
sys.path.insert(0, PKG)

import main as tg  # noqa: E402


# ============================================================ A. 按键映射
def test_keymap():
    print("== A. 按键映射 ==")
    expected = set('wsadqezxikjl')
    check(set(tg.KEYMAP) == expected, "KEYMAP 覆盖 W/S/A/D/Q/E/Z/X/I/K/J/L")
    check(len(set(tg.KEYMAP.values())) == len(tg.KEYMAP), "每个按键对应不同动作")
    check(tg.KEYMAP['w'] == 'forward' and tg.KEYMAP['s'] == 'backward', "W/S -> 前进/后退")
    check(tg.KEYMAP['a'] == 'left' and tg.KEYMAP['d'] == 'right', "A/D -> 左移/右移")
    check(tg.KEYMAP['q'] == 'up' and tg.KEYMAP['e'] == 'down', "Q/E -> 上升/下降")
    check(tg.KEYMAP['z'] == 'yaw_left' and tg.KEYMAP['x'] == 'yaw_right', "Z/X -> 左转/右转")
    check(tg.KEYMAP['i'] == 'gimbal_up' and tg.KEYMAP['k'] == 'gimbal_down', "I/K -> 云台俯仰")
    check(tg.KEYMAP['j'] == 'gimbal_left' and tg.KEYMAP['l'] == 'gimbal_right', "J/L -> 云台偏航")


# ============================================================ B. 速度指令
def test_command():
    print("== B. 按键 -> NED 速度指令 ==")
    check(tg.command_from_actions(['forward']) == (3.0, 0.0, 0.0, 0.0), "前进 -> vx=+3")
    check(tg.command_from_actions(['backward']) == (-3.0, 0.0, 0.0, 0.0), "后退 -> vx=-3")
    check(tg.command_from_actions(['left']) == (0.0, -2.0, 0.0, 0.0), "左移 -> vy=-2")
    check(tg.command_from_actions(['right']) == (0.0, 2.0, 0.0, 0.0), "右移 -> vy=+2")
    check(tg.command_from_actions(['up']) == (0.0, 0.0, -1.5, 0.0), "上升 -> vz=-1.5（NED 向上为负）")
    check(tg.command_from_actions(['down']) == (0.0, 0.0, 1.5, 0.0), "下降 -> vz=+1.5")
    check(tg.command_from_actions(['yaw_left'])[3] == -45.0, "左转 -> 负偏航角速度")
    check(tg.command_from_actions(['yaw_right'])[3] == 45.0, "右转 -> 正偏航角速度")
    check(tg.command_from_actions(['forward', 'right']) == (3.0, 2.0, 0.0, 0.0), "组合键叠加")
    check(tg.command_from_actions([]) == (0.0, 0.0, 0.0, 0.0), "无按键 -> 全零（悬停）")
    check(tg.command_from_actions(['forward'], forward_speed=1.0) == (1.0, 0.0, 0.0, 0.0),
          "速度系数可配置")
    check(tg.command_from_actions(['yaw_left'], yaw_rate=90.0)[3] == -90.0, "偏航角速度可配置")


# ============================================================ C. 云台
def test_gimbal():
    print("== C. 云台更新与限位 ==")
    pitch, yaw = tg.gimbal_from_actions(['gimbal_up'], 0.0, 0.0)
    check((pitch, yaw) == (2.0, 0.0), "I -> 俯仰 +2°")
    pitch, yaw = tg.gimbal_from_actions(['gimbal_down'], 0.0, 0.0)
    check((pitch, yaw) == (-2.0, 0.0), "K -> 俯仰 -2°")
    pitch, _ = tg.gimbal_from_actions(['gimbal_up'], 89.0, 0.0)
    check(pitch == 90.0, "俯仰上限 +90°")
    pitch, _ = tg.gimbal_from_actions(['gimbal_down'], -89.0, 0.0)
    check(pitch == -90.0, "俯仰下限 -90°")
    _, yaw = tg.gimbal_from_actions(['gimbal_left'], 0.0, 0.0)
    check(yaw == -2.0, "J -> 偏航 -2°")
    _, yaw = tg.gimbal_from_actions(['gimbal_right'], 0.0, 0.0)
    check(yaw == 2.0, "L -> 偏航 +2°")
    _, yaw = tg.gimbal_from_actions(['gimbal_right'], 0.0, 179.0)
    check(abs(yaw + 179.0) < 1e-9, "偏航超过 180° 后归一化到 -179°")
    _, yaw = tg.gimbal_from_actions(['gimbal_left'], 0.0, -179.0)
    check(abs(yaw - 179.0) < 1e-9, "偏航低于 -180° 后归一化到 +179°")
    pitch, _ = tg.gimbal_from_actions(['gimbal_up'], 0.0, 0.0, step=10.0)
    check(pitch == 10.0, "步长可配置")


# ============================================================ D. 动作集合
def test_actions():
    print("== D. 按下键 -> 动作集合 ==")
    check(tg.actions_from_pressed({'w', 'i'}) == {'forward', 'gimbal_up'}, "{w,i} -> 前进+云台上")
    check(tg.actions_from_pressed({'up'}) == {'gimbal_up'}, "方向上键 -> 云台上（别名）")
    check(tg.actions_from_pressed({'left', 'right'}) == {'gimbal_left', 'gimbal_right'},
          "方向左右 -> 云台偏航")
    check(tg.actions_from_pressed({'W'}) == {'forward'}, "大写字母也能识别")
    check(tg.actions_from_pressed({'p'}) == set(), "未映射按键被忽略")
    check(tg.actions_from_pressed(set()) == set(), "空集合 -> 无动作")


# ============================================================ E. 打包与文档
def test_packaging_and_docs():
    print("== E. 打包与文档 ==")
    check(callable(getattr(tg, 'main', None)), "模块提供 main() 入口")
    check(callable(getattr(tg, 'run', None)), "模块提供 run() 主循环")

    setup_py = open(os.path.join(PKG, 'setup.py'), encoding='utf-8').read()
    check('drone_gimbal_teleop = main:main' in setup_py, "setup.py 的 console_scripts 指向 main:main")
    check("py_modules=['main']" in setup_py, "setup.py 以 py_modules 安装 main.py")

    package_xml = open(os.path.join(PKG, 'package.xml'), encoding='utf-8').read()
    check('<build_type>ament_python</build_type>' in package_xml, "package.xml 构建类型为 ament_python")
    check('python3-setuptools' in package_xml, "package.xml 使用 python3-setuptools 作为 buildtool")
    check('ament_cmake' not in package_xml, "package.xml 不再混用 ament_cmake")

    readme_path = os.path.join(ROOT, 'docs', 'airsim_drone_gimbal_teleop', 'README.md')
    readme = open(readme_path, encoding='utf-8').read()
    demo_path = os.path.join(ROOT, 'docs', 'airsim_drone_gimbal_teleop', 'demo.png')
    check('![运行效果](demo.png)' in readme, "README 引用 demo.png")
    check(os.path.isfile(demo_path), "demo.png 确实存在")
    check(readme.count('```') % 2 == 0, "README 代码围栏成对闭合")
    check(re.search(r'(?m)^[ \t]+##\s', readme) is None, "README 没有缩进的标题（否则会渲染进列表/代码块）")
    for key in ('`W` / `S`', '`A` / `D`', '`Q` / `E`', '`Z` / `X`',
                '`I` / `K`', '`J` / `L`', '`ESC`'):
        check(key in readme, "按键表包含 %s" % key)
    check('--takeoff' in readme and '--ip' in readme, "README 说明常用参数")

    index_md = open(os.path.join(ROOT, 'docs', 'index.md'), encoding='utf-8').read()
    check('airsim_drone_gimbal_teleop' in index_md, "docs/index.md 有模块入口")
    mkdocs = open(os.path.join(ROOT, 'mkdocs.yml'), encoding='utf-8').read()
    check('airsim_drone_gimbal_teleop' in mkdocs, "mkdocs.yml 导航有模块入口")


# ============================================================ F. 命令行
def test_cli():
    print("== F. 命令行 ==")
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        try:
            tg.main(['--help'])
        except SystemExit as exc:
            code = exc.code
    check(code == 0, "--help 正常退出（返回码 0）")
    text = buffer.getvalue()
    check('--ip' in text and '--takeoff' in text, "--help 列出参数")


def main():
    test_keymap()
    test_command()
    test_gimbal()
    test_actions()
    test_packaging_and_docs()
    test_cli()
    print("\n==================== 结果 ====================")
    print("通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    if FAIL:
        for item in FAIL:
            print("  FAIL: %s" % item)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
