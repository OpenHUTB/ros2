#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""airsim_drone_gimbal_teleop 的本地测试（不需要 AirSim / pygame / ROS）.

覆盖：
    A. 按键映射 KEYMAP 完整性
    B. 按键 -> NED 速度指令（含组合键、可配置速度）
    C. 云台更新、俯仰限位与偏航归一化边界
    D. 按下键集合 -> 动作集合（含方向键别名、大小写）
    E. 相机图像方向：display_array 只做 x/y 转置（四角验证，不是旋转）
    F. 退出清理：连接成功后一定降落、上锁、释放 API 控制权
    G. run() 失败路径：缺依赖 / SDL 初始化失败时也会降落
    H. 打包与文档：包目录、entry point、package.xml、README、图片体积
    I. 命令行 --help

用法: python3 tests/test_airsim_drone_gimbal_teleop_local.py
"""
from __future__ import annotations

import argparse
import contextlib
import io
import os
import re
import sys
import types

import numpy as np

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "airsim_drone_gimbal_teleop")
sys.path.insert(0, PKG)

from airsim_drone_gimbal_teleop import main as tg  # noqa: E402


class _Future(object):
    def join(self):
        return None


class _Client(object):
    def __init__(self):
        self.calls = []

    def confirmConnection(self):
        self.calls.append("confirm")

    def enableApiControl(self, enabled):
        self.calls.append(("enableApiControl", enabled))

    def armDisarm(self, armed):
        self.calls.append(("armDisarm", armed))

    def takeoffAsync(self):
        self.calls.append("takeoff")
        return _Future()

    def moveToZAsync(self, z, speed):
        self.calls.append(("moveToZ", z))
        return _Future()

    def landAsync(self):
        self.calls.append("land")
        return _Future()


def make_args(**overrides):
    base = dict(ip="127.0.0.1", port=41451, camera="0", fps=30, width=640,
                height=480, duration=0.1, takeoff=False, altitude=3.0)
    base.update(overrides)
    return argparse.Namespace(**base)


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


# ============================================================ C. 云台与角度
def test_gimbal_and_angles():
    print("== C. 云台更新与角度归一化 ==")
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
    _, yaw = tg.gimbal_from_actions(['gimbal_right'], 0.0, 179.0)
    check(abs(yaw + 179.0) < 1e-9, "179° 向右 -> 归一化到 -179°")

    check(tg.wrap_degrees(180) == 180.0, "180 -> 180（区间上界）")
    check(tg.wrap_degrees(-180) == 180.0, "-180 -> 180（同一方向）")
    check(tg.wrap_degrees(181) == -179.0, "181 -> -179")
    check(tg.wrap_degrees(-181) == 179.0, "-181 -> 179")
    check(abs(tg.wrap_degrees(360.0)) < 1e-9, "360 -> 0")
    check(abs(tg.wrap_degrees(-360.0)) < 1e-9, "-360 -> 0")


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


# ============================================================ E. 图像方向
def test_display_array():
    print("== E. 相机图像方向（只转置，不旋转）==")
    image = np.zeros((2, 3, 3), dtype=np.uint8)
    image[0, 0] = (1, 2, 3)        # 左上
    image[0, 2] = (4, 5, 6)        # 右上
    image[1, 0] = (7, 8, 9)        # 左下
    image[1, 2] = (10, 11, 12)     # 右下
    out = tg.display_array(image)
    check(out.shape == (3, 2, 3), "形状 (H,W,3) -> (W,H,3)")
    check(tuple(out[0, 0]) == (1, 2, 3), "左上角保持左上")
    check(tuple(out[2, 0]) == (4, 5, 6), "右上角映射到 x=W-1（未旋转 90°）")
    check(tuple(out[0, 1]) == (7, 8, 9), "左下角映射到 y=H-1（未旋转 90°）")
    check(tuple(out[2, 1]) == (10, 11, 12), "右下角保持右下")
    check(out.flags["C_CONTIGUOUS"], "输出是 C 连续数组（pygame.surfarray 需要）")
    check(not np.array_equal(out, np.rot90(image)), "结果与 np.rot90 不同（证明没有旋转）")
    try:
        tg.display_array(np.zeros((2, 2), dtype=np.uint8))
        check(False, "非 (H,W,3) 输入应抛 ValueError")
    except ValueError:
        check(True, "非 (H,W,3) 输入抛 ValueError")


# ============================================================ F. 退出清理
def test_shutdown():
    print("== F. 退出清理 ==")
    client = _Client()
    tg._shutdown(client, True, False)
    check("land" in client.calls, "连接成功 -> 调用 landAsync")
    check(("armDisarm", False) in client.calls, "连接成功 -> 上锁 armDisarm(False)")
    check(("enableApiControl", False) in client.calls, "连接成功 -> 释放 API 控制权")

    client2 = _Client()
    tg._shutdown(client2, False, False)
    check("land" not in client2.calls, "未连接成功 -> 不发降落指令")
    check(("enableApiControl", False) in client2.calls, "未连接成功 -> 仍尝试释放控制权")


# ============================================================ G. 失败路径
def test_run_failures():
    print("== G. run() 失败路径仍会降落 ==")
    original = {name: sys.modules.get(name) for name in ("airsim", "pygame")}
    try:
        # G1：连接成功但 pygame 初始化失败（无 DISPLAY / SDL 报错）
        client = _Client()
        fake_airsim = types.ModuleType("airsim")
        fake_airsim.MultirotorClient = lambda ip, port: client
        fake_pygame = types.ModuleType("pygame")

        def _boom():
            raise RuntimeError("No available video device")

        fake_pygame.init = _boom
        fake_pygame.quit = lambda: None
        sys.modules["airsim"] = fake_airsim
        sys.modules["pygame"] = fake_pygame

        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            code = tg.run(make_args())
        check(code == 1, "SDL 初始化失败 -> 返回码 1")
        check("land" in client.calls, "SDL 初始化失败 -> 仍执行降落")
        check(("armDisarm", False) in client.calls, "SDL 初始化失败 -> 仍上锁")
        check("No available video device" in buffer.getvalue(), "打印了 SDL 失败原因")

        # G2：缺少 pygame 依赖
        sys.modules["pygame"] = None
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            code = tg.run(make_args())
        check(code == 1, "缺少 pygame -> 返回码 1")
        check("pip3 install airsim pygame numpy" in buffer.getvalue(), "提示安装依赖")
    finally:
        for name, module in original.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


# ============================================================ H. 打包与文档
def test_packaging_and_docs():
    print("== H. 打包与文档 ==")
    check(callable(getattr(tg, "main", None)), "模块提供 main() 入口")
    package_main = os.path.join(PKG, "airsim_drone_gimbal_teleop", "main.py")
    init_py = os.path.join(PKG, "airsim_drone_gimbal_teleop", "__init__.py")
    check(os.path.isfile(package_main), "源码位于标准包目录 airsim_drone_gimbal_teleop/main.py")
    check(os.path.isfile(init_py), "包目录有 __init__.py")
    check(not os.path.isfile(os.path.join(PKG, "main.py")), "顶层不再有 main.py（避免装进 site-packages）")

    setup_py = open(os.path.join(PKG, "setup.py"), encoding="utf-8").read()
    check("packages=[package_name]" in setup_py, "setup.py 用 packages=[...] 安装包目录")
    check("py_modules" not in setup_py, "setup.py 不再用 py_modules 装顶层模块")
    check("drone_gimbal_teleop = airsim_drone_gimbal_teleop.main:main" in setup_py,
          "entry point 指向 airsim_drone_gimbal_teleop.main:main")
    check("example.com" not in setup_py, "setup.py 不再用占位邮箱")

    package_xml = open(os.path.join(PKG, "package.xml"), encoding="utf-8").read()
    check("<buildtool_depend>ament_python</buildtool_depend>" in package_xml,
          "package.xml 的 buildtool 为 ament_python")
    check("ament_cmake" not in package_xml, "package.xml 不再混用 ament_cmake")
    check("example.com" not in package_xml, "package.xml 不再用占位邮箱")

    readme_path = os.path.join(ROOT, "docs", "airsim_drone_gimbal_teleop", "README.md")
    readme = open(readme_path, encoding="utf-8").read()
    demo_path = os.path.join(ROOT, "docs", "airsim_drone_gimbal_teleop", "demo.png")
    check("![运行效果](demo.png)" in readme, "README 引用 demo.png")
    check(os.path.isfile(demo_path), "demo.png 存在")
    check(os.path.getsize(demo_path) < 1000 * 1024, "demo.png 已压缩到 1 MB 以内")
    check(readme.count("```") % 2 == 0, "README 代码围栏成对闭合")
    check(re.search(r"(?m)^[ \t]+##\s", readme) is None, "README 没有缩进的标题")
    for key in ("`W` / `S`", "`A` / `D`", "`Q` / `E`", "`Z` / `X`",
                "`I` / `K`", "`J` / `L`", "`ESC`"):
        check(key in readme, "按键表包含 %s" % key)
    check("--takeoff" in readme and "--ip" in readme, "README 说明常用参数")
    check("display_array" in readme, "README 说明图像方向处理")

    index_md = open(os.path.join(ROOT, "docs", "index.md"), encoding="utf-8").read()
    check("airsim_drone_gimbal_teleop" in index_md, "docs/index.md 有模块入口")
    mkdocs = open(os.path.join(ROOT, "mkdocs.yml"), encoding="utf-8").read()
    check("airsim_drone_gimbal_teleop" in mkdocs, "mkdocs.yml 导航有模块入口")


# ============================================================ I. 命令行
def test_cli():
    print("== I. 命令行 ==")
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        try:
            tg.main(["--help"])
        except SystemExit as exc:
            code = exc.code
    check(code == 0, "--help 正常退出（返回码 0）")
    text = buffer.getvalue()
    check("--ip" in text and "--takeoff" in text, "--help 列出参数")


def main():
    test_keymap()
    test_command()
    test_gimbal_and_angles()
    test_actions()
    test_display_array()
    test_shutdown()
    test_run_failures()
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
