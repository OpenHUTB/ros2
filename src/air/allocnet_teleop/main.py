#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AllocNet 四旋翼避障规划 · 键盘遥操作模块的统一入口。

对应仓库约定「每个模块必须支持 launch 启动，入口为 main. 开头」。

本模块不包含 AllocNet 规划器本身（它依赖 libtorch / ompl / osqp-eigen，
需要单独编译，见 docs/air/allocnet_teleop.md 的环境搭建章节）。
本模块提供的是**人机交互层**：键盘下发航点、地图转发、数据记录、
轨迹可视化，以及演示素材采集。

用法
====
    # 1. 把本模块的文件装入 AllocNet 工作区的 planner 包（只需一次）
    python3 main.py install --ws ~/allocnet_ws

    # 2. 环境自检：确认依赖与前置条件是否满足
    python3 main.py check

    # 3. 一键启动完整仿真（等价于 roslaunch planner teleop_planning.launch）
    python3 main.py run

    # 4. 无图形界面启动（服务器 / Xvfb 虚屏）
    python3 main.py run --no-gui

    # 5. 采集演示素材（需要 Xvfb，见文档）
    python3 main.py capture

    # 6. 查看常用命令速查
    python3 main.py help

为什么需要 install
==================
launch 文件通过 `pkg="planner"` 引用节点脚本，因此这些脚本必须位于
AllocNet 工作区的 `src/AllocNet/src/planner/` 包内才能被 ROS 找到。
本模块是这些文件的**源**，install 负责把它们就位。
"""
import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# 本模块依赖的上游 ROS 包（由 AllocNet 工作区提供）
REQUIRED_PKGS = {
    "planner": "AllocNet 规划器包（含 learning_planning 可执行文件）",
    "param_env": "地图生成包（structure_map / read_grid_map）",
}

# 本模块自带的脚本
OWN_SCRIPTS = [
    "teleop_keyboard.py",
    "map_republisher.py",
    "record_trajectory.py",
]


def _sh(cmd):
    """执行命令并返回 (rc, stdout+stderr)。

    ⚠ 必须指定 bash：`shell=True` 在 Ubuntu 上默认走 /bin/sh（dash），
    而 ROS 的 setup.bash 依赖 bash 语法（数组、`${VAR:-}` 展开等），
    用 dash 会报 "Bad substitution" 且静默失败。
    """
    kwargs = dict(shell=True, capture_output=True, text=True)
    if os.path.exists("/bin/bash"):
        kwargs["executable"] = "/bin/bash"
    p = subprocess.run(cmd, **kwargs)
    return p.returncode, (p.stdout or "") + (p.stderr or "")


def _ros_run(cmd, ws=None):
    """在一个 source 好 ROS 环境的 bash 里执行命令。

    ROS 的 setup.bash 会引用未定义变量，外层若开了 `set -u` 会静默杀脚本，
    因此在子 shell 里显式先 `set +u`。

    比起「source 后回读 `env` 再构造字典」，直接在同一 shell 里跑命令更可靠
    —— ROS 的环境依赖 shell 函数（如 rospack 的 wrapper），
    光是复制环境变量并不能完整复现。
    """
    ws = ws or os.environ.get("ALLOCNET_WS", os.path.expanduser("~/allocnet_ws"))
    setup = os.path.join(ws, "devel", "setup.bash")
    prefix = "set +u; . /opt/ros/noetic/setup.bash"
    if os.path.exists(setup):
        prefix += "; . %s" % setup
    return _sh("%s; set -u; %s" % (prefix, cmd))


def _planner_dir(ws):
    """定位工作区里的 planner 包目录。"""
    return os.path.join(ws, "src", "AllocNet", "src", "planner")


def cmd_install(args):
    """把本模块的文件装入工作区的 planner 包。

    launch 用 pkg="planner" 引用脚本，所以这些文件必须就位于
    src/AllocNet/src/planner/ 下才能被 ROS 找到。本模块是它们的源。
    """
    ws = os.path.expanduser(args.ws)
    pkg = _planner_dir(ws)
    if not os.path.isdir(pkg):
        print("✘ 未找到 planner 包: %s" % pkg)
        print("  请确认工作区路径（--ws），且已克隆 AllocNet 到 src/AllocNet。")
        return 1

    plan = [
        ("teleop_keyboard.py", os.path.join(pkg, "scripts")),
        ("map_republisher.py", os.path.join(pkg, "scripts")),
        ("record_trajectory.py", os.path.join(pkg, "scripts")),
        ("capture_teleop_demo.py", os.path.join(pkg, "scripts")),
        ("make_assets.py", os.path.join(pkg, "scripts")),
        ("teleop_planning.launch", os.path.join(pkg, "launch")),
        ("teleop_planner.rviz", os.path.join(pkg, "config")),
        ("capture_view.rviz", os.path.join(pkg, "config")),
    ]

    print("=" * 62)
    print(" 安装到 %s" % pkg)
    print("=" * 62)
    for name, dstdir in plan:
        src = os.path.join(HERE, name)
        if not os.path.exists(src):
            print("  [跳过] %s（源文件不存在）" % name)
            continue
        os.makedirs(dstdir, exist_ok=True)
        dst = os.path.join(dstdir, name)
        shutil.copy2(src, dst)
        print("  [OK] %s" % os.path.relpath(dst, ws))

    # CMakeLists 需注册脚本，否则 rosrun / roslaunch 找不到
    cml = os.path.join(pkg, "CMakeLists.txt")
    if os.path.exists(cml):
        with open(cml, encoding="utf-8") as f:
            txt = f.read()
        missing = [s for s in ("teleop_keyboard.py", "map_republisher.py",
                               "record_trajectory.py") if s not in txt]
        if missing:
            print("\n  ⚠ CMakeLists.txt 未注册: %s" % ", ".join(missing))
            print("    请在 catkin_install_python(PROGRAMS ...) 中补上，否则")
            print("    rosrun/roslaunch 找不到这些脚本。")
        else:
            print("\n  [OK] CMakeLists.txt 已注册全部脚本")

    print("\n下一步：")
    print("  cd %s && catkin_make --pkg planner" % ws)
    print("  python3 main.py check")
    return 0


def cmd_check(args):
    """环境自检：把「跑不起来」的原因提前暴露，而不是等到 roslaunch 报错。"""
    print("=" * 62)
    print(" AllocNet 键盘遥操作模块 · 环境自检")
    print("=" * 62)
    ok = True

    # 1. ROS 是否就绪
    ros_ok = os.path.exists("/opt/ros/noetic/setup.bash")
    print("\n[1] ROS Noetic: %s" % ("已安装" if ros_ok else "未找到 /opt/ros/noetic"))
    if not ros_ok:
        ok = False

    # 2. 工作区
    ws = os.environ.get("ALLOCNET_WS", os.path.expanduser("~/allocnet_ws"))
    ws_ok = os.path.isdir(os.path.join(ws, "devel"))
    print("\n[2] AllocNet 工作区 %s: %s" % (ws, "已构建" if ws_ok else "未找到"))
    if not ws_ok:
        print("     提示：可用 ALLOCNET_WS 环境变量指定工作区路径。")
        print("     工作区需先按文档编译 AllocNet（含 planner / param_env 两个包）。")
        ok = False

    # 3. 上游包
    print("\n[3] 上游 ROS 包:")
    if ws_ok:
        probe = "; ".join(
            "echo \"%s=$(rospack find %s 2>/dev/null)\"" % (p, p)
            for p in REQUIRED_PKGS)
        rc, out = _ros_run(probe, ws)
        found = {}
        for line in out.splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                found[k.strip()] = v.strip()
        for pkg, desc in REQUIRED_PKGS.items():
            path = found.get(pkg, "")
            if path and os.path.isdir(path):
                print("     [OK] %-12s %s" % (pkg, path))
            else:
                print("     [缺] %-12s %s" % (pkg, desc))
                ok = False
    else:
        print("     （工作区未就绪，跳过）")

    # 4. 本模块脚本
    print("\n[4] 本模块脚本:")
    for s in OWN_SCRIPTS:
        p = os.path.join(HERE, s)
        print("     [%s] %s" % ("OK" if os.path.exists(p) else "缺", s))
        if not os.path.exists(p):
            ok = False

    # 5. launch 文件
    print("\n[5] launch 文件:")
    launch = os.path.join(HERE, "teleop_planning.launch")
    if os.path.exists(launch):
        with open(launch, encoding="utf-8") as f:
            head = f.read()
        # 反查：launch 里引用的脚本是否都在本目录
        for s in OWN_SCRIPTS:
            if s in head:
                print("     [OK] launch 引用了 %s" % s)
        print("     [OK] %s" % os.path.basename(launch))
    else:
        print("     [缺] teleop_planning.launch")
        ok = False

    # 6. 可选工具
    print("\n[6] 可选工具（仅采集素材时需要）:")
    for tool, why in [("rviz", "可视化"), ("import", "抓屏（ImageMagick）"),
                      ("Xvfb", "虚屏，无桌面环境时使用")]:
        p = shutil.which(tool)
        print("     [%s] %-8s %s" % ("OK" if p else "--", tool, why))

    print("\n" + "=" * 62)
    print(" 自检结果: %s" % ("通过，可以运行 `main.py run`" if ok else "存在问题，见上方 [缺] 项"))
    print("=" * 62)
    return 0 if ok else 1


def cmd_run(args):
    """启动完整仿真。"""
    if not os.path.exists("/opt/ros/noetic/setup.bash"):
        print("✘ 未找到 ROS Noetic。请先安装 ROS，参见 docs/ubuntu20.04.md")
        return 1

    ws = os.environ.get("ALLOCNET_WS", os.path.expanduser("~/allocnet_ws"))
    if not os.path.isdir(os.path.join(ws, "devel")):
        print("✘ 未找到工作区 %s" % ws)
        print("  请先按 docs/air/allocnet_teleop.md 的「环境搭建」编译 AllocNet，")
        print("  或用 ALLOCNET_WS 指定已有工作区。")
        return 1

    # launch 以 pkg="planner" 引用脚本，因此必须从 planner 包内运行。
    # 若用户还没 install，这里直接给出可执行的补救命令。
    rc, out = _ros_run("rospack find planner", ws)
    pkg_dir = out.strip().splitlines()[-1] if out.strip() else ""
    if not pkg_dir or not os.path.isdir(pkg_dir):
        print("✘ 未能在工作区中找到 planner 包。")
        print("  请先执行： python3 main.py install --ws %s" % ws)
        return 1
    launch = os.path.join(pkg_dir, "launch", "teleop_planning.launch")
    if not os.path.exists(launch):
        print("✘ planner 包里没有 teleop_planning.launch：")
        print("    %s" % launch)
        print("  请先执行： python3 main.py install --ws %s" % ws)
        return 1

    use_gui = "false" if args.no_gui else "true"
    rec = "false" if args.no_record else "true"
    print("启动: roslaunch planner teleop_planning.launch "
          "use_gui:=%s record:=%s" % (use_gui, rec))
    print("提示：键盘操作需在独立终端 rosrun planner teleop_keyboard.py；")
    print("      或向 /teleop/key 话题注入按键（见 `main.py help`）。")

    setup = os.path.join(ws, "devel", "setup.bash")
    cmd = ("set +u; . /opt/ros/noetic/setup.bash; . %s; set -u; "
           "roslaunch planner teleop_planning.launch use_gui:=%s record:=%s"
           % (setup, use_gui, rec))
    return subprocess.call(cmd, shell=True, executable="/bin/bash")


def cmd_capture(args):
    """采集演示素材（需要 Xvfb 虚屏，见文档说明）。"""
    script = os.path.join(HERE, "capture_teleop_demo.py")
    if not os.path.exists(script):
        print("✘ 未找到 capture_teleop_demo.py")
        return 1
    print("将调用 capture_teleop_demo.py 采集帧，再用 make_assets.py 合成。")
    print("前置：Xvfb 虚屏 + RViz（capture_view.rviz），详见文档 6.4 节。")
    return subprocess.call([sys.executable, script])


HELP = """
 AllocNet 键盘遥操作 · 常用命令速查
 ======================================================================
 环境
    source ~/allocnet_ws/devel/setup.bash

 一键启动
    python3 main.py run                 # 含 RViz
    python3 main.py run --no-gui        # 无头模式
    python3 main.py run --no-record     # 不记录轨迹

 键盘操作（需要 TTY，在独立终端运行）
    rosrun planner teleop_keyboard.py

 无 TTY 时用话题注入按键
    rostopic pub -1 /teleop/key std_msgs/String "data: 'w'"    # 前进
    rostopic pub -1 /teleop/key std_msgs/String "data: 'g'"    # 下发航点

 关键话题
    /move_base_simple/goal      下发航点（高度写在 orientation.z）
    /map/global_gridmap         latched 地图（规划器订阅此话题）
    /visualizer/trajectory      规划出的轨迹
    /teleop/cursor              游标，RViz 可见

 记录与绘图
    rosrun planner record_trajectory.py _output:=$HOME/run1.csv
    python3 src/planner/scripts/plot_trajectory.py --csv $HOME/run1.csv
 ======================================================================
 完整说明见 docs/air/allocnet_teleop.md
"""


def main():
    ap = argparse.ArgumentParser(
        description="AllocNet 键盘遥操作模块入口",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=HELP)
    sub = ap.add_subparsers(dest="cmd")

    sub.add_parser("check", help="环境自检")
    p_ins = sub.add_parser("install", help="把本模块装入工作区的 planner 包")
    p_ins.add_argument("--ws", default=os.path.expanduser("~/allocnet_ws"),
                       help="AllocNet 工作区路径（默认 ~/allocnet_ws）")
    p_run = sub.add_parser("run", help="启动完整仿真")
    p_run.add_argument("--no-gui", action="store_true", help="不启动 RViz")
    p_run.add_argument("--no-record", action="store_true", help="不记录轨迹")
    sub.add_parser("capture", help="采集演示素材")
    sub.add_parser("help", help="显示帮助")

    args = ap.parse_args()

    if args.cmd == "check":
        return cmd_check(args)
    if args.cmd == "install":
        return cmd_install(args)
    if args.cmd == "run":
        return cmd_run(args)
    if args.cmd == "capture":
        return cmd_capture(args)
    if args.cmd == "help" or args.cmd is None:
        print(HELP)
        return 0
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
