#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 1 环境自检与文件卫生检查。

为什么需要它
------------
在课程仓库里跑别人的模块时，报错往往不是代码错，而是**环境没配好**：

* 终端里混了 ROS 2（``ROS_DISTRO=foxy``）却去 ``catkin_make``；
* 忘了 ``source /opt/ros/noetic/setup.bash``，于是 ``import rospy`` 失败；
* 缺 ``python3-numpy`` / ``python3-matplotlib``，程序跑到一半才崩；
* 从 Windows 拷过来的文件带 **BOM** 或 **CRLF**，脚本报 ``bad interpreter``；
* 脚本没有可执行位，``roslaunch`` 报找不到可执行文件。

这些现象各自都有明确的修法，但错误信息往往指向别处，容易误判成"代码有问题"。
本脚本把上述检查集中到一处，一次性打印「什么没问题、什么有问题、怎么修」。

用法
----
::

    python3 check_env.py                    # 检查 ROS 1 运行环境
    python3 check_env.py --scan .           # 额外扫描文件卫生（BOM / CRLF / 可执行位）
    python3 check_env.py --scan . --strict  # 把警告也算失败（适合放进 CI）
    python3 check_env.py --quiet            # 只打印问题和结论

退出码：0 = 全部通过；1 = 存在 FAIL（或 --strict 下有 WARN）。

只依赖 Python 标准库，不需要 ROS 也能运行——因此在没装 ROS 的机器上
它同样能给出「缺什么」的结论，而不是自己先崩掉。
"""

import argparse
import os
import subprocess
import sys

# ---------------------------------------------------------------- 输出与计数
OK, WARN, FAIL = "OK", "WARN", "FAIL"
_ICON = {OK: "[ OK ]", WARN: "[WARN]", FAIL: "[FAIL]"}

counts = {OK: 0, WARN: 0, FAIL: 0}
_findings = []          # (level, 标题, 修复建议)
QUIET = False           # --quiet：只打印非 OK 项


def _supports_color() -> bool:
    if os.environ.get("NO_COLOR"):
        return False
    return hasattr(sys.stdout, "isatty") and sys.stdout.isatty()


_COLOR = {"OK": "\033[32m", "WARN": "\033[33m", "FAIL": "\033[31m", "0": "\033[0m"}


def report(level: str, message: str, fix: str = "") -> None:
    counts[level] += 1
    if level == OK and QUIET:
        return
    icon = _ICON[level]
    if _supports_color():
        icon = _COLOR[level] + icon + _COLOR["0"]
    print(f"{icon} {message}")
    if fix:
        print(f"       修复：{fix}")
    if level != OK:
        _findings.append((level, message, fix))


def section(title: str) -> None:
    if QUIET:
        return
    print(f"\n== {title} ==")


def _run(cmd):
    """执行外部命令，返回 (成功?, 标准输出)。失败不抛异常。"""
    try:
        out = subprocess.run(cmd, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, timeout=10)
        return out.returncode == 0, out.stdout.decode("utf-8", "replace").strip()
    except (OSError, subprocess.SubprocessError):
        return False, ""


# ---------------------------------------------------------------- 1. ROS 环境
ROS1_DISTROS = {"kinetic", "melodic", "noetic"}
ROS2_DISTROS = {"foxy", "galactic", "humble", "iron", "jazzy", "rolling"}


def _installed_distros():
    """列出 /opt/ros 下已安装的发行版（Linux）。

    课程环境常见「ROS 1 与 ROS 2 同时安装」——Ubuntu 20.04 上通常是
    noetic + foxy。这种情况下真正的问题不是"没装"，而是**当前终端 source 了哪一个**，
    因此要先把已安装的版本列出来，再判断当前终端的状态。
    """
    base = "/opt/ros"
    if not os.path.isdir(base):
        return []
    try:
        return sorted(d for d in os.listdir(base)
                      if os.path.isdir(os.path.join(base, d)))
    except OSError:
        return []


def check_ros() -> None:
    section("ROS 环境")

    distro = os.environ.get("ROS_DISTRO", "").strip()
    ros_version = os.environ.get("ROS_VERSION", "").strip()
    ament = os.environ.get("AMENT_PREFIX_PATH", "").strip()
    colcon = os.environ.get("COLCON_PREFIX_PATH", "").strip()

    installed = _installed_distros()
    ros1_installed = [d for d in installed if d in ROS1_DISTROS]
    ros2_installed = [d for d in installed if d in ROS2_DISTROS]

    if os.name == "nt":
        report(OK, "跳过 /opt/ros 安装检查（Windows 没有该目录，属正常）")
    elif installed:
        report(OK, "/opt/ros 下已安装：" + "、".join(installed))
    else:
        report(WARN, "未在 /opt/ros 下找到任何发行版——ROS 似乎没有安装",
               "安装 ROS Noetic，或确认安装在非标准路径并 source 对应 setup.bash")

    if ros1_installed and ros2_installed:
        report(WARN,
               "本机同时安装了 ROS 1（{}）与 ROS 2（{}）".format(
                   "、".join(ros1_installed), "、".join(ros2_installed)),
               "本书实验基于 ROS 1。请**新开一个终端**，只 source "
               "/opt/ros/{}/setup.bash；不要先 source ROS 2 再 catkin_make，"
               "两者混用会导致 ROS_DISTRO 冲突、catkin_make 行为异常".format(
                   ros1_installed[-1]))
    elif ros1_installed:
        report(OK, "已安装 ROS 1：" + "、".join(ros1_installed))

    # ---- 当前终端到底处于哪一版 ----
    if not distro:
        if ros1_installed:
            report(FAIL, "ROS_DISTRO 未设置——ROS 已安装，但当前终端没有 source",
                   "source /opt/ros/{}/setup.bash".format(ros1_installed[-1]))
        else:
            report(FAIL, "ROS_DISTRO 未设置——当前终端没有 source 任何 ROS",
                   "source /opt/ros/noetic/setup.bash")
    elif distro in ROS2_DISTROS or ros_version == "2":
        report(FAIL,
               "当前终端是 ROS 2 环境（ROS_DISTRO={}，ROS_VERSION={}）".format(
                   distro, ros_version or "2"),
               "本书实验基于 ROS 1 Noetic。**新开终端**只执行 "
               "source /opt/ros/noetic/setup.bash（不要再 source 那份 {}/setup.bash）；"
               "已经在同一个终端里混用过的，直接关掉终端重开最省事".format(distro))
    elif distro in ROS1_DISTROS:
        if ros_version == "2":
            report(FAIL, "ROS_DISTRO={} 但 ROS_VERSION=2，环境已被混用".format(distro),
                   "关掉当前终端重开，只 source /opt/ros/{}/setup.bash".format(distro))
        else:
            report(OK, "当前终端为 ROS 1 环境：ROS_DISTRO={}".format(distro))
    else:
        report(WARN, "ROS_DISTRO = {}（不是已知的 ROS 1 发行版）".format(distro),
               "确认是否装的是 ROS 1 Noetic")

    if ament:
        report(WARN, "AMENT_PREFIX_PATH 非空，说明环境里还残留 ROS 2 的路径",
               "关掉终端重开，只 source ROS 1；临时处理可 unset AMENT_PREFIX_PATH")
    if colcon:
        report(WARN, "COLCON_PREFIX_PATH 非空，当前终端混有 ROS 2 的工作空间",
               "ROS 1 用 catkin_make，ROS 2 用 colcon build，二者不要在同一终端操作")

    # rosversion 是判断"装没装、装的是哪版"最直接的依据
    ok, out = _run(["rosversion", "-d"])
    if ok and out:
        report(OK, f"rosversion -d = {out}")
    else:
        ok_ver, ver = _run(["rosversion", "--version"])
        if ok_ver and ver:
            report(OK, f"rosversion = {ver.splitlines()[0]}")
        else:
            report(FAIL, "命令 rosversion 不可用——ROS 1 没装好或没 source",
                   "确认已安装 ROS Noetic，并 source /opt/ros/noetic/setup.bash")

    master = os.environ.get("ROS_MASTER_URI", "").strip()
    if master:
        report(OK, f"ROS_MASTER_URI = {master}")
    else:
        report(WARN, "ROS_MASTER_URI 未设置",
               "roslaunch 会自动启动 master，一般可忽略；单独跑 rosrun 前需先 roscore")


# ---------------------------------------------------------- 2. Python 与依赖
REQUIRED_MODULES = [
    ("rospy", True, "source /opt/ros/noetic/setup.bash"),
    ("numpy", True, "sudo apt-get install -y python3-numpy"),
]
OPTIONAL_MODULES = [
    ("yaml", "sudo apt-get install -y python3-yaml"),
    ("matplotlib", "sudo apt-get install -y python3-matplotlib"),
    ("catkin_pkg", "sudo apt-get install -y python3-catkin-pkg"),
    ("tf", "sudo apt-get install -y python3-tf"),
]


def check_python() -> None:
    section("Python 与依赖")

    major, minor = sys.version_info[:2]
    text = f"Python {major}.{minor}.{sys.version_info[2]}"
    if (major, minor) >= (3, 6):
        report(OK, text + "（满足 >= 3.6）")
    else:
        report(FAIL, text + " 版本过低", "ROS Noetic 需要 Python 3.6 以上，通常是 3.8")

    for name, required, fix in REQUIRED_MODULES:
        try:
            __import__(name)
            report(OK, f"import {name} 正常")
        except Exception as exc:                     # noqa: BLE001 - 需捕获所有导入错误
            level = FAIL if required else WARN
            report(level, f"import {name} 失败：{type(exc).__name__}", fix)

    for name, fix in OPTIONAL_MODULES:
        try:
            __import__(name)
            report(OK, f"import {name} 正常（可选）")
        except Exception:                            # noqa: BLE001
            report(WARN, f"import {name} 失败（可选依赖，影响对应功能）", fix)


# ------------------------------------------------------------ 3. 工作空间检查
def _find_ws_root(start: str):
    """从 start 向上找含 src/ 的 catkin 工作空间根目录。"""
    cur = os.path.abspath(start)
    while True:
        if os.path.isdir(os.path.join(cur, "src")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            return None
        cur = parent


def check_workspace(path: str) -> None:
    section("工作空间")

    ws = _find_ws_root(path)
    if ws is None:
        report(WARN, f"未在 {os.path.abspath(path)} 及其上级找到 catkin 工作空间（没有 src/）",
               "单独建一个：mkdir -p ~/catkin_ws/src && cd ~/catkin_ws && catkin_make")
        return

    report(OK, f"工作空间根目录：{ws}")
    for sub in ("src", "build", "devel"):
        if os.path.isdir(os.path.join(ws, sub)):
            report(OK, f"存在 {sub}/")
        elif sub == "src":
            report(FAIL, "缺少 src/ 目录", "catkin 工作空间必须有 src/")
        else:
            report(WARN, f"缺少 {sub}/（尚未编译过）", f"cd {ws} && catkin_make")

    # devel 的 Python 路径是否已进入 sys.path，等价于"有没有 source devel/setup.bash"
    devel_py = os.path.join(ws, "devel", "lib", "python3", "dist-packages")
    if os.path.isdir(devel_py):
        if any(os.path.abspath(p) == os.path.abspath(devel_py) for p in sys.path):
            report(OK, "devel 的 Python 包路径已生效（source devel/setup.bash 已执行）")
        else:
            report(WARN, "devel 存在但未生效——当前终端没 source devel/setup.bash",
                   f"source {devel_py.split('/devel/')[0]}/devel/setup.bash")

    # 包数量（只数含 package.xml 的目录）
    n_pkg = 0
    src_dir = os.path.join(ws, "src")
    for _root, _dirs, files in os.walk(src_dir):
        if "package.xml" in files:
            n_pkg += 1
    report(OK if n_pkg else WARN, f"src/ 下共有 {n_pkg} 个 catkin 包",
           "" if n_pkg else "把模块目录拷进 src/ 后再 catkin_make")


# --------------------------------------------------------- 4. 文件卫生（扫描）
SCAN_EXTS = (".py", ".sh", ".md", ".launch", ".xml", ".yaml", ".yml", ".cpp", ".h")
SKIP_DIRS = {".git", "__pycache__", "build", "devel", "install", "node_modules",
             "site", ".pytest_cache"}


def check_files(root: str, limit: int = 8) -> None:
    """检查 BOM、CRLF 与 shell 脚本的可执行位。

    这三类问题在 Windows 与 Linux 之间来回拷文件时非常常见，
    报错信息却通常指向别处（例如 ``bad interpreter`` 或 ``找不到可执行文件``）。
    """
    section(f"文件卫生（扫描 {os.path.abspath(root)}）")

    bom, crlf, noexec = [], [], []
    scanned = 0
    for cur, dirs, files in os.walk(root):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for name in files:
            if not name.endswith(SCAN_EXTS):
                continue
            full = os.path.join(cur, name)
            scanned += 1
            try:
                with open(full, "rb") as fh:
                    head = fh.read(3)
                    is_bom = head == b"\xef\xbb\xbf"
                    fh.seek(0)
                    blob = fh.read()
            except OSError:
                continue
            if is_bom:
                bom.append(os.path.relpath(full, root))
            if b"\r\n" in blob:
                crlf.append(os.path.relpath(full, root))
            if name.endswith(".sh") and not os.access(full, os.X_OK):
                noexec.append(os.path.relpath(full, root))
    report(OK, f"共扫描 {scanned} 个文本文件")

    def _list(title, items, level, fix, why):
        if not items:
            report(OK, f"无 {title}")
            return
        shown = "、".join(items[:limit]) + ("…" if len(items) > limit else "")
        report(level, f"{len(items)} 个文件含 {title}：{shown}", fix)
        if not QUIET:
            print(f"       说明：{why}")

    _list("UTF-8 BOM", bom, FAIL,
          "逐个转成「UTF-8 无 BOM」；Python 可容忍，但 BOM 出现在 mkdocs.yml、"
          "launch、shell 脚本里会引发难以定位的异常",
          "BOM 是文件最前面的 EF BB BF 三个字节，多数编辑器会把它显示成不可见字符")
    _list("CRLF 行尾", crlf, WARN,
          "在仓库里统一为 LF：git config core.autocrlf false 后重新检出，"
          "或用编辑器把换行符改成 LF",
          "shell 脚本带 CRLF 会报 bad interpreter: /bin/bash^M，"
          ".py 带 CRLF 在部分环境会报 SyntaxError")
    if os.name == "nt":
        # Windows 没有 POSIX 可执行位，os.access(X_OK) 只检查文件是否存在，
        # 在这里判断会产生假阴性，因此显式跳过，避免给出错误结论。
        report(OK, "跳过「可执行位」检查（Windows 文件系统没有 POSIX 权限位）")
    else:
        _list("缺少可执行位（.sh）", noexec, WARN,
              "chmod +x 对应脚本；注意 git 里要用 git update-index --chmod=+x 才会被记录",
              "roslaunch 与直接执行都要求脚本可执行")


# ---------------------------------------------------------------------- 结论
def summary(strict: bool) -> int:
    print("\n" + "=" * 68)
    print(f"结果：{counts[OK]} 项通过，{counts[WARN]} 项警告，{counts[FAIL]} 项失败")
    if _findings:
        print("\n需要处理：")
        for level, msg, fix in _findings:
            print(f"  [{level}] {msg}")
            if fix:
                print(f"         → {fix}")
    else:
        print("环境自检全部通过，可以开始跑实验了。")
    print("=" * 68)

    if counts[FAIL] or (strict and counts[WARN]):
        return 1
    return 0


def main(argv=None) -> int:
    global QUIET

    parser = argparse.ArgumentParser(
        description="ROS 1 环境自检与文件卫生检查（不依赖 ROS，可在任意机器运行）")
    parser.add_argument("--scan", metavar="DIR", default=None,
                        help="额外扫描该目录下的 BOM / CRLF / 可执行位问题")
    parser.add_argument("--strict", action="store_true",
                        help="把警告也视为失败，适合放进 CI")
    parser.add_argument("--quiet", action="store_true",
                        help="只打印警告、失败与最终结论")
    args = parser.parse_args(argv)

    QUIET = args.quiet

    if not QUIET:
        print("=" * 68)
        print("ROS 1 环境自检")
        print("=" * 68)

    check_ros()
    check_python()
    check_workspace(os.getcwd())
    if args.scan:
        check_files(args.scan)

    return summary(args.strict)


if __name__ == "__main__":
    sys.exit(main())
