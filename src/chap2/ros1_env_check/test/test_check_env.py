# -*- coding: utf-8 -*-
"""ros1_env_check 的回归测试：覆盖评审 #628 指出的三个问题。

为什么用 subprocess 而不是直接 import：
    `check_env.py` 是个命令行脚本（靠 print 输出结论、靠退出码表达成败），
    这里要断言的正是"命令行行为"——退出码与输出文字。
    直接调用函数反而测不到"退出码是否为 1"这类最容易回归的部分。

运行方式（两种都支持）：
    python3 -m pytest test/ -q
    python3 test/test_check_env.py        # 不装 pytest 也能跑
"""

import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(PKG_ROOT)))
SCRIPT = os.path.join(PKG_ROOT, "check_env.py")
TMP = os.path.join(HERE, "_tmp_scan")

PASS = 0
FAIL = 0


def check(cond, msg):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  [PASS] %s" % msg)
    else:
        FAIL += 1
        print("  [FAIL] %s" % msg)


def run(*args, **kw):
    """执行 check_env.py，返回 (退出码, 输出文本)。

    可选 cwd：脚本用 os.getcwd() 判断"当前处于哪个工作空间"，
    所以测试必须显式指定工作目录，否则结果取决于测试是从哪里启动的。
    """
    proc = subprocess.run([sys.executable, SCRIPT] + list(args),
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          timeout=120, cwd=kw.get("cwd"))
    return proc.returncode, proc.stdout.decode("utf-8", "replace")


# ---------------------------------------------------- 评审问题 ①：BOM 必须能检出
def test_bom_detected():
    print("\n== ① BOM 检测 ==")
    if os.path.isdir(TMP):
        shutil.rmtree(TMP)
    os.makedirs(TMP)
    try:
        # 造两个文件：一个带 BOM，一个不带
        with open(os.path.join(TMP, "with_bom.md"), "wb") as fh:
            fh.write(b"\xef\xbb\xbf" + "# BOM 文件\n正文\n".encode("utf-8"))
        with open(os.path.join(TMP, "clean.md"), "wb") as fh:
            fh.write("# 干净文件\n正文\n".encode("utf-8"))

        code, out = run("--scan", TMP)
        check("with_bom.md" in out, "带 BOM 的文件被报出来")
        check("UTF-8 BOM" in out, "输出里明确写了 UTF-8 BOM")
        check(code == 1, "检出 BOM 时退出码为 1（实际 %d）" % code)

        # 干净目录不应报 BOM
        clean_dir = os.path.join(TMP, "clean_only")
        os.makedirs(clean_dir)
        with open(os.path.join(clean_dir, "ok.md"), "wb") as fh:
            fh.write("# 干净\n".encode("utf-8"))
        code2, out2 = run("--scan", clean_dir)
        bom_lines = [ln for ln in out2.splitlines()
                     if "UTF-8 BOM" in ln and "[ OK ]" not in ln and "无 " not in ln]
        check(not bom_lines, "干净目录不报 BOM")
    finally:
        shutil.rmtree(TMP, ignore_errors=True)


# ------------------------------ 评审问题 ②：扫描路径不存在必须失败，不能静默通过
def test_missing_scan_dir_fails():
    print("\n== ② --scan 路径不存在 ==")
    missing = os.path.join(HERE, "definitely_not_here_%d" % os.getpid())
    code, out = run("--scan", missing, "--quiet")
    check("扫描路径不存在" in out, "输出里明确指出路径不存在")
    check(code == 1, "退出码为 1，而不是静默全绿（实际 %d）" % code)

    # 文件（而非目录）也要拒绝
    a_file = os.path.join(HERE, "not_a_dir.txt")
    with open(a_file, "w", encoding="utf-8") as fh:
        fh.write("x\n")
    try:
        code2, out2 = run("--scan", a_file, "--quiet")
        check("不是目录" in out2, "传入文件时提示「不是目录」")
        check(code2 == 1, "传入文件时退出码为 1（实际 %d）" % code2)
    finally:
        os.remove(a_file)


# --------------------------- 评审问题 ③：仓库根目录不能被当成 catkin 工作空间
def test_repo_root_not_workspace():
    print("\n== ③ 仓库根目录（有 src/ 但无 catkin 标记） ==")
    if not os.path.isdir(os.path.join(REPO_ROOT, "src")):
        print("  [SKIP] 当前不在仓库内，跳过")
        return
    code, out = run("--quiet", cwd=REPO_ROOT)
    check("源码树" in out, "识别为源码树而不是工作空间")
    check("不要在这里直接 catkin_make" in out, "明确提示不要在此处 catkin_make")
    check("build/" in out and "devel/" in out, "说明了会生成 build/、devel/")

    # 反过来：真的 catkin 工作空间应被识别出来（用临时目录伪造标记）
    fake_ws = os.path.join(TMP, "fake_ws")
    os.makedirs(os.path.join(fake_ws, "src"))
    try:
        with open(os.path.join(fake_ws, ".catkin_workspace"), "w", encoding="utf-8") as fh:
            fh.write("#!/usr/bin/env bash\n")
        proc = subprocess.run([sys.executable, SCRIPT],
                              cwd=fake_ws, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, timeout=120)
        out2 = proc.stdout.decode("utf-8", "replace")
        check("工作空间根目录" in out2, "带 .catkin_workspace 标记时识别为工作空间")
    finally:
        shutil.rmtree(TMP, ignore_errors=True)


def main():
    print("=" * 72)
    print("ros1_env_check 回归测试（对应评审 #628 的三条意见）")
    print("=" * 72)
    test_bom_detected()
    test_missing_scan_dir_fails()
    test_repo_root_not_workspace()
    print("\n" + "=" * 72)
    print("通过 %d 项，失败 %d 项" % (PASS, FAIL))
    print("=" * 72)
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
