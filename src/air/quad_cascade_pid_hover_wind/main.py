#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键实验：构建 -> 启动仿真 -> 等待完成 -> 生成分析图表。"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time


def find_repo_root(start: str) -> str:
    cur = os.path.abspath(start)
    while True:
        if os.path.isdir(os.path.join(cur, ".git")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            raise RuntimeError("未找到仓库根目录")
        cur = parent


def run(cmd, env=None, cwd=None, timeout=None):
    print(">>", " ".join(cmd)[:300], flush=True)
    return subprocess.run(cmd, env=env, cwd=cwd, timeout=timeout)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=None, help="结果输出目录")
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--build", action="store_true", help="先 colcon build")
    parser.add_argument("--keep", action="store_true", help="保留旧结果")
    args = parser.parse_args()

    repo = find_repo_root(os.path.dirname(__file__))
    pkg_src = os.path.join(repo, "src", "air", "quad_cascade_pid_hover_wind")
    run_id = time.strftime("run_%m%d_%H%M%S")
    out_dir = args.output or os.path.join(pkg_src, "results", run_id)
    if not args.keep and os.path.isdir(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    # 干净环境：去掉 ROS1 noetic 路径；LD_LIBRARY_PATH 采用前置追加，
    # 保留用户原有的库路径，避免覆盖系统其他依赖
    base_env = os.environ.copy()
    extra_ld = ["/opt/ros/humble/lib", "/usr/lib/x86_64-linux-gnu"]
    existing_ld = [p for p in base_env.get("LD_LIBRARY_PATH", "").split(os.pathsep)
                   if p and p not in extra_ld]
    base_env["LD_LIBRARY_PATH"] = os.pathsep.join(extra_ld + existing_ld)
    ros1_paths = [p for p in base_env.get("PYTHONPATH", "").split(":") if "noetic" not in p]
    base_env["PYTHONPATH"] = ":".join(ros1_paths)

    # 构建工作区
    ws = os.path.join(repo, "build_quad_ws")
    src_link = os.path.join(ws, "src")
    os.makedirs(ws, exist_ok=True)
    if os.path.islink(src_link) or os.path.isdir(src_link):
        if os.path.islink(src_link):
            os.unlink(src_link)
        else:
            shutil.rmtree(src_link)
    os.symlink(os.path.join(repo, "src", "air"), src_link)

    setup_bash = os.path.join(ws, "install", "setup.bash")
    if args.build or not os.path.exists(setup_bash):
        print("=== colcon build ===")
        r = run(["colcon", "build", "--packages-select", "quad_cascade_pid_hover_wind",
                 "--symlink-install", "--cmake-args", "-DCMAKE_BUILD_TYPE=Release"],
                cwd=ws, env=base_env)
        if r.returncode != 0:
            print("构建失败", file=sys.stderr)
            sys.exit(1)

    env = base_env.copy()
    # 加载工作区
    cmd = (f'source /opt/ros/humble/setup.bash && source {setup_bash} && '
           f'ros2 launch quad_cascade_pid_hover_wind main.launch.py '
           f'output_dir:={out_dir} duration:={args.duration}')
    print("=== 启动仿真 ===")
    proc = subprocess.Popen(["bash", "-c", cmd], env=env, cwd=ws)

    # 等待完成标志
    status_path = os.path.join(out_dir, "status.json")
    start = time.time()
    while proc.poll() is None:
        if os.path.exists(status_path):
            try:
                with open(status_path) as f:
                    s = json.load(f)
                if s.get("status") == "finished":
                    break
            except Exception:
                pass
        time.sleep(0.5)
        if time.time() - start > args.duration + 120:
            print("超时，强制结束仿真", file=sys.stderr)
            break

    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()

    # 分析
    print("=== 生成分析图表 ===")
    analyze = os.path.join(pkg_src, "tools", "analyze.py")
    r = run(["python3", analyze, "--data", out_dir], env=env)
    if r.returncode != 0:
        print("分析脚本退出码 %d" % r.returncode, file=sys.stderr)

    print("实验完成，结果目录：", out_dir)


if __name__ == "__main__":
    main()
