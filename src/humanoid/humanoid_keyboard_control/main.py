#!/usr/bin/env python3
"""
人形机器人 MuJoCo 仿真与键盘运动控制主入口
==========================================
课程要求：入口为 main.py，能够直接运行整个模块（无 ROS 依赖）。

支持模式：
  1. 交互模式（默认）—— 键盘实时控制：W/S 前进后退、A/D 转向、空格急停；
  2. 演示模式 --demo    —— 自动演示"前进→转向→前进→后退"并录制 GIF 动图与快照；
  3. 测试模式 --test    —— 运行自动化测试（站姿/驱动/转向/渲染/性能）；
  4. ROS2 模式 --ros2   —— 通过 ros2 launch 启动仿真 + 键盘遥控节点。

可视化：
  交互模式与演示模式默认打开 **MuJoCo 实时可视化窗口**（可直接看到人形机器人
  在场景中站立与移动）。窗口需要图形界面：本机桌面直接可用；SSH 远程需 `ssh -X`
  转发 X11；无图形环境请加 `--no-viewer`（仅命令行输出）。

示例：
  python3 main.py              # 交互控制（打开可视化窗口）
  python3 main.py --no-viewer  # 交互控制（无图形环境）
  python3 main.py --demo       # 自动演示并录制演示素材
  python3 main.py --test       # 自动化测试
  python3 main.py --ros2       # ROS2 launch（需已 source 环境并 colcon build）
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from humanoid_keyboard_control.sim_core import HumanoidSim, open_viewer

HELP_TEXT = """
============================================================
  人形机器人键盘运动控制（MuJoCo 物理仿真）
------------------------------------------------------------
  W / S     前进 / 后退     A / D   左转 / 右转
  空格       急停            ESC     退出
  松开按键   停止
============================================================
"""


# ----------------------------------------------------------------------
# 跨平台按键读取
# ----------------------------------------------------------------------
class KeyReader:
    """Linux 用 termios raw 模式；Windows 用 msvcrt 非阻塞读取。"""

    def __init__(self):
        self._platform = sys.platform
        self._fd = None
        self._old = None
        if self._platform.startswith("linux"):
            import termios
            import tty
            try:
                self._fd = os.open("/dev/tty", os.O_RDONLY)
            except OSError:
                self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setraw(self._fd)

    def get_key(self, timeout=0.03):
        if self._platform.startswith("linux"):
            import select
            import termios
            import tty
            r, _, _ = select.select([self._fd], [], [], timeout)
            if not r:
                return None
            ch = os.read(self._fd, 1)
            if not ch:
                return None
            try:
                return ch.decode("utf-8")
            except UnicodeDecodeError:
                return None
        elif self._platform.startswith("win"):
            import msvcrt
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch == '\x00' or ch == '\xe0':  # 功能键前缀
                    ch2 = msvcrt.getwch()
                    return {'H': 'up', 'P': 'down', 'K': 'left', 'M': 'right'}.get(ch2, ch2)
                return ch
            return None
        return None

    def close(self):
        if self._fd is not None and self._old is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            if self._fd != sys.stdin.fileno():
                os.close(self._fd)


# ----------------------------------------------------------------------
# 可视化窗口
# ----------------------------------------------------------------------
class _Pacer:
    """把仿真步进节流到实时，避免可视化窗口下仿真瞬间跑完。"""

    def __init__(self, dt):
        self.dt = dt
        self._next = time.perf_counter()

    def wait(self):
        self._next += self.dt
        delay = self._next - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        else:
            self._next = time.perf_counter()


# ----------------------------------------------------------------------
# 交互模式
# ----------------------------------------------------------------------
def run_interactive(demo=False, gif_path=None, snapshot_path=None, viewer=True):
    sim = HumanoidSim(realtime=False)
    reader = KeyReader()
    v = 0.0
    w = 0.0
    print(HELP_TEXT)
    print(f"[INFO] 仿真已启动 | 物理步长 {sim.dt*1000:.1f} ms | "
          f"可视化窗口 {'开' if viewer else '关'} | 按 ESC 退出\n")

    view = open_viewer(sim) if viewer else None
    pacer = _Pacer(sim.dt) if view is not None else None

    frames = []
    try:
        if demo:
            # 自动演示脚本：前进 → 转向 → 前进 → 转向回正 → 后退
            script = [("v", 0.3, 1.5), ("w", 0.5, 1.0), ("v", 0.3, 1.0),
                      ("w", -0.5, 1.0), ("v", -0.3, 1.0)]
            step_total = 0
            while step_total < sum(s[2] for s in script) / sim.dt:
                # 按脚本时间轴取指令
                t_sim = step_total * sim.dt
                cmd = (0.0, 0.0)
                acc = 0.0
                for mode, val, dur in script:
                    acc += dur
                    if t_sim < acc:
                        cmd = (val, 0.0) if mode == "v" else (0.0, val)
                        break
                v, w = cmd
                sim.step(v_lin=v, w_yaw=w)
                step_total += 1
                if step_total % 25 == 0:   # 50 Hz 采集帧（场景第三视角）
                    frames.append(sim.render(camera="overview"))
                od = sim.read_odom()
                if step_total % 250 == 0:
                    print(f"[演示] t={t_sim:4.1f}s pos=({od['x']:.2f}, {od['y']:.2f}) "
                          f"yaw={np.degrees(od['yaw']):5.1f}° z={od['z']:.2f}")
                if view is not None:
                    view.sync()
                    if not view.is_running():
                        print("\n[INFO] 可视化窗口已关闭，提前结束演示。")
                        break
                    pacer.wait()
            print("\n[演示] 动作序列完成 ✅")
        else:
            while True:
                key = reader.get_key()
                if key is None:
                    v, w = 0.0, 0.0
                elif key in ('w', 'W', 'up'):
                    v, w = sim.V_MAX * 0.8, 0.0
                elif key in ('s', 'S', 'down'):
                    v, w = -sim.V_MAX * 0.8, 0.0
                elif key in ('a', 'A', 'left'):
                    v, w = 0.0, sim.W_MAX * 0.6
                elif key in ('d', 'D', 'right'):
                    v, w = 0.0, -sim.W_MAX * 0.6
                elif key in (' ',):
                    v, w = 0.0, 0.0
                elif key == '\x1b' or key == 'q':
                    break
                sim.step(v_lin=v, w_yaw=w)
                od = sim.read_odom()
                # 周期状态输出（1 Hz）
                if abs(sim.data.time - round(sim.data.time)) < sim.dt:
                    print(f"\r[{sim.data.time:5.1f}s] pos=({od['x']:.2f}, {od['y']:.2f}) "
                          f"yaw={np.degrees(od['yaw']):5.1f}° z={od['z']:.2f} v=({v:+.2f}, {w:+.2f}) "
                          f"{'✅站立' if sim.standing() else '⚠️跌倒'}", end='', flush=True)
                if view is not None:
                    view.sync()
                    if not view.is_running():
                        break
                    pacer.wait()
                else:
                    time.sleep(sim.dt)
    finally:
        if view is not None:
            try:
                view.close()
            except Exception:                                     # noqa: BLE001
                pass
        reader.close()

    if gif_path:
        save_gif(frames, gif_path)
    if snapshot_path and frames:
        save_png(frames[-1], snapshot_path)
    print("\n[INFO] 仿真结束。")

    if not sim.standing():
        print("[WARN] 机器人未能保持站立（可尝试降低转向速度参数 w_max）。")


# ----------------------------------------------------------------------
# GIF / 快照输出
# ----------------------------------------------------------------------
def save_gif(frames, path, fps=25):
    try:
        import imageio
    except ImportError:
        print(f"[WARN] 未安装 imageio，跳过 GIF 保存: {path}")
        return
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    imageio.mimsave(path, frames, fps=fps)
    print(f"[INFO] 演示动图已保存: {path} ({len(frames)} 帧, "
          f"{os.path.getsize(path)/1024:.0f} KB)")


def save_png(img, path):
    try:
        import imageio.v2 as iio
    except ImportError:
        try:
            from PIL import Image
            Image.fromarray(img).save(path)
            print(f"[INFO] 快照已保存: {path}")
            return
        except ImportError:
            print(f"[WARN] 未安装 imageio/PIL，跳过快照: {path}")
            return
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    iio.imwrite(path, img)
    print(f"[INFO] 快照已保存: {path}")


# ----------------------------------------------------------------------
# 主入口
# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="人形机器人 MuJoCo 仿真与键盘运动控制")
    parser.add_argument("--demo", action="store_true", help="自动演示并录制演示素材")
    parser.add_argument("--gif", type=str, default=None, help="演示动图输出路径 (.gif)")
    parser.add_argument("--snapshot", type=str, default=None, help="快照输出路径 (.png)")
    parser.add_argument("--test", action="store_true", help="运行自动化测试")
    parser.add_argument("--ros2", action="store_true", help="通过 ros2 launch 启动 ROS2 节点")
    parser.add_argument("--no-viewer", dest="viewer", action="store_false",
                        help="不打开可视化窗口（无图形环境时使用，仅命令行输出）")
    parser.set_defaults(viewer=True)
    args = parser.parse_args()

    if args.test:
        import subprocess
        test_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "test_standalone.py")
        sys.exit(subprocess.run([sys.executable, test_path]).returncode)
    if args.ros2:
        import subprocess
        print("[INFO] 正在通过 ros2 launch 启动人形机器人仿真与键盘遥控...")
        sys.exit(subprocess.run(["ros2", "launch", "humanoid_keyboard_control", "main.launch.py"]).returncode)

    # 默认演示素材输出路径：当前工作目录 demo_assets/（演示素材统一归档至 docs/img）
    out_dir = os.path.join(os.getcwd(), "demo_assets")
    gif_path = args.gif or (os.path.join(out_dir, "humanoid_demo.gif") if args.demo else None)
    snap_path = args.snapshot or (os.path.join(out_dir, "humanoid_demo.png") if args.demo else None)
    run_interactive(demo=args.demo, gif_path=gif_path, snapshot_path=snap_path,
                    viewer=args.viewer)


if __name__ == "__main__":
    main()
