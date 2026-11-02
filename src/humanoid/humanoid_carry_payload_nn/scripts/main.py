#!/usr/bin/env python3
"""main.py —— humanoid_carry_payload_nn 的统一入口（可在 ROS 环境外独立运行）。

模式
----
demo     : 按预设指令曲线演示搬运（可选 MuJoCo 可视化窗口）
keyboard : 键盘遥控（W/S 前后、A/D 转向、空格急停、Q 退出）
eval     : 0/2/4 kg 负载场景下 open / pd / nn 三种控制器对照评测
collect  : 采集数据并训练平衡神经网络（等价于 train_mlp.py）

控制器
------
open : 纯开环步态
pd   : PD 姿态反馈专家（带负载质量前馈）
nn   : 神经网络残差控制器（默认读取 models/balance_mlp.npz）

示例
----
    python3 scripts/main.py --mode demo --controller nn
    python3 scripts/main.py --mode keyboard --controller nn
    python3 scripts/main.py --mode eval
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import BalanceMLP, PDExpert                                   # noqa: E402
from sim_core import CarryPayloadSim, open_viewer, package_path         # noqa: E402

MAX_SPEED = 0.22
MAX_YAW_RATE = 0.45
WEIGHTS = package_path("models/balance_mlp.npz")


def make_controller(name):
    """按名字构造控制器；nn 权重缺失时自动退回 PD 专家并给出提示。"""
    if name == "pd":
        return PDExpert()
    if name == "nn":
        if os.path.isfile(WEIGHTS):
            return BalanceMLP.load(WEIGHTS)
        print("[WARN] 未找到 %s，改用 PD 专家（先运行 train_mlp.py 生成权重）" % WEIGHTS)
        return PDExpert()
    return None


def log(logger, msg):
    (logger.info if logger is not None else print)(msg)


def demo_command(t, speed):
    """演示指令：0.5 s 内起步，4~7 s 轻微转向，随后匀速搬运。"""
    v = speed * min(1.0, t / 0.5)
    w = 0.12 * math.sin(0.6 * t) if 4.0 < t < 7.0 else 0.0
    return v, w


def run_demo(args, log_obj=None):
    controller = make_controller(args.controller)
    sim = CarryPayloadSim(payload_mass=args.mass, gait=None)
    sim.reset(args.mass)
    view = None if args.no_viewer else open_viewer(sim, log_obj)
    log(log_obj, "[demo] 负载 %.1f kg，控制器 %s，时长 %.0f s"
        % (args.mass, args.controller, args.seconds))

    n = int(args.seconds / sim.dt)
    next_print = 0.0
    try:
        for i in range(n):
            t = i * sim.dt
            v_cmd, w_cmd = demo_command(t, args.speed)
            sim.step(v_cmd=v_cmd, w_cmd=w_cmd, controller=controller)
            if view is not None:
                if not view.is_running():
                    log(log_obj, "[demo] 可视化窗口已关闭，继续无界面运行")
                    view = None
                else:
                    view.sync()
            if t >= next_print:
                next_print += 2.0
                _, pitch, _, _ = sim.posture()
                log(log_obj, "[demo] t=%5.2f s  v=%.2f m/s  俯仰=%+.1f°  高度=%.2f m  "
                    "等效里程=%.1f m  站立=%s"
                    % (t, v_cmd, math.degrees(pitch), sim.odom()["z"], sim.distance,
                       "是" if sim.standing() else "否"))
    except KeyboardInterrupt:
        log(log_obj, "[demo] 收到中断，结束演示")
    finally:
        sim.close()


def run_keyboard(args, log_obj=None):
    """键盘遥控：POSIX 用 /dev/tty，Windows 用 msvcrt。"""
    controller = make_controller(args.controller)
    sim = CarryPayloadSim(payload_mass=args.mass)
    sim.reset(args.mass)
    view = open_viewer(sim, log_obj)
    log(log_obj, "[keyboard] W/S 前后，A/D 转向，空格急停，Q 退出（目标速度 %.2f m/s）"
        % MAX_SPEED)

    keys = _KeyboardReader(log_obj)
    v_cmd, w_cmd = 0.0, 0.0
    t0 = time.time()
    try:
        while not keys.quit:
            key = keys.pop()
            if key == "w":
                v_cmd = min(MAX_SPEED, v_cmd + 0.04)
            elif key == "s":
                v_cmd = max(-MAX_SPEED, v_cmd - 0.04)
            elif key == "a":
                w_cmd = min(MAX_YAW_RATE, w_cmd + 0.06)
            elif key == "d":
                w_cmd = max(-MAX_YAW_RATE, w_cmd - 0.06)
            elif key == " ":
                v_cmd, w_cmd = 0.0, 0.0
            sim.step(v_cmd=v_cmd, w_cmd=w_cmd, controller=controller)
            if view is not None and view.is_running():
                view.sync()
            if int(time.time() - t0) % 2 == 0 and keys.tick:
                _, pitch, _, _ = sim.posture()
                log(log_obj, "[keyboard] v=%+.2f w=%+.2f 俯仰=%+.1f° 里程=%.1f m"
                    % (v_cmd, w_cmd, math.degrees(pitch), sim.distance))
    except KeyboardInterrupt:
        pass
    finally:
        keys.stop()
        sim.close()


class _KeyboardReader:
    """非阻塞逐键读取，跨平台。"""

    def __init__(self, log_obj=None):
        self.quit = False
        self.tick = False
        self._buf = ""
        self._fd = None
        self._msvcrt = None
        if os.name == "nt":
            import msvcrt
            self._msvcrt = msvcrt
        else:
            import select
            import termios
            import tty
            self._select, self._termios, self._tty = select, termios, tty
            try:
                self._fd = os.open("/dev/tty", os.O_RDONLY | os.O_NONBLOCK)
                self._old = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)
            except OSError as exc:
                log(log_obj, "[keyboard] 无法打开 /dev/tty（%s），键盘不可用" % exc)

    def pop(self):
        """取出一个新按键（无按键时返回空串）。"""
        self.tick = not getattr(self, "_last_tick", False)
        self._last_tick = self.tick
        key = ""
        if self._msvcrt is not None:
            if self._msvcrt.kbhit():
                key = self._msvcrt.getwch()
        elif self._fd is not None:
            ready, _, _ = self._select.select([self._fd], [], [], 0.002)
            if ready:
                try:
                    key = os.read(self._fd, 1).decode("utf-8", "ignore")
                except OSError:
                    key = ""
        if key:
            key = key.lower()
            if key == "q":
                self.quit = True
        return key

    def stop(self):
        if self._fd is not None:
            try:
                self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)
                os.close(self._fd)
            except Exception:                                     # noqa: BLE001
                pass


def main():
    ap = argparse.ArgumentParser(description="人形机器人双臂搬运负载（MuJoCo + 神经网络平衡）")
    ap.add_argument("--mode", default="demo",
                    choices=["demo", "keyboard", "eval", "collect"])
    ap.add_argument("--controller", default="nn", choices=["open", "pd", "nn"])
    ap.add_argument("--mass", type=float, default=2.0, help="负载质量 kg（0 / 2 / 4）")
    ap.add_argument("--speed", type=float, default=0.15, help="演示速度 m/s")
    ap.add_argument("--seconds", type=float, default=15.0, help="演示时长 s")
    ap.add_argument("--no-viewer", action="store_true", help="不打开可视化窗口")
    ap.add_argument("--log", default="", help="兼容 ROS launch 的日志名，可忽略")
    args = ap.parse_args()

    logger = None
    if args.log:
        try:
            import rospy
            logger = rospy
        except Exception:                                         # noqa: BLE001
            logger = None

    if args.mode == "demo":
        run_demo(args, logger)
    elif args.mode == "keyboard":
        run_keyboard(args, logger)
    elif args.mode == "eval":
        import evaluate
        sys.argv = ["evaluate.py"]
        evaluate.main()
    else:
        import train_mlp
        sys.argv = ["train_mlp.py"]
        train_mlp.main()


if __name__ == "__main__":
    main()
