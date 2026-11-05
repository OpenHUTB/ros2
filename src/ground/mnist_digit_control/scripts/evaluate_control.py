#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制性能评价：验证数字指令 -> 动作执行的控制精度。

方法：只启动 turtlesim + 数字指令控制节点（无检测节点），
由本脚本扮演检测节点：向 /digit_detector/digit 发布识别数字、
向 /digit_detector/confidence 发布置信度（0.95），对每个动作重复 3 次，统计：
    - 旋转动作（1/3/7/9/0）：累计转角相对目标角度的误差（度）；
    - 平移动作（2/8/4/6）：单次动作位移（turtlesim 世界坐标单位）。

每次测试前通过 /turtle1/teleport_absolute 把乌龟复位到 (5.5, 5.5, 0)；
两次动作之间发布 6 帧 -1（模拟数字从画面消失），使控制节点允许同一数字再次触发。

用法（需要先在另一终端启动 turtlesim 与 commander）：
    python3 scripts/evaluate_control.py
输出：artifacts/control_eval.json 与 artifacts/control_heading.png
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String

try:
    from turtlesim_msgs.msg import Pose
    from turtlesim_msgs.srv import TeleportAbsolute
except ImportError:  # 兼容消息绑定在 turtlesim.msg 下的环境
    from turtlesim.msg import Pose
    from turtlesim.srv import TeleportAbsolute

MODULE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

TESTS = [
    {"digit": 7, "name": "左转90度", "kind": "rotate", "target_deg": 90.0},
    {"digit": 9, "name": "右转90度", "kind": "rotate", "target_deg": -90.0},
    {"digit": 1, "name": "左转45度", "kind": "rotate", "target_deg": 45.0},
    {"digit": 3, "name": "右转45度", "kind": "rotate", "target_deg": -45.0},
    {"digit": 0, "name": "原地旋转360度", "kind": "rotate", "target_deg": 360.0},
    {"digit": 2, "name": "前进", "kind": "translate"},
    {"digit": 4, "name": "左移", "kind": "translate"},
    {"digit": 8, "name": "后退", "kind": "translate"},
    {"digit": 6, "name": "右移", "kind": "translate"},
]
REPEATS = 3
CONF = 0.95  # 模拟检测节点输出的置信度（高于门限 0.7）


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ControlEval(Node):
    def __init__(self) -> None:
        super().__init__("control_eval")
        self.pub_digit = self.create_publisher(Int32, "/digit_detector/digit", 10)
        self.pub_conf = self.create_publisher(Float32, "/digit_detector/confidence", 10)
        self.pub_cmd = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_subscription(Pose, "/turtle1/pose", self._on_pose, 10)
        self.create_subscription(String, "/mnist_digit_control/action", self._on_action, 10)
        self.cli_tp = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        self.pose = None
        self.action = ""
        self.saw_action = False    # send_digit 期间曾观察到动作开始（闩锁）
        self.yaw_series = []
        self.yaw_prev = None
        self.yaw_accum = 0.0
        self.t_rec0 = time.monotonic()

    def _on_pose(self, msg: Pose) -> None:
        self.pose = msg
        if self.yaw_prev is not None:
            self.yaw_accum += wrap(msg.theta - self.yaw_prev)
            self.yaw_series.append((time.monotonic() - self.t_rec0,
                                    math.degrees(self.yaw_accum)))
        self.yaw_prev = msg.theta

    def _on_action(self, msg: String) -> None:
        self.action = msg.data
        if msg.data != "":
            self.saw_action = True

    def reset(self) -> None:
        while not self.cli_tp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待 teleport 服务……")
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = 5.5, 5.5, 0.0
        self.cli_tp.call_async(req)
        self.yaw_prev = None
        self.yaw_accum = 0.0
        self.yaw_series = []
        self.t_rec0 = time.monotonic()
        time.sleep(0.5)

    def send_digit(self, digit: int, seconds: float = 1.2, hz: float = 12.0) -> None:
        """以 hz 频率发送同一数字与置信度 seconds 秒，模拟检测节点的连续输出。"""
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.pub_digit.publish(Int32(data=digit))
            self.pub_conf.publish(Float32(data=CONF))
            rclpy.spin_once(self, timeout_sec=1.0 / hz)

    def send_idle(self, seconds: float = 0.6, hz: float = 12.0) -> None:
        """发布 -1（数字消失），使控制节点复位 last_executed，允许同数字再次触发。"""
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.pub_digit.publish(Int32(data=-1))
            self.pub_conf.publish(Float32(data=0.0))
            rclpy.spin_once(self, timeout_sec=1.0 / hz)

    def wait_started(self, timeout: float = 6.0) -> bool:
        """等待动作开始（action 话题变为非空）。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.action != "":
                return True
        return False

    def wait_done(self, timeout: float = 30.0) -> bool:
        """等待动作执行完成（action 话题回到空字符串）。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.action == "":
                return True
        return False

    def wait_idle(self, timeout: float = 10.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.action == "":
                return True
        return False

    def run_one(self, digit: int, kind: str, target_deg: float) -> dict:
        self.wait_idle()
        self.send_idle()          # 数字消失 -> 复位 last_executed
        self.reset()
        self.saw_action = False
        self.send_digit(digit)    # 1.2 s，足够填满 6 帧稳定窗口
        # 平移动作仅持续 1.0 s，可能在 send_digit 期间就已开始并结束，
        # 所以先检查闩锁标志，再回退到显式等待
        if self.saw_action:
            ok = self.wait_done()
        else:
            ok = self.wait_started() and self.wait_done()
        time.sleep(0.5)           # 等 cooldown 结束
        rclpy.spin_once(self, timeout_sec=0.1)
        result = {"digit": digit, "finished": bool(ok)}
        if kind == "rotate":
            result["achieved_deg"] = math.degrees(self.yaw_accum)
            result["error_deg"] = abs(math.degrees(self.yaw_accum) - target_deg)
        else:
            if self.pose is not None:
                result["dx"] = float(self.pose.x - 5.5)
                result["dy"] = float(self.pose.y - 5.5)
                result["displacement"] = float(math.hypot(self.pose.x - 5.5,
                                                          self.pose.y - 5.5))
        result["yaw_series"] = self.yaw_series
        return result


def main() -> None:
    rclpy.init()
    node = ControlEval()
    all_results = []
    for test in TESTS:
        for rep in range(REPEATS):
            r = node.run_one(test["digit"], test["kind"], test.get("target_deg", 0.0))
            r["name"] = test["name"]
            r["target_deg"] = test.get("target_deg", 0.0)
            r["rep"] = rep + 1
            all_results.append(r)
            node.get_logger().info(
                f"{test['name']} #{rep + 1}: finished={r['finished']} "
                + (f"error={r['error_deg']:.2f}°" if test["kind"] == "rotate"
                   else f"disp={r.get('displacement', 0):.3f}"))
    node.pub_cmd.publish(Twist())  # 停车

    # 汇总
    summary = []
    for test in TESTS:
        rows = [r for r in all_results if r["name"] == test["name"]]
        row = {"name": test["name"], "digit": test["digit"], "kind": test["kind"]}
        if test["kind"] == "rotate":
            errs = [r["error_deg"] for r in rows]
            row.update({"target_deg": test["target_deg"],
                        "mean_abs_error_deg": float(np.mean(errs)),
                        "max_abs_error_deg": float(np.max(errs))})
        else:
            disps = [r.get("displacement", 0.0) for r in rows]
            row.update({"mean_displacement": float(np.mean(disps)),
                        "std_displacement": float(np.std(disps))})
        summary.append(row)

    out_dir = os.path.join(MODULE_DIR, "artifacts")
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "control_eval.json"), "w", encoding="utf-8") as fh:
        json.dump({"summary": summary, "detail": all_results}, fh,
                  ensure_ascii=False, indent=2)

    plot_results(all_results, out_dir)

    print("| 动作 | 目标 | 指标 |")
    print("| --- | --- | --- |")
    for row in summary:
        if row["kind"] == "rotate":
            print(f"| {row['name']} | {row['target_deg']:.0f}° | "
                  f"平均绝对误差 {row['mean_abs_error_deg']:.2f}°，"
                  f"最大 {row['max_abs_error_deg']:.2f}° |")
        else:
            print(f"| {row['name']} | 定时长开环 | 平均位移 {row['mean_displacement']:.3f} "
                  f"± {row['std_displacement']:.3f} |")
    rclpy.shutdown()


ROT_LABELS = {"左转45度": "+45 deg", "右转45度": "-45 deg",
              "左转90度": "+90 deg", "右转90度": "-90 deg",
              "原地旋转360度": "360 deg"}


def plot_results(all_results, out_dir) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(9.6, 3.6), dpi=120)
    # 左图：+90° 旋转的角度跟踪曲线（3 次）
    for rep in range(1, REPEATS + 1):
        rows = [r for r in all_results if r["name"] == "左转90度" and r["rep"] == rep]
        if rows and rows[0]["yaw_series"]:
            ts, ys = zip(*rows[0]["yaw_series"])
            axes[0].plot(ts, ys, label=f"run {rep}")
    axes[0].axhline(90, color="gray", ls="--", lw=1)
    axes[0].set_xlabel("time (s)")
    axes[0].set_ylabel("accumulated yaw (deg)")
    axes[0].set_title("+90 deg turn closed-loop tracking")
    axes[0].legend(fontsize=8)
    # 右图：各旋转动作的误差柱状图
    names, means, maxs = [], [], []
    for name in ("左转45度", "右转45度", "左转90度", "右转90度", "原地旋转360度"):
        rows = [r for r in all_results if r["name"] == name]
        if rows:
            names.append(ROT_LABELS[name])
            means.append(np.mean([r["error_deg"] for r in rows]))
            maxs.append(np.max([r["error_deg"] for r in rows]))
    x = np.arange(len(names))
    axes[1].bar(x - 0.18, means, width=0.36, label="mean abs err")
    axes[1].bar(x + 0.18, maxs, width=0.36, label="max abs err")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(names, fontsize=8, rotation=15)
    axes[1].set_ylabel("error (deg)")
    axes[1].set_title("rotation error per command")
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "control_heading.png"))
    plt.close(fig)


if __name__ == "__main__":
    main()
