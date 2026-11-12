#!/usr/bin/env python3
"""离线回放：同一套判据在合成遥测时间线上跑完三类场景。

场景时间线（10 Hz，40 s，与 test_ros_loop_smoke.py 完全一致）：
  0–6 s     健康悬停，学习指令基线（25 帧后冻结）      → INIT → MONITORING
  6–16 s    电机 2 性能衰减：效率 1.00→0.85（8 s 内）
            12–16 s 进一步恶化到 0.72                   → IMBALANCE → EMERGENCY
            （控制器按 50% 程度抬高该电机指令做部分补偿）
  18.4–19.9 s ESC 遥测中断（mixer 指令照常）          → stream_timeout 逐拍降级
  20–26 s   恢复健康                                     → 恢复 MONITORING
  26–34 s   桨损场景：转速正常但 0 号电机必须持续多拉
            24% 悬停指令才能维持姿态（指令偏移判据）    → IMBALANCE

用法：
  python main.py                # 跑回放，打印状态事件与统计
  python main.py --plot DIR     # 额外写出 offline_replay.csv / .svg
  python main.py --ros          # 启动 ROS 2 节点（需要 rclpy）
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from uav_motor_thrust_imbalance_monitor import (  # noqa: E402
        Config,
        MotorImbalanceMonitor,
        MotorSample,
        MotorState,
        clamp_commands,
    )
else:
    from .uav_motor_thrust_imbalance_monitor import (
        Config,
        MotorImbalanceMonitor,
        MotorSample,
        MotorState,
        clamp_commands,
    )

DT = 0.1
FLIGHT_END = 34.0
HOVER = 0.5
N = 4
# ESC 遥测中断窗口（该窗口内不发遥测帧；mixer 指令话题也随帧驱动，一并中断，
# 由看门狗负责降级）
DROPOUT = (18.4, 19.9)


def motor_efficiency(t: float) -> float:
    """电机 2 的转速效率（实测转速 = 指令转速 × 效率）。其余电机恒为 1.0。"""

    if t < 6.0:
        return 1.0
    if t < 8.0:
        return 1.0 - 0.15 * (t - 6.0) / 2.0          # 1.00 → 0.85
    if t < 12.0:
        return 0.85
    if t < 13.0:
        return 0.85 - 0.13 * (t - 12.0)              # 0.85 → 0.72
    if t < 20.0:
        return 0.72
    return 1.0


def controller_commands(t: float) -> List[float]:
    """姿态控制器输出的 mixer 指令。

    健康段四路相等；电机 2 衰减时，控制器以 50% 程度抬高该电机指令
    （完全补偿会让 RPM 跟踪误差消失，把故障掩掉；50% 是保守的中间值，
    相对跟踪误差与补偿程度无关，仍稳定暴露 (1-效率) 的缺口）。
    桨损段（26 s 后）转速效率为 1，但 0 号电机必须多拉 24% 悬停指令。
    """

    cmds = [HOVER] * N
    eff = motor_efficiency(t)
    if eff < 1.0:
        need = HOVER / eff                            # 完全补偿所需指令
        cmds[2] = min(1.0, HOVER + 0.5 * (need - HOVER))
    if t >= 26.0:
        cmds[0] = HOVER + 0.24 * HOVER
    return cmds


def frame(t: float) -> MotorSample:
    cmds = controller_commands(t)
    eff = motor_efficiency(t)
    rpm = []
    for i in range(N):
        nominal = cmds[i] * Config().rpm_max
        e = eff if i == 2 else 1.0
        # 1% 量级的测量噪声，避免判据对理想值过拟合
        rpm.append(nominal * e * (1.0 + 0.005 * math.sin(37.0 * t + i)))
    return MotorSample(t=t, commands=cmds, rpm=rpm)


def run_replay(cfg: Optional[Config] = None) -> Tuple[
    MotorImbalanceMonitor, List[Dict], List[Tuple[float, MotorState, str]], List[MotorSample]
]:
    cfg = cfg or Config()
    monitor = MotorImbalanceMonitor(cfg)
    rows: List[Dict] = []
    events: List[Tuple[float, MotorState, str]] = []
    samples: List[MotorSample] = []
    prev_state = monitor.state
    t = 0.0
    while t <= FLIGHT_END + 1e-9:
        if DROPOUT[0] <= t < DROPOUT[1]:
            t = round(t + DT, 4)
            continue                                  # 遥测中断：看门狗管
        s = frame(t)
        out = monitor.update(s)
        samples.append(s)
        if out.state != prev_state:
            events.append((t, out.state, out.reason))
            prev_state = out.state
        rows.append({
            "t": round(t, 3),
            "state": out.state.value,
            "reason": out.reason,
            "healthy": out.healthy,
            "emergency_land": out.emergency_land,
            "worst_motor": out.worst_motor,
            "track_err2_pct": round(out.track_err_pct[2], 2) if math.isfinite(
                out.track_err_pct[2]) else None,
            "cmd_dev0_pct": round(out.cmd_dev_pct[0], 2) if math.isfinite(
                out.cmd_dev_pct[0]) else None,
            "cmd_dev2_pct": round(out.cmd_dev_pct[2], 2) if math.isfinite(
                out.cmd_dev_pct[2]) else None,
            "cmd2": round(s.commands[2], 4),
            "comp2": round(out.compensation[2], 4),
        })
        t = round(t + DT, 4)
    # 中断窗口内的看门狗事件（按虚拟时间补记，供文档统计）
    return monitor, rows, events, samples


def summarize(rows: Sequence[Dict]) -> str:
    lines = []
    states = [r["state"] for r in rows]
    for name in ("INIT", "MONITORING", "IMBALANCE", "EMERGENCY"):
        lines.append(f"{name:11s} {states.count(name):4d} 帧")
    emg = [r for r in rows if r["emergency_land"]]
    lines.append(f"emergency_land 触发 {len(emg)} 帧"
                 + (f"（首帧 t={emg[0]['t']} s）" if emg else ""))
    comp = [r["comp2"] for r in rows if r["comp2"]]
    if comp:
        lines.append(f"电机 2 补偿指令峰值 {max(comp):.3f}（上限 "
                     f"{Config().comp_limit}）")
    dev0 = [r["cmd_dev0_pct"] for r in rows if r["cmd_dev0_pct"] is not None]
    if dev0:
        lines.append(f"电机 0 指令偏移峰值 {max(dev0):.1f}%（桨损段判据）")
    te2 = [r["track_err2_pct"] for r in rows if r["track_err2_pct"] is not None]
    if te2:
        lines.append(f"电机 2 RPM 跟踪误差峰值 {max(te2):.1f}%")
    return "\n".join(lines)


# --- 证据导出 ---------------------------------------------------------------

def write_csv(path: Path, rows: Sequence[Dict]) -> None:
    import csv
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def write_svg(path: Path, rows: Sequence[Dict]) -> None:
    """两联 SVG：上联状态与判据信号，下联电机 2 指令与补偿。"""

    import csv  # noqa: F401  (保持依赖面与上一模块一致)

    W, H = 860, 460
    ML, MR, MT, MB = 58, 16, 26, 30
    cw, ch = W - ML - MR, (H - MT - MB - 14) // 2
    ts = [r["t"] for r in rows]
    t0, t1 = min(ts), max(ts)

    def sx(t: float) -> float:
        return ML + (t - t0) / max(t1 - t0, 1e-9) * cw

    # 上联：RPM 跟踪误差（%）与指令偏移（%），值域 [-5, 45]
    def sy1(v: float) -> float:
        return MT + ch - (v + 5.0) / 50.0 * ch

    # 下联：指令值 [0.3, 0.8]
    def sy2(v: float) -> float:
        return MT + ch + 14 + ch - (v - 0.3) / 0.5 * ch

    def y_axis1() -> str:
        parts = []
        for v in (0, 10, 25, 40):
            y = sy1(float(v))
            parts.append(
                f'<line x1="{ML}" y1="{y:.1f}" x2="{W - MR}" y2="{y:.1f}"'
                f' stroke="#e2e8f0" stroke-width="1"/>'
                f'<text x="{ML - 6}" y="{y + 4:.1f}" font-size="11"'
                f' text-anchor="end" fill="#64748b">{v}%</text>'
            )
        return "".join(parts)

    def y_axis2() -> str:
        parts = []
        for v in (0.3, 0.4, 0.5, 0.6, 0.7):
            y = sy2(float(v))
            parts.append(
                f'<line x1="{ML}" y1="{y:.1f}" x2="{W - MR}" y2="{y:.1f}"'
                f' stroke="#e2e8f0" stroke-width="1"/>'
                f'<text x="{ML - 6}" y="{y + 4:.1f}" font-size="11"'
                f' text-anchor="end" fill="#64748b">{v:.1f}</text>'
            )
        return "".join(parts)

    def x_axis() -> str:
        parts = []
        t = t0
        while t <= t1 + 1e-9:
            x = sx(t)
            parts.append(
                f'<line x1="{x:.1f}" y1="{MT}" x2="{x:.1f}"'
                f' y2="{MT + 2 * ch + 14}" stroke="#f1f5f9" stroke-width="1"/>'
                f'<text x="{x:.1f}" y="{H - 8}" font-size="11"'
                f' text-anchor="middle" fill="#64748b">{t:.0f}s</text>'
            )
            t += 5.0
        return "".join(parts)

    def poly(key: str, color: str, width_px: float, mapper) -> str:
        def vmap(r):
            return r[key]
        pts = []
        for r in rows:
            v = vmap(r)
            if v is None or not math.isfinite(float(v)):
                continue
            pts.append(f"{sx(r['t']):.1f},{mapper(float(v)):.1f}")
        return (
            f'<polyline fill="none" stroke="{color}" stroke-width="{width_px}"'
            f' stroke-linejoin="round" points="{" ".join(pts)}"/>'
        )

    def state_bands() -> str:
        colors = {
            "MONITORING": "#f1f5f9", "IMBALANCE": "#fef3c7",
            "EMERGENCY": "#fee2e2", "INIT": "#f8fafc",
        }
        parts = []
        cur = rows[0]["state"]
        start = rows[0]["t"]
        for r in rows[1:]:
            if r["state"] != cur:
                parts.append(
                    f'<rect x="{sx(start):.1f}" y="{MT}" width="'
                    f'{sx(r["t"]) - sx(start):.1f}" height="{2 * ch + 14}"'
                    f' fill="{colors.get(cur, "#ffffff")}" opacity="0.6"/>'
                )
                cur, start = r["state"], r["t"]
        parts.append(
            f'<rect x="{sx(start):.1f}" y="{MT}" width="'
            f'{sx(rows[-1]["t"]) - sx(start):.1f}" height="{2 * ch + 14}"'
            f' fill="{colors.get(cur, "#ffffff")}" opacity="0.6"/>'
        )
        return "".join(parts)

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}"
  font-family="system-ui, 'Segoe UI', sans-serif">
  <rect width="{W}" height="{H}" fill="#ffffff"/>
  <text x="{ML}" y="16" font-size="13" fill="#0f172a" font-weight="600">
    电机推力不对称监测：判据信号与状态（离线回放，10 Hz）</text>
  {state_bands()}
  {x_axis()}
  {y_axis1()}
  {poly("track_err2_pct", "#dc2626", 2.0, sy1)}
  {poly("cmd_dev0_pct", "#2563eb", 2.0, sy1)}
  <text x="{W - MR - 4}" y="{sy1(25) - 5:.1f}" font-size="11" text-anchor="end"
   fill="#dc2626">电机2 RPM 跟踪误差（红，右向）</text>
  <text x="{W - MR - 4}" y="{sy1(10) - 5:.1f}" font-size="11" text-anchor="end"
   fill="#2563eb">电机0 指令偏移（蓝，右向）</text>
  <line x1="{ML}" y1="{sy1(10)}" x2="{W - MR}" y2="{sy1(10)}" stroke="#dc2626"
   stroke-width="1" stroke-dasharray="4 3" opacity="0.55"/>
  <line x1="{ML}" y1="{sy1(25)}" x2="{W - MR}" y2="{sy1(25)}" stroke="#dc2626"
   stroke-width="1" stroke-dasharray="4 3" opacity="0.8"/>
  {y_axis2()}
  {poly("cmd2", "#0891b2", 2.0, sy2)}
  {poly("comp2", "#7c3aed", 2.0, sy2)}
  <text x="{W - MR - 4}" y="{sy2(0.72):.1f}" font-size="11" text-anchor="end"
   fill="#0891b2">电机2 mixer 指令（青）</text>
  <text x="{W - MR - 4}" y="{sy2(0.66):.1f}" font-size="11" text-anchor="end"
   fill="#7c3aed">电机2 建议补偿（紫）</text>
</svg>'''
    path.write_text(svg, encoding="utf-8")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plot", metavar="DIR",
                        help="写出 offline_replay.csv / offline_replay.svg")
    parser.add_argument("--config", help="JSON 覆盖 Config 字段")
    parser.add_argument("--show-config", action="store_true")
    parser.add_argument("--ros", action="store_true",
                        help="启动 ROS 2 节点（需要 rclpy 与 ROS 2 环境）")
    args = parser.parse_args(argv)

    if args.ros:
        try:
            from uav_motor_thrust_imbalance_monitor.motor_imbalance_node import (
                main as ros_main,
            )
        except ImportError as exc:
            print(
                "[错误] 未找到 rclpy / ROS 2 消息包，无法启动 ROS 节点。\n"
                "       请在已 source /opt/ros/<distro>/setup.bash 的环境中运行；\n"
                "       离线回放不需要 ROS，直接运行 `python main.py` 即可。",
                file=sys.stderr,
            )
            print(f"       原始错误: {exc}", file=sys.stderr)
            return 2
        return ros_main()

    cfg = Config(**json.loads(args.config)) if args.config else Config()
    if args.show_config:
        print(json.dumps(asdict(cfg), ensure_ascii=False, indent=2))
        return 0

    monitor, rows, events, _ = run_replay(cfg)
    print(f"回放 {len(rows)} 帧（{rows[0]['t']}–{rows[-1]['t']} s，含 "
          f"{DROPOUT[1] - DROPOUT[0]:.1f} s 遥测中断）")
    print("\n状态变化事件：")
    for t, st, why in events:
        print(f"  {t:6.2f} s  → {st.value:11s} ({why})")
    print("\n统计：")
    print(summarize(rows))
    if args.plot:
        out = Path(args.plot)
        out.mkdir(parents=True, exist_ok=True)
        write_csv(out / "offline_replay.csv", rows)
        write_svg(out / "offline_replay.svg", rows)
        print(f"\n已写出: {out / 'offline_replay.csv'}")
        print(f"已写出: {out / 'offline_replay.svg'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
