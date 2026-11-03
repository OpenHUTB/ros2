#!/usr/bin/env python3
"""离线回放入口：零第三方依赖，直接 `python main.py` 即可运行整个模块。

    python main.py                         # 打印离线回放结果
    python main.py --plot ./outputs        # 额外输出 CSV 和 SVG 曲线
    python main.py --config '{"max_speed": 8.0}'   # 覆盖判据
    python main.py --ros                   # 启动 ROS 2 节点（需要 rclpy）

离线回放是合成定位流，不连接仿真器、不模拟飞行动力学，
只用于验证判据、状态机与输出位姿；在线闭环验证范围见 README。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

# 允许以脚本方式直接运行，也允许 `python -m uav_gnss_integrity_monitor.main`
sys.path.insert(0, str(Path(__file__).resolve().parent))

from uav_gnss_integrity_monitor import Config, Fix, Monitor  # noqa: E402

DT = 0.1
N_SAMPLES = 110


@dataclass(frozen=True)
class Sample:
    truth: Fix
    fix: Fix
    note: str


def build_samples() -> List[Sample]:
    """构造一条匀速直线 + 轻微起伏的合成轨迹，并注入五类定位异常。"""
    samples: List[Sample] = []
    for i in range(N_SAMPLES):
        t = round(i * DT, 6)
        x = 0.8 * t
        y = 0.6 * math.sin(0.5 * t)
        z = 5.0 + 0.05 * math.sin(t)
        truth = Fix(t, x, y, z)
        fix = Fix(t, x, y, z, fix_type=3, num_sats=12, accuracy=0.4)
        note = "normal"

        if 20 <= i < 24:
            # 多路径 / 重新捕获导致的 +25 m 位置突跳
            fix = Fix(t, x + 25.0, y + 3.0, z, 3, 12, 0.4)
            note = "position_jump"
        elif i == 35:
            # 降级为 2D 定位、卫星数不足
            fix = Fix(t, x, y, z, 1, 4, 0.4)
            note = "degraded_fix"
        elif 45 <= i < 48:
            # 水平精度恶化
            fix = Fix(t, x, y, z, 3, 9, 8.0)
            note = "poor_accuracy"
        elif i == 60:
            # 非有限数值
            fix = Fix(t, float("nan"), y, z, 3, 12, 0.4)
            note = "non_finite"
        elif 70 <= i < 88:
            # 定位数据流完全中断 1.9 s
            continue

        samples.append(Sample(truth=truth, fix=fix, note=note))
    return samples


def run_replay(cfg: Config) -> Tuple[List[Dict], List[Tuple[float, str, str]], Dict]:
    monitor = Monitor(cfg)
    rows: List[Dict] = []
    events: List[Tuple[float, str, str]] = []
    prev_state: Optional[str] = None

    for idx, sample in enumerate(build_samples()):
        out = monitor.update(sample.fix)
        rows.append(
            {
                "i": idx,
                "t": sample.fix.t,
                "true_x": sample.truth.x,
                "true_y": sample.truth.y,
                "true_z": sample.truth.z,
                "raw_x": sample.fix.x,
                "output_x": out.x,
                "output_y": out.y,
                "output_z": out.z,
                "state": out.state.value,
                "valid": out.valid,
                "action": out.action,
                "reason": out.reason,
                "note": sample.note,
            }
        )
        if out.state.value != prev_state:
            events.append((sample.fix.t, out.state.value, out.reason))
            prev_state = out.state.value

    raw_errors = [
        abs(r["raw_x"] - r["true_x"])
        for r in rows
        if math.isfinite(r["raw_x"])
    ]
    output_errors = [abs(r["output_x"] - r["true_x"]) for r in rows]
    rejects: Dict[str, int] = {}
    for r in rows:
        if not r["valid"]:
            rejects[r["reason"]] = rejects.get(r["reason"], 0) + 1
    recoveries = sum(
        1
        for j in range(1, len(events))
        if events[j - 1][1] == "HOLD" and events[j][1] == "TRACKING"
    )

    metrics = {
        "samples": len(rows),
        "accepted_frames": sum(1 for r in rows if r["valid"]),
        "hold_frames": sum(1 for r in rows if r["state"] == "HOLD"),
        "raw_max_abs_error_m": round(max(raw_errors), 3),
        "output_max_abs_error_m": round(max(output_errors), 3),
        "recovery_count": recoveries,
        "rejected_reasons": rejects,
    }
    return rows, events, metrics


def write_csv(path: Path, rows: Sequence[Dict]) -> None:
    fields = [
        "i", "t", "true_x", "true_y", "true_z", "raw_x",
        "output_x", "output_y", "output_z", "state", "valid",
        "action", "reason", "note",
    ]
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row[k] for k in fields})


def write_svg(path: Path, rows: Sequence[Dict]) -> None:
    """把真实回放结果画成带标注的矢量曲线（纯标准库生成，无外部依赖）。"""
    width, height = 920, 420
    left, right, top, bottom = 72, 890, 54, 356
    t0 = min(r["t"] for r in rows)
    t1 = max(r["t"] for r in rows)
    y_lo, y_hi = -1.0, 12.0

    def sx(t: float) -> float:
        return left + (t - t0) / (t1 - t0) * (right - left)

    def sy(v: float) -> float:
        clipped = min(max(v, y_lo), y_hi)
        return bottom - (clipped - y_lo) / (y_hi - y_lo) * (bottom - top)

    def series(key: str, color: str, width_px: int) -> str:
        # 非有限值 / 数据流中断处断开折线，并把后续点继续画出来
        segments: List[List[str]] = []
        current: List[str] = []
        for r in rows:
            value = r[key]
            if not math.isfinite(value):
                if len(current) > 1:
                    segments.append(current)
                current = []
                continue
            current.append(f"{sx(r['t']):.1f},{sy(value):.1f}")
        if len(current) > 1:
            segments.append(current)
        return "".join(
            f'<polyline fill="none" stroke="{color}" stroke-width="{width_px}" '
            f'stroke-linejoin="round" points="{" ".join(seg)}"/>'
            for seg in segments
        )

    # HOLD 区间用浅色竖带标出
    bands: List[str] = []
    start: Optional[float] = None
    for r in rows:
        if r["state"] in ("HOLD", "SUSPECT") and start is None:
            start = r["t"]
        elif r["state"] not in ("HOLD", "SUSPECT") and start is not None:
            bands.append(
                f'<rect x="{sx(start):.1f}" y="{top}" width="{sx(r["t"]) - sx(start):.1f}" '
                f'height="{bottom - top}" fill="#d62728" opacity="0.10"/>'
            )
            start = None
    if start is not None:
        bands.append(
            f'<rect x="{sx(start):.1f}" y="{top}" width="{sx(t1) - sx(start):.1f}" '
            f'height="{bottom - top}" fill="#d62728" opacity="0.10"/>'
        )

    grid: List[str] = []
    for v in (0, 3, 6, 9, 12):
        y = sy(v)
        grid.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{right}" y2="{y:.1f}" '
            f'stroke="#e3e6ea" stroke-width="1"/>'
        )
        grid.append(
            f'<text x="{left - 10}" y="{y + 4:.1f}" text-anchor="end" '
            f'font-size="11" fill="#4a4f55">{v}</text>'
        )
    for t in range(0, int(t1) + 1, 2):
        x = sx(t)
        grid.append(
            f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{bottom}" '
            f'stroke="#f0f2f4" stroke-width="1"/>'
        )
        grid.append(
            f'<text x="{x:.1f}" y="{bottom + 20}" text-anchor="middle" '
            f'font-size="11" fill="#4a4f55">{t}</text>'
        )

    # 超出坐标范围的跳变单独标注，避免把正常轨迹压扁
    clipped = [
        r for r in rows
        if math.isfinite(r["raw_x"]) and r["raw_x"] > y_hi
    ]
    clip_marks: List[str] = []
    if clipped:
        r = clipped[0]
        clip_marks.append(
            f'<text x="{sx(r["t"]):.1f}" y="{top + 14}" text-anchor="middle" '
            f'font-size="11" fill="#d62728">↑ 原始定位突跳 +25 m（超出坐标范围，已截断显示）</text>'
        )

    legend = [
        ('<line x1="600" y1="26" x2="632" y2="26" stroke="#5b6068" stroke-width="2"/>'
         '<text x="638" y="30" font-size="11" fill="#30343a">真实轨迹</text>'),
        ('<line x1="710" y1="26" x2="742" y2="26" stroke="#d62728" stroke-width="2"/>'
         '<text x="748" y="30" font-size="11" fill="#30343a">原始 GNSS</text>'),
        ('<line x1="820" y1="26" x2="852" y2="26" stroke="#1f77b4" stroke-width="2"/>'
         '<text x="858" y="30" font-size="11" fill="#30343a">门控输出</text>'),
    ]

    svg = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="{width}" height="{height}" fill="#ffffff"/>
  <text x="{left}" y="24" font-size="14" fill="#1b1f24">GNSS 定位完整性门控 · 离线回放（x 方向位置）</text>
  {''.join(legend)}
  {''.join(bands)}
  {''.join(grid)}
  <line x1="{left}" y1="{bottom}" x2="{right}" y2="{bottom}" stroke="#8a9098" stroke-width="1"/>
  <line x1="{left}" y1="{top}" x2="{left}" y2="{bottom}" stroke="#8a9098" stroke-width="1"/>
  {series('true_x', '#5b6068', 2)}
  {series('raw_x', '#d62728', 2)}
  {series('output_x', '#1f77b4', 2)}
  {''.join(clip_marks)}
  <text x="{(left + right) / 2:.0f}" y="{height - 12}" text-anchor="middle" font-size="11" fill="#4a4f55">时间 t (s)</text>
  <text x="16" y="{(top + bottom) / 2:.0f}" transform="rotate(-90 16 {(top + bottom) / 2:.0f})" text-anchor="middle" font-size="11" fill="#4a4f55">x 位置 (m)</text>
  <text x="{left}" y="{height - 30}" font-size="10" fill="#7a8088">浅红竖带 = SUSPECT / HOLD 区间（门控冻结输出位姿，下游应保持悬停）</text>
</svg>
"""
    path.write_text(svg, encoding="utf-8")


def print_report(cfg: Config, rows: Sequence[Dict],
                 events: Sequence[Tuple[float, str, str]], metrics: Dict) -> None:
    print("=" * 66)
    print("GNSS 定位完整性门控 · 离线回放（合成定位流，非仿真闭环）")
    print("=" * 66)
    print("配置: " + json.dumps({k: v for k, v in asdict(cfg).items()},
                              ensure_ascii=False, sort_keys=True))
    print(f"样本数 {metrics['samples']}，其中被接受 {metrics['accepted_frames']}，"
          f"HOLD 帧 {metrics['hold_frames']}")
    print(f"原始定位最大绝对误差 {metrics['raw_max_abs_error_m']} m；"
          f"门控输出最大绝对误差 {metrics['output_max_abs_error_m']} m")
    print(f"从 HOLD 恢复到 TRACKING 的次数: {metrics['recovery_count']}")
    print("拒绝原因统计: " + json.dumps(metrics["rejected_reasons"],
                                     ensure_ascii=False, sort_keys=True))
    print("-" * 66)
    print("状态变化事件 (t / 状态 / 原因):")
    for t, state, reason in events:
        print(f"  t={t:5.2f}  {state:<9} {reason}")
    print("-" * 66)
    print("最后 8 帧 (t / 原始x / 输出x / 状态 / 原因):")
    for r in rows[-8:]:
        raw = "nan" if not math.isfinite(r["raw_x"]) else f"{r['raw_x']:7.2f}"
        print(f"  t={r['t']:5.2f}  {raw}  {r['output_x']:7.2f}  "
              f"{r['state']:<9} {r['reason']}")
    print("=" * 66)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="UAV GNSS 定位完整性监测模块的离线回放入口"
    )
    parser.add_argument("--plot", metavar="DIR", default=None,
                        help="把回放结果写成 CSV 和 SVG 到指定目录")
    parser.add_argument("--config", metavar="JSON", default=None,
                        help='覆盖默认判据，例如 \'{"max_speed": 8.0}\'')
    parser.add_argument("--show-config", action="store_true",
                        help="打印默认配置后退出")
    parser.add_argument("--ros", action="store_true",
                        help="启动 ROS 2 节点（需要 rclpy 与 ROS 2 环境）")
    args = parser.parse_args(argv)

    if args.ros:
        try:
            from uav_gnss_integrity_monitor.gnss_integrity_node import (
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

    rows, events, metrics = run_replay(cfg)
    print_report(cfg, rows, events, metrics)

    if args.plot:
        out_dir = Path(args.plot)
        out_dir.mkdir(parents=True, exist_ok=True)
        csv_path = out_dir / "offline_replay.csv"
        svg_path = out_dir / "offline_replay.svg"
        write_csv(csv_path, rows)
        write_svg(svg_path, rows)
        print(f"已输出: {csv_path}")
        print(f"已输出: {svg_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
