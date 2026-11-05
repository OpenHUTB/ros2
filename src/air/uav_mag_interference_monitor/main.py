#!/usr/bin/env python3
"""离线回放入口：零第三方依赖，直接 `python main.py` 即可运行整个模块。

    python main.py                         # 打印离线回放结果
    python main.py --plot ./outputs        # 额外输出 CSV 和 SVG 曲线
    python main.py --config '{"min_ground_speed": 3.0}'   # 覆盖判据
    python main.py --ros                   # 启动 ROS 2 节点（需要 rclpy）

离线回放用合成的磁场与航迹数据，不连接仿真器、不模拟飞行动力学，
只用于验证判据、状态机与输出航向；在线闭环验证范围见 README。
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

# 允许以脚本方式直接运行，也允许 `python -m uav_mag_interference_monitor.main`
sys.path.insert(0, str(Path(__file__).resolve().parent))

from uav_mag_interference_monitor import (  # noqa: E402
    Config,
    FlightSample,
    HeadingMonitor,
    MagSample,
    PreflightResult,
    fit_circle,
    run_preflight,
)

DT = 0.1
EARTH_H = 22.0           # 地磁场水平分量 (µT)
EARTH_V = -42.0          # 地磁场垂直分量 (µT)，向下为负
BIAS_IDLE = (4.0, -2.5, 1.2)   # 机体硬铁偏差（安装误差 + 机载电子）
MOTOR_OFFSET = (5.6, -3.2, 1.6)  # 上油门时动力系统叠加的磁场

FLIGHT_SECONDS = 34.0


def _rotate_z(v: tuple, yaw: float) -> Tuple[float, float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2])


def _noise(i: int, amp: float) -> float:
    """确定性伪噪声，保证回放结果可复现。"""

    return amp * math.sin(12.9898 * i + 78.233) * math.cos(3.7 * i)


def build_preflight_samples(
    n: int = 60, offset: Tuple[float, float, float] = (0.0, 0.0, 0.0), seed: int = 0
) -> List[MagSample]:
    """合成一圈整机旋转的磁场样本（机体坐标系下的地磁 + 硬铁偏差）。"""

    out: List[MagSample] = []
    for i in range(n):
        yaw = 2.0 * math.pi * i / n
        # 世界系地磁 (H, 0, V) 转到机体系：x 前、y 右（NED/FRD），
        # 与判据层 heading = atan2(my, mx) 的约定保持一致。
        body = _rotate_z((EARTH_H, 0.0, EARTH_V), yaw)
        mx = body[0] + BIAS_IDLE[0] + offset[0] + _noise(i + seed, 0.35)
        my = body[1] + BIAS_IDLE[1] + offset[1] + _noise(i + seed + 100, 0.35)
        mz = body[2] + BIAS_IDLE[2] + offset[2] + _noise(i + seed + 200, 0.35)
        out.append(MagSample(round(i * 0.05, 4), mx, my, mz))
    return out


@dataclass(frozen=True)
class FlightTruth:
    course: float        # 真实航向 / 航迹向 (rad)
    interference: float  # 该时刻注入的横向干扰强度 (µT)


def interference_profile(t: float) -> Tuple[float, float]:
    """返回 (横向干扰, 沿航向干扰)，均为机体坐标系下的磁场叠加量 (µT)。

    横向（垂直于水平地磁）干扰把磁航向"推偏"，几乎不改变模值，对应机载
    大电流线束产生的旋转磁场；沿航向干扰主要抬高模值、几乎不改变航向，
    对应贴近铁磁结构（起落架、电池仓）导致的模值异常。两类问题分别由
    航向一致性与模值判据覆盖。
    """

    cross = 0.0
    along = 0.0
    if 8.0 <= t < 10.0:
        cross = 13.0 * (t - 8.0) / 2.0
    elif 10.0 <= t < 18.0:
        cross = 13.0
    elif 18.0 <= t < 20.0:
        cross = 13.0 * (20.0 - t) / 2.0
    if 26.0 <= t < 27.0:
        along = 18.0 * (t - 26.0)
    elif 27.0 <= t < 31.0:
        along = 18.0
    elif 31.0 <= t < 32.0:
        along = 18.0 * (32.0 - t)
    return cross, along


def build_flight_samples(
    true_course: float = math.radians(45.0),
    speed: float = 4.5,
    bias: Sequence[float] = BIAS_IDLE,
) -> List[Tuple[FlightSample, FlightTruth]]:
    """合成直线平飞段：真实航向不变，中途注入两类磁场干扰。

    采样分三段，其中 22.4–23.2 s 故意丢帧，用来验证数据流超时判据。
    """

    out: List[Tuple[FlightSample, FlightTruth]] = []
    steps = int(round(FLIGHT_SECONDS / DT))
    for i in range(steps + 1):
        t = round(i * DT, 4)
        if 22.3 <= t < 23.6:
            continue  # 模拟磁场话题中断 1.3 s（超过 stale_timeout）
        cross, along = interference_profile(t)
        body = _rotate_z((EARTH_H, 0.0, EARTH_V), true_course)
        # 沿航向单位矢量 (hx, hy) 与它的左法向（横向）
        hx, hy = _rotate_z((1.0, 0.0, 0.0), true_course)[:2]
        cx, cy = -hy, hx
        mx = body[0] + bias[0] + cross * cx + along * hx + _noise(i, 0.3)
        my = body[1] + bias[1] + cross * cy + along * hy + _noise(i + 50, 0.3)
        mz = body[2] + BIAS_IDLE[2] + _noise(i + 90, 0.3)
        sample = FlightSample(
            t=t,
            mx=mx,
            my=my,
            mz=mz,
            speed=speed,
            course=true_course,
        )
        out.append((sample, FlightTruth(true_course, cross + along)))
    return out


def run_replay(cfg: Config) -> Tuple[Dict, List[Dict], List[Tuple[float, str, str]], Dict]:
    """跑完起飞前自检与飞行段回放，返回自检结论、逐帧记录、事件与指标。"""

    idle = build_preflight_samples(60, (0.0, 0.0, 0.0), seed=0)
    clean = build_preflight_samples(60, (0.0, 0.0, 0.0), seed=7)
    noisy = build_preflight_samples(60, MOTOR_OFFSET, seed=13)

    preflight_ok: PreflightResult = run_preflight(idle, clean, cfg)
    preflight_bad: PreflightResult = run_preflight(idle, noisy, cfg)

    calibration = fit_circle(idle)
    flight = build_flight_samples(bias=calibration.bias)
    # 用自检得到的水平半径作为模值基准，而不是拿首帧当基准
    monitor = HeadingMonitor(
        calibration.bias, cfg, expected_radius=calibration.radius
    )

    rows: List[Dict] = []
    events: List[Tuple[float, str, str]] = []
    prev_state: Optional[str] = None
    for sample, truth in flight:
        out = monitor.update(sample)
        rows.append(
            {
                "t": sample.t,
                "true_course_deg": math.degrees(truth.course),
                "mag_heading_deg": math.degrees(wrap(sample, monitor.bias)),
                "output_heading_deg": math.degrees(out.heading),
                "state": out.state.value,
                "source": out.heading_source,
                "valid": out.valid,
                "heading_error_deg": out.heading_error_deg,
                "field_norm_pct": out.field_norm_pct,
                "interference_ut": truth.interference,
                "reason": out.reason,
            }
        )
        if out.state.value != prev_state:
            events.append((sample.t, out.state.value, out.reason))
            prev_state = out.state.value

    switched = sum(
        1
        for j in range(1, len(rows))
        if rows[j]["source"] == "gnss_course" and rows[j - 1]["source"] != "gnss_course"
    )
    recovered = sum(
        1
        for j in range(1, len(events))
        if events[j - 1][1] in ("DEGRADED", "UNRELIABLE") and events[j][1] == "TRUSTED"
    )
    valid_rows = [r for r in rows if r["source"] == "gnss_course"]
    output_errors = [
        abs(((r["output_heading_deg"] - r["true_course_deg"] + 180.0) % 360.0) - 180.0)
        for r in rows
    ]
    mag_errors = [
        abs(((r["mag_heading_deg"] - r["true_course_deg"] + 180.0) % 360.0) - 180.0)
        for r in rows
    ]

    metrics = {
        "preflight_clean": preflight_ok.state.value,
        "preflight_clean_shift_pct": round(preflight_ok.bias_shift_pct, 2),
        "preflight_interfered": preflight_bad.state.value,
        "preflight_interfered_shift_pct": round(preflight_bad.bias_shift_pct, 2),
        "frames": len(rows),
        "degraded_frames": sum(1 for r in rows if r["state"] == "DEGRADED"),
        "unreliable_frames": sum(1 for r in rows if r["state"] == "UNRELIABLE"),
        "gnss_course_frames": len(valid_rows),
        "heading_source_switches": switched,
        "recovery_count": recovered,
        "mag_max_error_deg": round(max(mag_errors), 2),
        "output_max_error_deg": round(max(output_errors), 2),
    }
    return {"ok": preflight_ok, "bad": preflight_bad}, rows, events, metrics


def wrap(sample: FlightSample, bias: Sequence[float]) -> float:
    """回放报表用的磁航向（未门控的原始值）。"""

    cx = sample.mx - bias[0]
    cy = sample.my - bias[1]
    return math.atan2(cy, cx)


def load_mag_csv(path: Path) -> List[MagSample]:
    """读取真实磁力计日志：CSV 至少包含 t,mx,my,mz 四列（µT）。

    额外列会被忽略，方便直接把飞控日志导出后使用。
    """

    rows: List[MagSample] = []
    with path.open(encoding="utf-8-sig", newline="") as fh:
        for rec in csv.DictReader(fh):
            try:
                rows.append(
                    MagSample(
                        float(rec["t"]),
                        float(rec["mx"]),
                        float(rec["my"]),
                        float(rec["mz"]),
                    )
                )
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"{path} 缺少 t/mx/my/mz 列或数值非法: {exc}"
                ) from exc
    return rows


def write_mag_csv(path: Path, samples: Sequence[MagSample]) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t", "mx", "my", "mz"])
        for s in samples:
            writer.writerow([f"{s.t:.3f}", f"{s.mx:.3f}", f"{s.my:.3f}", f"{s.mz:.3f}"])


def write_csv(path: Path, rows: Sequence[Dict]) -> None:
    fields = [
        "t", "true_course_deg", "mag_heading_deg", "output_heading_deg",
        "state", "source", "valid", "heading_error_deg", "field_norm_pct",
        "interference_ut", "reason",
    ]
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row[k] for k in fields})


def write_svg(path: Path, rows: Sequence[Dict]) -> None:
    """两联图：上=航向对比，下=磁场模值与航向偏差（纯标准库生成）。"""

    width, height = 920, 620
    left, right = 78, 884
    top1, bottom1 = 56, 300
    top2, bottom2 = 396, 560
    t0 = min(r["t"] for r in rows)
    t1 = max(r["t"] for r in rows)
    h_lo, h_hi = 0.0, 90.0
    n_lo, n_hi = 60.0, 140.0

    def sx(t: float) -> float:
        return left + (t - t0) / (t1 - t0) * (right - left)

    def sy1(v: float) -> float:
        c = min(max(v, h_lo), h_hi)
        return bottom1 - (c - h_lo) / (h_hi - h_lo) * (bottom1 - top1)

    def sy2(v: float) -> float:
        c = min(max(v, n_lo), n_hi)
        return bottom2 - (c - n_lo) / (n_hi - n_lo) * (bottom2 - top2)

    def poly(key: str, color: str, width_px: int, mapper) -> str:
        pts = [
            f"{sx(r['t']):.1f},{mapper(r[key]):.1f}"
            for r in rows
            if isinstance(r[key], (int, float)) and math.isfinite(r[key])
        ]
        return (
            f'<polyline fill="none" stroke="{color}" stroke-width="{width_px}" '
            f'stroke-linejoin="round" points="{" ".join(pts)}"/>'
        )

    # 降级/不可信区间竖带
    bands: List[str] = []
    start: Optional[float] = None
    for r in rows:
        bad = r["state"] in ("DEGRADED", "UNRELIABLE")
        if bad and start is None:
            start = r["t"]
        elif not bad and start is not None:
            bands.append(
                f'<rect x="{sx(start):.1f}" y="{top1}" width="{sx(r["t"]) - sx(start):.1f}" '
                f'height="{bottom2 - top1}" fill="#d62728" opacity="0.08"/>'
            )
            start = None
    if start is not None:
        bands.append(
            f'<rect x="{sx(start):.1f}" y="{top1}" width="{sx(t1) - sx(start):.1f}" '
            f'height="{bottom2 - top1}" fill="#d62728" opacity="0.08"/>'
        )

    grid: List[str] = []
    for v in (0, 30, 60, 90):
        y = sy1(v)
        grid.append(f'<line x1="{left}" y1="{y:.1f}" x2="{right}" y2="{y:.1f}" stroke="#e3e6ea"/>')
        grid.append(
            f'<text x="{left - 10}" y="{y + 4:.1f}" text-anchor="end" font-size="11" fill="#4a4f55">{v}</text>'
        )
    for v in (60, 80, 100, 120, 140):
        y = sy2(v)
        grid.append(f'<line x1="{left}" y1="{y:.1f}" x2="{right}" y2="{y:.1f}" stroke="#e3e6ea"/>')
        grid.append(
            f'<text x="{left - 10}" y="{y + 4:.1f}" text-anchor="end" font-size="11" fill="#4a4f55">{v}</text>'
        )
    for t in range(0, int(t1) + 1, 4):
        x = sx(t)
        grid.append(f'<line x1="{x:.1f}" y1="{top1}" x2="{x:.1f}" y2="{bottom2}" stroke="#f3f5f7"/>')
        grid.append(
            f'<text x="{x:.1f}" y="{bottom2 + 20}" text-anchor="middle" font-size="11" fill="#4a4f55">{t}</text>'
        )

    legend = [
        '<line x1="520" y1="28" x2="552" y2="28" stroke="#5b6068" stroke-width="2"/>'
        '<text x="558" y="32" font-size="11" fill="#30343a">真实航向 / 航迹向</text>',
        '<line x1="680" y1="28" x2="712" y2="28" stroke="#d62728" stroke-width="2"/>'
        '<text x="718" y="32" font-size="11" fill="#30343a">原始磁航向</text>',
        '<line x1="808" y1="28" x2="840" y2="28" stroke="#1f77b4" stroke-width="2"/>'
        '<text x="846" y="32" font-size="11" fill="#30343a">门控输出</text>',
    ]

    svg = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="{width}" height="{height}" fill="#ffffff"/>
  <text x="{left}" y="26" font-size="14" fill="#1b1f24">磁力计干扰监测 · 离线回放（上：航向 单位 °；下：磁场模值 单位 %）</text>
  {''.join(legend)}
  {''.join(bands)}
  {''.join(grid)}
  <line x1="{left}" y1="{bottom1}" x2="{right}" y2="{bottom1}" stroke="#8a9098"/>
  <line x1="{left}" y1="{top1}" x2="{left}" y2="{bottom1}" stroke="#8a9098"/>
  <line x1="{left}" y1="{bottom2}" x2="{right}" y2="{bottom2}" stroke="#8a9098"/>
  <line x1="{left}" y1="{top2}" x2="{left}" y2="{bottom2}" stroke="#8a9098"/>
  {poly('true_course_deg', '#5b6068', 2, sy1)}
  {poly('mag_heading_deg', '#d62728', 1, sy1)}
  {poly('output_heading_deg', '#1f77b4', 3, sy1)}
  {poly('field_norm_pct', '#2ca02c', 2, sy2)}
  <line x1="{left}" y1="{sy2(100.0):.1f}" x2="{right}" y2="{sy2(100.0):.1f}" stroke="#2ca02c" stroke-dasharray="4 3" opacity="0.6"/>
  <text x="{(left + right) / 2:.0f}" y="{height - 12}" text-anchor="middle" font-size="11" fill="#4a4f55">时间 t (s)</text>
  <text x="22" y="{(top1 + bottom1) / 2:.0f}" transform="rotate(-90 22 {(top1 + bottom1) / 2:.0f})" text-anchor="middle" font-size="11" fill="#4a4f55">航向 (°)</text>
  <text x="22" y="{(top2 + bottom2) / 2:.0f}" transform="rotate(-90 22 {(top2 + bottom2) / 2:.0f})" text-anchor="middle" font-size="11" fill="#4a4f55">模值 (%)</text>
  <text x="{left}" y="{height - 30}" font-size="10" fill="#7a8088">浅红竖带 = DEGRADED / UNRELIABLE（此时建议改用 GNSS 航迹向作为航向来源）</text>
</svg>
"""
    path.write_text(svg, encoding="utf-8")


def print_report(
    preflight: Dict,
    cfg: Config,
    rows: Sequence[Dict],
    events: Sequence[Tuple[float, str, str]],
    metrics: Dict,
) -> None:
    ok: PreflightResult = preflight["ok"]
    bad: PreflightResult = preflight["bad"]
    print("=" * 70)
    print("磁力计干扰检测与航向一致性门控 · 离线回放（合成磁场，非仿真闭环）")
    print("=" * 70)
    print("配置: " + json.dumps(asdict(cfg), ensure_ascii=False, sort_keys=True))
    print("-" * 70)
    print("起飞前电机干扰自检:")
    print(
        f"  无干扰组: {ok.state.value:<5} 偏差偏移 {ok.bias_shift_pct:5.2f}% "
        f"拟合残差 {ok.spin.residual_rms:4.2f} µT  原因 {ok.reason}"
    )
    print(
        f"  注入干扰: {bad.state.value:<5} 偏差偏移 {bad.bias_shift_pct:5.2f}% "
        f"拟合残差 {bad.spin.residual_rms:4.2f} µT  原因 {bad.reason}"
    )
    print("-" * 70)
    print(
        f"飞行段 {metrics['frames']} 帧：DEGRADED {metrics['degraded_frames']} 帧，"
        f"UNRELIABLE {metrics['unreliable_frames']} 帧，"
        f"切换 GNSS 航迹向 {metrics['heading_source_switches']} 次，"
        f"恢复 {metrics['recovery_count']} 次"
    )
    print(
        f"磁航向最大偏差 {metrics['mag_max_error_deg']}°；"
        f"门控输出最大偏差 {metrics['output_max_error_deg']}°"
    )
    print("-" * 70)
    print("状态变化事件 (t / 状态 / 原因):")
    for t, state, reason in events:
        print(f"  t={t:5.2f}  {state:<11} {reason}")
    print("-" * 70)
    print("最后 6 帧 (t / 磁航向 / 输出 / 状态 / 来源 / 原因):")
    for r in rows[-6:]:
        print(
            f"  t={r['t']:5.2f}  {r['mag_heading_deg']:7.2f}  "
            f"{r['output_heading_deg']:7.2f}  {r['state']:<11} "
            f"{r['source']:<12} {r['reason']}"
        )
    print("=" * 70)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="UAV 磁力计干扰监测模块的离线回放入口"
    )
    parser.add_argument("--plot", metavar="DIR", default=None,
                        help="把回放结果写成 CSV 和 SVG 到指定目录")
    parser.add_argument("--config", metavar="JSON", default=None,
                        help='覆盖默认判据，例如 \'{"min_ground_speed": 3.0}\'')
    parser.add_argument("--show-config", action="store_true",
                        help="打印默认配置后退出")
    parser.add_argument("--ros", action="store_true",
                        help="启动 ROS 2 节点（需要 rclpy 与 ROS 2 环境）")
    parser.add_argument("--idle-csv", metavar="FILE", default=None,
                        help="真实日志：电机怠速整圈旋转样本 CSV（t,mx,my,mz）")
    parser.add_argument("--spin-csv", metavar="FILE", default=None,
                        help="真实日志：电机上油门整圈旋转样本 CSV（t,mx,my,mz）")
    parser.add_argument("--export-preflight", metavar="DIR", default=None,
                        help="把回放用的两组自检样本导出为 CSV，便于对照格式")
    args = parser.parse_args(argv)

    if args.ros:
        try:
            from uav_mag_interference_monitor.mag_interference_node import (
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

    if bool(args.idle_csv) != bool(args.spin_csv):
        print("--idle-csv 与 --spin-csv 必须同时提供", file=sys.stderr)
        return 2

    # 用真实日志跑起飞前自检：PASS/WARN 返回 0，FAIL 返回 3，便于接入脚本
    if args.idle_csv and args.spin_csv:
        try:
            idle = load_mag_csv(Path(args.idle_csv))
            spin = load_mag_csv(Path(args.spin_csv))
            res = run_preflight(idle, spin, cfg)
        except (OSError, ValueError) as exc:
            print(f"[错误] {exc}", file=sys.stderr)
            return 2
        print("=" * 70)
        print("起飞前电机干扰自检（真实日志）")
        print("=" * 70)
        print(f"怠速样本 {res.idle.n_samples} 个，水平半径 {res.idle.radius:.2f} µT，"
              f"拟合残差 {res.idle.residual_rms:.2f} µT")
        print(f"油门样本 {res.spin.n_samples} 个，水平半径 {res.spin.radius:.2f} µT，"
              f"拟合残差 {res.spin.residual_rms:.2f} µT")
        print(f"偏差偏移 {res.bias_shift_ut:.2f} µT（占水平分量 "
              f"{res.bias_shift_pct:.2f}%）")
        print(f"结论: {res.state.value}  原因 {res.reason}")
        if res.idle.bias:
            print(f"建议标定值: bias_x={res.idle.bias[0]:.3f} "
                  f"bias_y={res.idle.bias[1]:.3f} "
                  f"expected_radius={res.idle.radius:.3f}")
        print("=" * 70)
        return 0 if res.state.value != "FAIL" else 3

    if args.export_preflight:
        out_dir = Path(args.export_preflight)
        out_dir.mkdir(parents=True, exist_ok=True)
        write_mag_csv(out_dir / "preflight_idle.csv",
                      build_preflight_samples(60, (0.0, 0.0, 0.0), seed=0))
        write_mag_csv(out_dir / "preflight_spin_interfered.csv",
                      build_preflight_samples(60, MOTOR_OFFSET, seed=13))
        print(f"已输出: {out_dir / 'preflight_idle.csv'}")
        print(f"已输出: {out_dir / 'preflight_spin_interfered.csv'}")
        return 0

    preflight, rows, events, metrics = run_replay(cfg)
    print_report(preflight, cfg, rows, events, metrics)

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
