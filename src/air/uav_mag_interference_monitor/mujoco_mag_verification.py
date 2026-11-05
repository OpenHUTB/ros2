#!/usr/bin/env python3
"""MuJoCo 物理仿真验证：自检圆拟合与飞行中门控在物理仿真数据上的行为。

    python mujoco_mag_verification.py              # 跑全部场景并打印报告
    python mujoco_mag_verification.py --plot DIR   # 额外输出 CSV 与 SVG 证据

需要 `pip install mujoco`（本模块唯一引入第三方依赖的场景，其余入口纯标准库）。

与离线回放（main.py）的分工：
  * main.py 的磁场数据是解析合成的（匀速整圈、匀速平飞）；
  * 本脚本的旋转与平飞由 MuJoCo 物理引擎驱动——伺服起转的角加速度、
    非均匀角速度、速度伺服的稳态误差都会体现在采样里；
  * 水平地磁场读数来自 MuJoCo 的 magnetometer 传感器（site 姿态旋转由
    引擎完成，参考磁场为世界系 (0,-0.5,0)，×44 缩放到 22 µT）；
  * MuJoCo 不模拟电磁场，机体固定磁场按文档化解析模型叠加：硬铁偏差
    (4.0, -2.5, 1.2) µT、油门比例电机磁场 (5.6, -3.2, 1.6) µT、垂直地磁
    -42 µT。这是磁力计标准误差模型项，任何仿真器都只能这样注入。

坐标系映射（与 MuJoCo 磁力计参考场指向保持一致）：
  东 = -x_w，北 = -y_w；航迹向 course = atan2(Δeast, Δnorth)。
  机头对准航迹时机体 yaw q = 225°，此时磁航向 = 航迹向 = 45°（已数值验证）。

三个场景：
  1) 起飞前自检 · 无干扰：怠速整圈 × 2 → 圆拟合偏差一致 → PASS（不误报）；
  2) 起飞前自检 · 电机干扰：怠速整圈 vs 上油门整圈 → 偏差偏移 ≈30% → FAIL；
  3) 飞行中门控：MuJoCo 平飞 + 与离线回放相同的干扰剖面（横向推偏航向 /
     沿场抬高模值 / 1.3 s 磁力计丢帧），统计四态状态机事件与输出误差。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

from uav_mag_interference_monitor import (  # noqa: E402
    Config,
    FlightSample,
    HeadingMonitor,
    MagSample,
    fit_circle,
    run_preflight,
)
from main import interference_profile, write_csv, write_svg  # noqa: E402

try:
    import mujoco
except ImportError:  # pragma: no cover
    print(
        "[错误] 未安装 mujoco，本脚本需要 `pip install mujoco`。\n"
        "       模块其余入口（单元测试 / 冒烟测试 / 离线回放）均为纯标准库。",
        file=sys.stderr,
    )
    raise SystemExit(2)

MAG_SCALE = 44.0        # MuJoCo 参考磁场 (0,-0.5,0) → 水平分量 22 µT
HARD_IRON = (4.0, -2.5, 1.2)      # 机体固定硬铁偏差 (µT)
MOTOR_FIELD = (5.6, -3.2, 1.6)    # 油门=1 时的电机电流磁场 (µT，机体系)
EARTH_V = -42.0                   # 垂直地磁 (µT，解析叠加，见模块 docstring)
COURSE_DEG = 45.0                 # 航迹向（东北）
SPEED = 4.5                       # 目标地速 (m/s)
SPIN_RATE = 0.5                   # 自检旋转角速度 (rad/s)
FLIGHT_END = 34.0
DROPOUT = (22.3, 23.6)            # 磁力计丢帧区间（与离线回放一致）

MODEL_XML = """
<mujoco>
  <option timestep="0.002" integrator="RK4"/>
  <worldbody>
    <body name="drone">
      <joint name="px" type="slide" axis="1 0 0" damping="0.5"/>
      <joint name="py" type="slide" axis="0 1 0" damping="0.5"/>
      <joint name="yaw" type="hinge" axis="0 0 1" damping="0.2"/>
      <geom type="sphere" size="0.2" mass="1.5"/>
      <site name="imu" pos="0 0 0" size="0.01"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="vx" joint="px" kv="10"/>
    <velocity name="vy" joint="py" kv="10"/>
    <velocity name="wz" joint="yaw" kv="5"/>
  </actuator>
  <sensor>
    <magnetometer name="mag" site="imu"/>
  </sensor>
</mujoco>
"""


def _make_sim():
    model = mujoco.MjModel.from_xml_string(MODEL_XML)
    data = mujoco.MjData(model)
    return model, data


def _module_mag(sensor_xy, throttle: float, extra=(0.0, 0.0)) -> Tuple[float, float]:
    """MuJoCo 磁力计读数 → 模块约定 (µT)：传感器 × 缩放 + 硬铁 + 电机 + 注入。"""

    mx = sensor_xy[0] * MAG_SCALE + HARD_IRON[0] + MOTOR_FIELD[0] * throttle + extra[0]
    my = sensor_xy[1] * MAG_SCALE + HARD_IRON[1] + MOTOR_FIELD[1] * throttle + extra[1]
    return mx, my


def _mz(throttle: float) -> float:
    """垂直分量：传感器参考场无垂直分量，按解析模型给出。"""

    return EARTH_V + HARD_IRON[2] + MOTOR_FIELD[2] * throttle


# ---------------------------------------------------------------------------
# 场景 1/2：起飞前自检（整圈旋转，MuJoCo 伺服驱动）
# ---------------------------------------------------------------------------

def spin_circle(throttle: float, sample_hz: float = 25.0) -> List[MagSample]:
    """速度伺服驱动机体绕竖轴旋转一周以上，按 sample_hz 采样磁力计。"""

    model, data = _make_sim()
    data.ctrl[2] = SPIN_RATE
    samples: List[MagSample] = []
    dt_sample = 1.0 / sample_hz
    next_t = 0.2                     # 跳过起转瞬态
    yaw0 = 0.0
    target = 2.0 * math.pi * 1.08    # 采样到 ~2.16π 覆盖即停
    while True:
        mujoco.mj_step(model, data)
        if data.time >= next_t:
            mx, my = _module_mag(data.sensordata[:2], throttle)
            samples.append(MagSample(round(data.time, 4), mx, my, _mz(throttle)))
            next_t += dt_sample
        if abs(data.qpos[2] - yaw0) >= target or data.time > 60.0:
            break
    return samples


def run_preflight_scenarios(cfg: Config) -> dict:
    """两组自检：无干扰（应 PASS）与电机干扰（应 FAIL）。"""

    idle = spin_circle(throttle=0.0)
    clean = spin_circle(throttle=0.0)
    noisy = spin_circle(throttle=1.0)

    ok = run_preflight(idle, clean, cfg)
    bad = run_preflight(idle, noisy, cfg)
    return {
        "idle_n": len(idle),
        "ok": ok,
        "bad": bad,
        "cal": fit_circle(idle),
    }


# ---------------------------------------------------------------------------
# 场景 3：飞行中门控（MuJoCo 平飞 + 干扰剖面）
# ---------------------------------------------------------------------------

def run_flight(cfg: Config, cal_bias, cal_radius: float) -> Tuple[List[dict], List[Tuple[float, str, str]]]:
    """MuJoCo 平飞 34 s：磁力计与位置取自仿真，干扰剖面与离线回放一致。"""

    model, data = _make_sim()
    yaw0 = math.radians(135.0 + 90.0)   # 机头对准航迹（东=-x_w、北=-y_w）
    data.qpos[2] = yaw0
    v = SPEED / math.sqrt(2.0)
    data.ctrl[:] = [-v, -v, 0.0]        # 世界系 -x/-y 方向平飞，yaw 保持

    # 干扰方向（机体系，相对干净场）：横向 = 左法向，沿场 = 场方向
    fx, fy = 22.0 * -math.sin(yaw0), 22.0 * -math.cos(yaw0)
    cross_unit = (-fy / 22.0, fx / 22.0)
    along_unit = (fx / 22.0, fy / 22.0)

    monitor = HeadingMonitor(cal_bias, cfg, expected_radius=cal_radius)

    rows: List[dict] = []
    events: List[Tuple[float, str, str]] = []
    prev_state: Optional[str] = None
    last_pos: Optional[Tuple[float, float]] = None
    course = None
    speed = 0.0
    dt_frame = 0.1
    next_t = 0.0
    while data.time <= FLIGHT_END + 1e-9:
        mujoco.mj_step(model, data)
        if data.time + 1e-9 < next_t:
            continue
        t = round(data.time, 4)
        # GNSS：世界位置 → 东/北位移 → 航迹向与地速
        x, y = data.qpos[0], data.qpos[1]
        if last_pos is not None:
            east, north = -(x - last_pos[0]), -(y - last_pos[1])
            dist = math.hypot(east, north)
            if dist > 1e-6:
                speed = dist / dt_frame
                course = math.atan2(east, north) if speed >= cfg.min_ground_speed else None
        last_pos = (x, y)

        if DROPOUT[0] <= t < DROPOUT[1]:
            next_t = round(next_t + dt_frame, 4)   # 磁力计丢帧：GNSS 继续
            continue

        cross, along = interference_profile(t)
        mx, my = _module_mag(
            data.sensordata[:2],
            throttle=0.0,
            extra=(cross * cross_unit[0] + along * along_unit[0],
                   cross * cross_unit[1] + along * along_unit[1]),
        )
        sample = FlightSample(t, mx, my, _mz(0.0), speed=speed, course=course)
        out = monitor.update(sample)
        rows.append(
            {
                "t": t,
                "true_course_deg": COURSE_DEG,
                "mag_heading_deg": math.degrees(math.atan2(my - cal_bias[1], mx - cal_bias[0])),
                "output_heading_deg": math.degrees(out.heading),
                "state": out.state.value,
                "source": out.heading_source,
                "valid": out.valid,
                "heading_error_deg": out.heading_error_deg,
                "field_norm_pct": out.field_norm_pct,
                "interference_ut": cross + along,
                "reason": out.reason,
            }
        )
        if out.state.value != prev_state:
            events.append((t, out.state.value, out.reason))
            prev_state = out.state.value
        next_t = round(next_t + dt_frame, 4)
    return rows, events


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------

def print_report(pre: dict, rows: List[dict], events, cfg: Config) -> None:
    ok, bad, cal = pre["ok"], pre["bad"], pre["cal"]
    print("=" * 70)
    print("磁力计干扰检测 · MuJoCo 物理仿真验证")
    print(f"mujoco 版本: {mujoco.__version__}   配置: "
          f"min_ground_speed={cfg.min_ground_speed} "
          f"norm_tolerance_pct={cfg.norm_tolerance_pct}")
    print("-" * 70)
    print(f"自检采样：怠速整圈 {pre['idle_n']} 帧（MuJoCo 速度伺服，非均匀角速度）")
    print(
        f"  无干扰组: {ok.state.value:<5} 偏差偏移 {ok.bias_shift_pct:5.2f}% "
        f"拟合残差 {ok.spin.residual_rms:5.3f} µT  原因 {ok.reason}"
    )
    print(
        f"  注入干扰: {bad.state.value:<5} 偏差偏移 {bad.bias_shift_pct:5.2f}% "
        f"拟合残差 {bad.spin.residual_rms:5.3f} µT  原因 {bad.reason}"
    )
    print(
        f"  圆拟合硬铁估计: ({cal.bias[0]:.2f}, {cal.bias[1]:.2f}) µT "
        f"（注入真值 {HARD_IRON[0]:.1f}, {HARD_IRON[1]:.1f}）"
        f" 半径 {cal.radius:.2f} µT（真值 22.00）"
    )
    print("-" * 70)
    degraded = sum(1 for r in rows if r["state"] == "DEGRADED")
    unreliable = sum(1 for r in rows if r["state"] == "UNRELIABLE")
    switched = sum(
        1 for j in range(1, len(rows))
        if rows[j]["source"] == "gnss_course" and rows[j - 1]["source"] != "gnss_course"
    )
    recovered = sum(
        1 for j in range(1, len(events))
        if events[j - 1][1] in ("DEGRADED", "UNRELIABLE") and events[j][1] == "TRUSTED"
    )
    out_err = [
        abs(((r["output_heading_deg"] - r["true_course_deg"] + 180.0) % 360.0) - 180.0)
        for r in rows
    ]
    mag_err = [
        abs(((r["mag_heading_deg"] - r["true_course_deg"] + 180.0) % 360.0) - 180.0)
        for r in rows
    ]
    print(
        f"飞行段 {len(rows)} 帧（MuJoCo 平飞）：DEGRADED {degraded} 帧，"
        f"UNRELIABLE {unreliable} 帧，切换 GNSS 航迹向 {switched} 次，恢复 {recovered} 次"
    )
    print(f"磁航向最大偏差 {max(mag_err):.2f}°；门控输出最大偏差 {max(out_err):.2f}°")
    print("-" * 70)
    print("状态变化事件 (t / 状态 / 原因):")
    for t, state, reason in events:
        print(f"  t={t:5.2f}  {state:<11} {reason}")
    print("=" * 70)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="MuJoCo 磁力计仿真验证")
    parser.add_argument("--plot", metavar="DIR", default=None,
                        help="把飞行段结果写成 CSV 和 SVG 到指定目录")
    args = parser.parse_args(argv)

    cfg = Config()
    pre = run_preflight_scenarios(cfg)
    rows, events = run_flight(cfg, pre["cal"].bias, pre["cal"].radius)
    print_report(pre, rows, events, cfg)

    if pre["ok"].state.value != "PASS" or pre["bad"].state.value != "FAIL":
        print("[失败] 自检场景结论与预期不符（无干扰应 PASS / 干扰应 FAIL）",
              file=sys.stderr)
        return 1
    if not events:
        print("[失败] 飞行段没有任何状态变化事件", file=sys.stderr)
        return 1

    if args.plot:
        out_dir = Path(args.plot)
        out_dir.mkdir(parents=True, exist_ok=True)
        csv_path = out_dir / "mujoco_replay.csv"
        svg_path = out_dir / "mujoco_replay.svg"
        write_csv(csv_path, rows)
        write_svg(svg_path, rows)
        print(f"已输出: {csv_path}")
        print(f"已输出: {svg_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
