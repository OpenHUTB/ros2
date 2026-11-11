"""膝关节肌肉骨骼仿真与轨迹跟踪（纯 Python 复现）。

用法：
    python main.py                # 完整仿真，生成 PNG + GIF + 指标
    python main.py --no-render    # 不生成 GIF，其余相同
    python main.py --quick        # 缩短时长，用于快速冒烟测试

替代关系：原报告使用 MuJoCo + ROS 2；本实现用纯 Python 的刚体
ODE + 拮抗肌激活动力学，控制与指标口径保持一致。
"""

from __future__ import annotations

import argparse
import dataclasses
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from src.config import AppConfig, ConfigError, load_config
from src.controller import PDController
from src.metrics import sine_metrics, step_metrics
from src.model import KneeModel
from src.plotting import animate_step, plot_sine, plot_step
from src.simulation import SimulationResult, simulate
from src.trajectory import (
    make_time_vector,
    sine_reference,
    step_reference,
)

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = BASE_DIR / "config" / "params.yaml"
RESULTS_DIR = BASE_DIR / "results"


def _configure_stdio() -> None:
    """管道/重定向时用 UTF-8，避免中文日志在采集端变成乱码。"""
    for stream in (sys.stdout, sys.stderr):
        try:
            if not stream.isatty():
                stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="膝关节肌肉骨骼仿真与轨迹跟踪（纯 Python，无 ROS / MuJoCo）"
    )
    parser.add_argument(
        "--no-render",
        action="store_true",
        help="不生成 GIF 动画（仍生成 PNG 与指标文件）",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="缩短正弦/阶跃时长，用于快速验证流程",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"参数文件路径（默认 {DEFAULT_CONFIG}）",
    )
    return parser.parse_args(argv)


def _quick_config(cfg: AppConfig) -> AppConfig:
    """快速模式：只缩短时长，不改物理参数。"""
    return dataclasses.replace(
        cfg,
        simulation=dataclasses.replace(
            cfg.simulation, sine_duration=6.0, step_duration=2.5
        ),
        render=dataclasses.replace(
            cfg.render, gif_max_frames=60, gif_fps=12
        ),
    )


def _assert_finite(name: str, result: SimulationResult) -> None:
    for field_name in (
        "t",
        "theta_deg",
        "omega_rad_s",
        "reference_deg",
        "flex",
        "ext",
        "tau_pd",
        "error_deg",
    ):
        values = getattr(result, field_name)
        if not np.all(np.isfinite(values)):
            raise FloatingPointError(f"{name} 的 {field_name} 出现 NaN/Inf")


def _run_sine(
    cfg: AppConfig, model: KneeModel, controller: PDController
) -> tuple[SimulationResult, dict[str, float]]:
    traj = cfg.trajectory
    t = make_time_vector(cfg.simulation.sine_duration, cfg.simulation.dt)

    def reference_fn(time_s: float) -> float:
        return float(
            sine_reference(
                time_s,
                traj.sine_bias_deg,
                traj.sine_amplitude_deg,
                traj.sine_frequency_hz,
            )
        )

    result = simulate(
        model,
        controller,
        reference_fn,
        t,
        initial_theta_deg=traj.sine_bias_deg,
        dt=cfg.simulation.dt,
    )
    _assert_finite("正弦段", result)
    metrics = sine_metrics(t, result.theta_deg, result.reference_deg)
    return result, metrics


def _run_step(
    cfg: AppConfig, model: KneeModel, controller: PDController
) -> tuple[SimulationResult, dict[str, float | bool | None], float]:
    traj = cfg.trajectory
    total = traj.step_time + cfg.simulation.step_duration
    t = make_time_vector(total, cfg.simulation.dt)

    def reference_fn(time_s: float) -> float:
        return float(
            step_reference(
                time_s,
                traj.step_start_deg,
                traj.step_target_deg,
                traj.step_time,
            )
        )

    result = simulate(
        model,
        controller,
        reference_fn,
        t,
        initial_theta_deg=traj.step_start_deg,
        dt=cfg.simulation.dt,
    )
    _assert_finite("阶跃段", result)

    idx = int(np.searchsorted(result.t, traj.step_time - 1e-12))
    if idx >= result.t.size:
        raise RuntimeError("阶跃时刻超出仿真时间范围")
    t_rel = result.t[idx:] - result.t[idx]
    metrics = step_metrics(
        t_rel,
        result.theta_deg[idx:],
        target=traj.step_target_deg,
        step_amplitude_deg=abs(traj.step_target_deg - traj.step_start_deg),
        band_percent=cfg.metrics.step_band_percent,
        wide_band_deg=cfg.metrics.wide_band_deg,
        steady_window_s=cfg.metrics.steady_state_window_s,
    )
    return result, metrics, traj.step_time


def _write_csv(
    path: Path,
    sine: SimulationResult,
    step: SimulationResult,
    step_time: float,
) -> None:
    lines = [
        "segment,time_s,reference_deg,actual_deg,error_deg,"
        "flex_activation,extensor_activation,tau_pd_nm"
    ]

    def rows(segment: str, result: SimulationResult, offset: float) -> None:
        for i in range(result.t.size):
            relative = result.t[i] - offset
            if relative < -1e-12:
                continue
            lines.append(
                f"{segment},{relative:.4f},"
                f"{result.reference_deg[i]:.6f},{result.theta_deg[i]:.6f},"
                f"{result.error_deg[i]:.6f},{result.flex[i]:.6f},"
                f"{result.ext[i]:.6f},{result.tau_pd[i]:.6f}"
            )

    rows("sine", sine, 0.0)
    rows("step", step, step_time)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_metrics(
    path: Path,
    cfg: AppConfig,
    sine_metrics_dict: dict[str, float],
    step_metrics_dict: dict[str, float | bool | None],
    sine: SimulationResult,
    step: SimulationResult,
    elapsed_s: float,
    quick: bool,
    baseline: tuple | None = None,
) -> None:
    traj = cfg.trajectory
    band = float(step_metrics_dict["settling_band_deg"])
    settle = step_metrics_dict["settling_time_s"]
    settle_text = (
        "未进入 ±2% 带" if settle is None else f"{float(settle):.3f} s"
    )
    wide = step_metrics_dict["wide_band_entry_s"]
    if wide is None:
        peak = float(step_metrics_dict["peak_deg"])
        wide_text = (
            f"未进入 ±5° 带（峰值 {peak:.2f} deg，"
            f"距目标 {abs(traj.step_target_deg - peak):.2f} deg）"
        )
    else:
        wide_text = f"{float(wide):.3f} s"

    text = f"""膝关节肌肉骨骼仿真与轨迹跟踪评估（纯 Python）
==================================================
生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
运行模式: {'quick（缩短时长，仅用于流程验证）' if quick else 'full'}
替代关系: 原报告基于 MuJoCo + ROS 2；本实现用纯 Python 刚体 ODE +
          两条拮抗肌的一阶激活动力学，PD 与指标口径保持一致。

控制器: tau = kp*e - kd*theta_dot - g_ff*tau_g, kp={cfg.controller.kp:g} N·m/rad,
        kd={cfg.controller.kd:g} N·m·s/rad, g_ff={cfg.controller.gravity_feedforward:g}
单肌最大力矩: {cfg.muscle.max_force * cfg.muscle.moment_arm:.2f} N·m
积分: 关节刚体 RK4 + 激活解析指数更新, dt={cfg.simulation.dt:g} s

[正弦段] bias={traj.sine_bias_deg:g} deg, amplitude={traj.sine_amplitude_deg:g} deg,
          {traj.sine_frequency_hz:g} Hz, 时长 {cfg.simulation.sine_duration:g} s
  RMSE: {sine_metrics_dict['rmse_deg']:.4f} deg
  最大绝对误差: {sine_metrics_dict['max_abs_error_deg']:.4f} deg

[阶跃段] {traj.step_start_deg:g} deg -> {traj.step_target_deg:g} deg
  超调: {float(step_metrics_dict['overshoot_percent']):.4f} %（相对阶跃幅值）
  调节时间（±2% 带 = ±{band:.3f} deg）: {settle_text}
  首次进入 ±{cfg.metrics.wide_band_deg:g}° 带: {wide_text}
  稳态误差（末 {cfg.metrics.steady_state_window_s:g} s 平均 - 目标）:
    {float(step_metrics_dict['steady_state_error_deg']):+.4f} deg
  峰值: {float(step_metrics_dict['peak_deg']):.4f} deg
  末段均值: {float(step_metrics_dict['final_deg']):.4f} deg

[安全与数值检查]
  正弦段非法控制指令数: {sine.invalid_count}
  正弦段上限触达采样数: {sine.upper_limit_count}
  正弦段下限触达采样数: {sine.lower_limit_count}
  阶跃段非法控制指令数: {step.invalid_count}
  阶跃段上限触达采样数: {step.upper_limit_count}
  阶跃段下限触达采样数: {step.lower_limit_count}
  NaN/Inf: 无
  实际耗时: {elapsed_s:.2f} s
"""
    if baseline is not None:
        base_sine_m, base_step_m, base_sine, base_step = baseline
        base_settle = base_step_m["settling_time_s"]
        base_settle_text = (
            "未进入" if base_settle is None else f"{float(base_settle):.3f} s"
        )
        text += f"""
[重力前馈对照：同一组 PD 增益]
                         无前馈(g_ff=0)      有前馈(g_ff={cfg.controller.gravity_feedforward:g})
  正弦 RMSE              {base_sine_m['rmse_deg']:14.4f}      {sine_metrics_dict['rmse_deg']:14.4f} deg
  正弦最大误差           {base_sine_m['max_abs_error_deg']:14.4f}      {sine_metrics_dict['max_abs_error_deg']:14.4f} deg
  阶跃稳态误差           {float(base_step_m['steady_state_error_deg']):+14.4f}      {float(step_metrics_dict['steady_state_error_deg']):+14.4f} deg
  阶跃调节时间           {base_settle_text:>14}      {settle_text:>14}
  阶跃末段均值           {float(base_step_m['final_deg']):14.4f}      {float(step_metrics_dict['final_deg']):14.4f} deg

  结论：不补偿重力时 PD 必须靠持续误差撑住重力矩，稳态下垂约
        {abs(float(base_step_m['steady_state_error_deg'])):.2f} deg；打开前馈后降到
        {abs(float(step_metrics_dict['steady_state_error_deg'])):.4f} deg 量级。
        正弦段残差主要来自肌肉激活/失活时间常数（tau_act=0.02 s、
        tau_deact=0.05 s），不是 PD 增益不够。
"""
    path.write_text(text, encoding="utf-8")


def _render_gif(
    cfg: AppConfig,
    step: SimulationResult,
    step_time: float,
    out_path: Path,
) -> Path | None:
    """生成 GIF，并尽量压到 3 MB 以内。"""
    attempts = [
        (cfg.render.gif_fps, cfg.render.gif_dpi, cfg.render.gif_max_frames),
        (max(8, cfg.render.gif_fps // 2), max(60, cfg.render.gif_dpi - 10),
         max(40, cfg.render.gif_max_frames // 2)),
        (6, 60, 40),
    ]
    for fps, dpi, frames in attempts:
        animate_step(
            step,
            step_time,
            out_path,
            fps=fps,
            dpi=dpi,
            max_frames=frames,
        )
        size_mb = out_path.stat().st_size / (1024 * 1024)
        if size_mb <= 3.0:
            print(f"[render] GIF: {out_path} ({size_mb:.2f} MB, {fps} fps)")
            return out_path
        print(f"[render] GIF {size_mb:.2f} MB 超过 3 MB，降低画质重试...")
    out_path.unlink(missing_ok=True)
    print("[render] 警告：无法把 GIF 压到 3 MB 以内，已跳过 GIF（PNG 不受影响）")
    return None


def main(argv: list[str] | None = None) -> int:
    _configure_stdio()
    args = parse_args(argv)
    started = time.perf_counter()
    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        print(f"[config] 错误：{exc}", file=sys.stderr)
        return 2
    if args.quick:
        cfg = _quick_config(cfg)

    model = KneeModel(cfg.joint, cfg.muscle)

    def make_controller(gravity_feedforward: float) -> PDController:
        controller_cfg = dataclasses.replace(
            cfg.controller, gravity_feedforward=gravity_feedforward
        )
        return PDController(
            controller_cfg,
            max_torque=model.max_muscle_torque,
            angle_min_deg=cfg.joint.angle_min_deg,
            angle_max_deg=cfg.joint.angle_max_deg,
            gravity_torque_fn=model.gravity_torque,
        )

    controller = make_controller(cfg.controller.gravity_feedforward)

    print("=" * 62)
    print("膝关节肌肉骨骼仿真与轨迹跟踪（纯 Python，无 ROS / MuJoCo）")
    print("=" * 62)
    print(
        f"参数: kp={cfg.controller.kp:g}, kd={cfg.controller.kd:g}, "
        f"I={cfg.joint.inertia:g}, b={cfg.joint.damping:g}, "
        f"max_torque={model.max_muscle_torque:.2f} N·m"
    )

    RESULT_DIR = RESULTS_DIR
    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    sine_result, sine_m = _run_sine(cfg, model, controller)
    step_result, step_m, step_time = _run_step(cfg, model, controller)

    # 对照：同一组增益、关闭重力前馈，量化“稳态下垂”有多大。
    baseline = make_controller(0.0)
    base_sine_result, base_sine_m = _run_sine(cfg, model, baseline)
    base_step_result, base_step_m, base_step_time = _run_step(cfg, model, baseline)

    plot_sine(sine_result, sine_m, RESULT_DIR / "tracking_eval.png")
    plot_step(
        step_result,
        step_m,
        target_deg=cfg.trajectory.step_target_deg,
        step_time=step_time,
        path=RESULT_DIR / "tracking_step.png",
        band_wide_deg=cfg.metrics.wide_band_deg,
    )
    _write_csv(RESULT_DIR / "tracking.csv", sine_result, step_result, step_time)

    elapsed = time.perf_counter() - started
    _write_metrics(
        RESULT_DIR / "metrics.txt",
        cfg,
        sine_m,
        step_m,
        sine_result,
        step_result,
        elapsed,
        args.quick,
        baseline=(base_sine_m, base_step_m, base_sine_result, base_step_result),
    )

    if args.no_render or not cfg.render.gif_enabled:
        print("[render] 已跳过 GIF（--no-render 或 render.gif_enabled=false）")
    else:
        _render_gif(cfg, step_result, step_time, RESULT_DIR / "tracking.gif")

    print("\n[正弦段]")
    print(f"  RMSE            = {sine_m['rmse_deg']:.2f} deg")
    print(f"  最大绝对误差    = {sine_m['max_abs_error_deg']:.2f} deg")
    print("\n[阶跃段]")
    print(f"  超调            = {float(step_m['overshoot_percent']):.2f} %")
    settle = step_m["settling_time_s"]
    print(
        "  调节时间        = "
        + ("未进入 ±2% 带" if settle is None else f"{float(settle):.2f} s")
    )
    wide = step_m["wide_band_entry_s"]
    print(
        "  进入 ±5° 带     = "
        + ("未进入" if wide is None else f"{float(wide):.2f} s")
    )
    print(f"  峰值            = {float(step_m['peak_deg']):.2f} deg")
    print(
        f"  稳态误差        = {float(step_m['steady_state_error_deg']):+.2f} deg"
    )
    print(f"\n结果目录: {RESULT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
