#!/usr/bin/env python3
"""运行肘关节阶跃仿真，导出与仿真时长一致的轨迹和肌腱力动图。"""

import argparse
from bisect import bisect_left
from math import ceil, isfinite
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))

from mujoco_musculoskeletal_elbow_nn_control.controller import (  # noqa: E402
    AntagonisticPDController,
    ControlState,
)
from mujoco_musculoskeletal_elbow_nn_control.simulation import (  # noqa: E402
    ElbowSimulation,
)
from mujoco_musculoskeletal_elbow_nn_control.trajectories import (  # noqa: E402
    TrajectorySpec,
)


DURATION_S = 4.0
FPS = 50
FRAME_COUNT = int(DURATION_S * FPS)
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT
    / "docs/img/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control"
    / "elbow_trajectory.gif"
)


def collect_rows():
    """保留完整仿真采样，按有限步数运行并在退出前清零激活量。"""
    simulation = ElbowSimulation(PACKAGE_ROOT / "models/elbow.xml")
    controller = AntagonisticPDController()
    trajectory = TrajectorySpec(kind="step", duration_s=DURATION_S)
    rows = []
    try:
        timestep = simulation.timestep_s
        if not isfinite(timestep) or timestep <= 0.0:
            raise ValueError("模型步长必须是有限正数")
        steps = ceil(DURATION_S / timestep)
        if steps > 100_000:
            raise ValueError("模型步长过小，超出录制步数上限")
        trajectory.validate_joint_range(*simulation.joint_range_rad)
        observation = simulation.reset(trajectory.initial_angle_rad)
        for step in range(steps + 1):
            target = trajectory.sample(observation.time_s)
            row = (
                observation.time_s,
                target,
                observation.angle_rad,
                observation.flexor_force_n,
                observation.extensor_force_n,
            )
            if not all(isfinite(value) for value in row):
                raise ValueError("仿真产生非有限数值，停止录制")
            rows.append(row)
            if step == steps:
                break
            command = controller.command(
                ControlState(
                    target_angle=target,
                    angle=observation.angle_rad,
                    angular_velocity=observation.angular_velocity_rad_s,
                    flexor_force=observation.flexor_force_n,
                    extensor_force=observation.extensor_force_n,
                )
            )
            observation = simulation.step(command)
        if rows[-1][0] < DURATION_S - 1e-9:
            raise RuntimeError("仿真未达到指定录制时长")
    finally:
        simulation.safe_stop()
    return rows


def record_gif(output: Path) -> None:
    """每隔 20 毫秒抽取一帧，将 4 秒仿真保存为 4 秒播放的动图。"""
    rows = collect_rows()
    times, targets, angles, flexor, extensor = zip(*rows)
    # 首帧对应 0.02 秒，末帧对应 4.00 秒，共 200 帧。
    frame_indices = [
        min(len(times) - 1, bisect_left(times, (frame + 1) / FPS - 1e-9))
        for frame in range(FRAME_COUNT)
    ]
    force_min = min(0.0, *flexor, *extensor)
    force_max = max(0.0, *flexor, *extensor)
    force_padding = max(1.0, 0.08 * (force_max - force_min))

    figure, (angle_axes, force_axes) = plt.subplots(
        2, 1, figsize=(7, 5), dpi=100, constrained_layout=True
    )
    try:
        angle_axes.set_ylabel("angle (rad)")
        angle_axes.set_ylim(0.1, 1.35)
        angle_axes.set_xlim(0, DURATION_S)
        angle_axes.grid(alpha=0.25)
        force_axes.set_xlabel("time (s)")
        force_axes.set_ylabel("tendon force (N)")
        force_axes.set_ylim(force_min - force_padding, force_max + force_padding)
        force_axes.set_xlim(0, DURATION_S)
        force_axes.grid(alpha=0.25)
        target_line, = angle_axes.plot([], [], "--", label="target")
        angle_line, = angle_axes.plot([], [], label="elbow")
        flexor_line, = force_axes.plot([], [], label="flexor")
        extensor_line, = force_axes.plot([], [], label="extensor")
        angle_axes.legend(loc="upper left")
        force_axes.legend(loc="upper left")

        def update(index):
            end = index + 1
            target_line.set_data(times[:end], targets[:end])
            angle_line.set_data(times[:end], angles[:end])
            flexor_line.set_data(times[:end], flexor[:end])
            extensor_line.set_data(times[:end], extensor[:end])
            return target_line, angle_line, flexor_line, extensor_line

        animation = FuncAnimation(
            figure,
            update,
            frames=frame_indices,
            interval=1000 / FPS,
            blit=True,
            repeat=False,
        )
        output = Path(output).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        animation.save(output, writer=PillowWriter(fps=FPS))
    finally:
        plt.close(figure)
    print(f"已保存 {output}")
    print(
        f"仿真 {DURATION_S:.2f} 秒，{len(rows)} 条采样；"
        f"动图 {FRAME_COUNT} 帧，每帧 {1000 // FPS} 毫秒，播放 4.00 秒"
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="录制肘关节轨迹与肌腱力动图")
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="输出 GIF 路径，默认保存到文档的肘关节模块素材目录",
    )
    arguments = parser.parse_args(argv)
    record_gif(arguments.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
