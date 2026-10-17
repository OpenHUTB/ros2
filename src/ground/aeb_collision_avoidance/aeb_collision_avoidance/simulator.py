"""Standalone 2D vehicle and obstacle simulator for the AEB demo."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .aeb_core import AEBController, SafetyState


@dataclass
class SimulationResult:
    times: list[float]
    distances: list[float]
    speeds: list[float]
    ttcs: list[float]
    states: list[SafetyState]
    brakes: list[float]
    collision: bool
    final_distance: float
    final_speed: float
    stopped: bool


def run_simulation(
    duration: float = 8.0,
    dt: float = 0.05,
    initial_distance: float = 32.0,
    initial_speed: float = 8.0,
    controller: AEBController | None = None,
    on_step: Callable[[float, float, float, SafetyState], None] | None = None,
) -> SimulationResult:
    """Integrate a simple longitudinal model under AEB control."""
    controller = controller or AEBController()
    times: list[float] = []
    distances: list[float] = []
    speeds: list[float] = []
    ttcs: list[float] = []
    states: list[SafetyState] = []
    brakes: list[float] = []

    distance = initial_distance
    speed = initial_speed
    steps = int(duration / dt)
    for index in range(steps + 1):
        time_s = index * dt
        decision = controller.decide(distance, speed)
        times.append(time_s)
        distances.append(distance)
        speeds.append(speed)
        ttcs.append(decision.ttc)
        states.append(decision.state)
        brakes.append(decision.brake)
        if on_step:
            on_step(time_s, distance, speed, decision.state)

        new_speed = max(0.0, speed - decision.brake * dt)
        distance = max(0.0, distance - speed * dt)
        speed = new_speed
        if speed <= 0.1:
            break

    return SimulationResult(
        times=times,
        distances=distances,
        speeds=speeds,
        ttcs=ttcs,
        states=states,
        brakes=brakes,
        collision=distance <= 0.0 and speed > 0.1,
        final_distance=distance,
        final_speed=speed,
        stopped=speed <= 0.1,
    )


def save_report(result: SimulationResult, output_dir: str | Path) -> tuple[Path, Path]:
    """Save a plot and CSV report without requiring ROS or CARLA."""
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    plot_path = output / "aeb_simulation.png"
    report_path = output / "aeb_simulation.csv"

    import matplotlib.pyplot as plt

    fig, (ax_distance, ax_speed) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax_distance.plot(result.times, result.distances, color="#2f80ed", linewidth=2, label="obstacle distance (m)")
    ax_distance.axhline(1.0, color="#eb5757", linestyle="--", label="stop distance")
    ax_distance.set_ylabel("Distance (m)")
    ax_distance.grid(alpha=0.25)
    ax_distance.legend(loc="upper right")

    ax_speed.plot(result.times, result.speeds, color="#27ae60", linewidth=2, label="ego speed (m/s)")
    ax_speed.fill_between(result.times, result.brakes, color="#f2994a", alpha=0.25, label="brake command")
    ax_speed.set_xlabel("Time (s)")
    ax_speed.set_ylabel("Speed / brake")
    ax_speed.grid(alpha=0.25)
    ax_speed.legend(loc="upper right")

    fig.suptitle("AEB collision avoidance: TTC-triggered emergency braking")
    fig.tight_layout()
    fig.savefig(plot_path, dpi=140)
    plt.close(fig)

    with report_path.open("w", encoding="utf-8") as stream:
        stream.write("time,distance,speed,ttc,state,brake\n")
        for row in zip(result.times, result.distances, result.speeds, result.ttcs, result.states, result.brakes):
            stream.write(
                ",".join(
                    [
                        f"{row[0]:.2f}",
                        f"{row[1]:.3f}",
                        f"{row[2]:.3f}",
                        f"{row[3]:.3f}",
                        row[4].value,
                        f"{row[5]:.3f}",
                    ]
                )
                + "\n"
            )
    return plot_path, report_path
