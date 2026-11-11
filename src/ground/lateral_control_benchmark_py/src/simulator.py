"""闭环仿真主循环。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .controllers import BaseController
from .metrics import evaluate
from .path import Scenario, make_scenarios
from .vehicle import (
    SteeringActuator,
    VehicleParams,
    initial_dynamic_state,
    longitudinal_target,
    rk4_step,
    rk4_step_dynamic,
)


@dataclass
class RunResult:
    scenario: str
    scenario_title: str
    controller: str
    label: str
    model: str
    time: np.ndarray
    x: np.ndarray
    y: np.ndarray
    psi: np.ndarray
    v: np.ndarray
    e: np.ndarray
    delta: np.ndarray
    s: np.ndarray
    metrics: dict


def simulate(scenario: Scenario, controller: BaseController, model: str = "kinematic",
             cruise_speed: float = 8.0, dt: float = 0.02,
             params: VehicleParams | None = None) -> RunResult:
    """闭环仿真一个场景 + 一个控制器。"""
    if model not in {"kinematic", "dynamic"}:
        raise ValueError("model must be 'kinematic' or 'dynamic'")
    if params is None:
        params = controller.params
    if dt <= 0.0:
        raise ValueError("dt must be positive")

    controller.reset()
    actuator = SteeringActuator(params)
    state = scenario.initial_state()
    if model == "dynamic":
        state = initial_dynamic_state(state)

    projection = scenario.path.project(float(state[0]), float(state[1]), s_hint=scenario.s0)
    steps = int(round(scenario.duration / dt))
    count = steps + 1

    time = np.zeros(count)
    xs = np.zeros(count)
    ys = np.zeros(count)
    psis = np.zeros(count)
    speeds = np.zeros(count)
    errors = np.zeros(count)
    deltas = np.zeros(count)
    progress = np.zeros(count)

    def set_record(index, current_state, current_projection, current_delta):
        xs[index] = current_state[0]
        ys[index] = current_state[1]
        psis[index] = current_state[2]
        speeds[index] = current_state[3]
        errors[index] = current_projection.e
        deltas[index] = current_delta
        progress[index] = current_projection.s

    set_record(0, state, projection, 0.0)
    controller.applied_delta = 0.0

    for k in range(steps):
        pose = state[:4]
        v_target = longitudinal_target(projection.s, scenario.path, cruise_speed, params)
        desired = controller.compute(pose, projection, scenario.path, dt)
        applied = actuator.command(desired, dt)
        controller.applied_delta = applied

        if model == "kinematic":
            state = rk4_step(state, applied, v_target, dt, params)
        else:
            state = rk4_step_dynamic(state, applied, v_target, dt, params)

        if not np.all(np.isfinite(state)):
            raise FloatingPointError(
                f"non-finite vehicle state in {scenario.key}/{controller.name}"
            )
        projection = scenario.path.project(float(state[0]), float(state[1]),
                                           s_hint=projection.s)
        time[k + 1] = time[k] + dt
        set_record(k + 1, state, projection, applied)
        if model == "kinematic" and abs(state[3]) < 1e-4:
            # 速度不会真的降到零；这里只防御无意义的值。
            pass

    metrics = evaluate(
        time, errors, deltas, dt,
        offroad_threshold=scenario.offroad_threshold,
    )
    if not all(np.isfinite(list(metrics.values()))):
        raise FloatingPointError("metrics contain non-finite values")

    return RunResult(
        scenario=scenario.key,
        scenario_title=scenario.title,
        controller=controller.name,
        label=controller.label,
        model=model,
        time=time,
        x=xs,
        y=ys,
        psi=psis,
        v=speeds,
        e=errors,
        delta=deltas,
        s=progress,
        metrics=metrics,
    )


def run_benchmark(controllers, scenarios=None, model: str = "kinematic",
                  cruise_speed: float = 8.0, dt: float = 0.02):
    """运行 4 场景 x 3 控制器，返回按场景组织的二维列表。"""
    if scenarios is None:
        scenarios = make_scenarios()
    results = []
    for scenario in scenarios.values():
        row = []
        for controller in controllers:
            row.append(simulate(scenario, controller, model=model,
                                cruise_speed=cruise_speed, dt=dt))
        results.append(row)
    return results


def results_to_records(results):
    """flatten RunResult 为指标字典列表。"""
    records = []
    for row in results:
        for result in row:
            record = {
                "scenario": result.scenario,
                "scenario_title": result.scenario_title,
                "controller": result.controller,
                "label": result.label,
                "model": result.model,
            }
            record.update(result.metrics)
            records.append(record)
    return records
