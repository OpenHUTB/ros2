#!/usr/bin/env python3
"""运行 MuJoCo 差速小车电机热仿真。"""

from __future__ import annotations

import argparse
import json

from mujoco_motor_thermal_control.simulation import run_constant_command


def main() -> None:
    parser = argparse.ArgumentParser(description="运行差速小车驱动电机热仿真")
    parser.add_argument("--steps", type=int, default=600)
    parser.add_argument("--linear", type=float, default=0.55, help="目标线速度（米/秒）")
    parser.add_argument("--angular", type=float, default=0.0, help="目标角速度（弧度/秒）")
    args = parser.parse_args()
    states = run_constant_command(args.steps, args.linear, args.angular)
    final = states[-1]
    print(json.dumps({
        "steps": len(states),
        "duration_s": round(final.time_s, 3),
        "x_m": round(final.x_m, 3),
        "y_m": round(final.y_m, 3),
        "speed_mps": round(final.speed_mps, 3),
        "left_temperature_c": round(final.left_temperature_c, 3),
        "right_temperature_c": round(final.right_temperature_c, 3),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
