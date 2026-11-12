#!/usr/bin/env python3
"""运行 MuJoCo 差速小车电机热仿真。"""

from __future__ import annotations

import argparse
import json

from mujoco_motor_thermal_control.simulation import run_constant_command


def main() -> None:
    parser = argparse.ArgumentParser(description="运行差速小车驱动电机热仿真")
    parser.add_argument("--steps", type=int, default=12000, help="仿真步数；默认对应 120 秒")
    parser.add_argument("--linear", type=float, default=0.55, help="目标线速度（米/秒）")
    parser.add_argument("--angular", type=float, default=0.0, help="目标角速度（弧度/秒）")
    parser.add_argument("--ambient", type=float, default=25.0, help="环境温度（摄氏度）")
    parser.add_argument("--render", help="将本次仿真终点的真实 MuJoCo 帧保存为 PNG")
    args = parser.parse_args()
    states = run_constant_command(args.steps, args.linear, args.angular, args.ambient, render_path=args.render or None)
    final = states[-1]
    print(json.dumps({
        "steps": len(states),
        "duration_s": round(final.time_s, 3),
        "x_m": round(final.x_m, 3),
        "y_m": round(final.y_m, 3),
        "yaw_rad": round(final.yaw_rad, 4),
        "pitch_rad": round(final.pitch_rad, 4),
        "max_abs_pitch_rad": round(max(abs(state.pitch_rad) for state in states), 6),
        "speed_mps": round(final.speed_mps, 3),
        "left_temperature_c": round(final.left_temperature_c, 3),
        "right_temperature_c": round(final.right_temperature_c, 3),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
