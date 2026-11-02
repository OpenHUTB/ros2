"""垂直车位泊车路径规划模块入口。

* 在装有 ROS2 的环境下运行：``python3 main.py`` 启动规划节点，配合 RViz2 查看结果。
* 没有 ROS2 时自动退化为离线演示：只做规划，输出 SVG 轨迹图与 JSON 路径点，
  便于在没有仿真环境的机器上复现结果。
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from parking_path_planner.planner import (  # noqa: E402
    PlannerConfig,
    SlotParams,
    VehicleParams,
    count_gear_shifts,
    densify,
    goal_pose_from_slot,
    max_curvature,
    plan_parking,
    wrap_angle,
)
from parking_path_planner.viz import render_svg  # noqa: E402


def default_scene():
    """默认场景：车位中心 (0, 2.75)，车辆在车道内朝 +x 行驶。"""
    vehicle = VehicleParams()
    slot = SlotParams()
    start = (-7.0, -4.20, 0.0)
    return vehicle, slot, start


def ros_available():
    try:
        import rclpy  # noqa: F401
        return True
    except ImportError:
        return False


def run_offline(output_dir):
    """离线规划并输出结果文件。"""
    vehicle, slot, start = default_scene()
    goal = goal_pose_from_slot(slot, vehicle)
    result = plan_parking(start, goal, vehicle, slot, PlannerConfig())

    print(f'最小转弯半径 R_min = {vehicle.min_turn_radius:.2f} m')
    print(f'起点 ({start[0]:.2f}, {start[1]:.2f}, '
          f'{math.degrees(start[2]):.1f} deg)')
    if not result.success:
        print(f'规划失败：{result.message}（扩展 {result.iterations} 次）')
        return 1

    dense = densify(result.poses)
    curvature = max_curvature(result.poses)
    dx = goal[0] - result.poses[-1][0]
    dy = goal[1] - result.poses[-1][1]
    dyaw = math.degrees(wrap_angle(goal[2] - result.poses[-1][2]))

    print(f'规划成功：扩展 {result.iterations} 次，路径节点 {len(result.poses)}，'
          f'稠密点 {len(dense)}')
    print(f'  总里程 {result.cost:.2f} m，换挡 {count_gear_shifts(result.poses)} 次')
    print(f'  最大曲率 {curvature:.4f} 1/m（上限 {1.0 / vehicle.min_turn_radius:.4f}）')
    print(f'  终点误差 dx={dx:.3f} m, dy={dy:.3f} m, dyaw={dyaw:.2f} deg')

    os.makedirs(output_dir, exist_ok=True)
    svg_file = os.path.join(output_dir, 'parking_path.svg')
    json_file = os.path.join(output_dir, 'parking_waypoints.json')
    render_svg(result, slot, vehicle, start, svg_file)
    with open(json_file, 'w', encoding='utf-8') as handle:
        json.dump({'frame_id': 'map',
                   'waypoints': [{'x': round(p[0], 3),
                                  'y': round(p[1], 3),
                                  'yaw': round(p[2], 4),
                                  'gear': 'D' if p[3] > 0 else 'R'} for p in dense]},
                  handle, ensure_ascii=False, indent=1)
    print(f'  已输出 {svg_file} 与 {json_file}')
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='垂直车位泊车路径规划（Hybrid A*）')
    parser.add_argument('--offline', action='store_true',
                        help='不启动 ROS 节点，仅离线规划并输出 SVG / JSON')
    parser.add_argument('--output', default='results',
                        help='离线结果的输出目录（默认 results）')
    args = parser.parse_args(argv)

    if not args.offline and ros_available():
        from parking_path_planner.ros_node import main as node_main
        node_main()
        return 0

    if not args.offline:
        print('未检测到 rclpy，转为离线模式。')
    return run_offline(args.output)


if __name__ == '__main__':
    raise SystemExit(main())
