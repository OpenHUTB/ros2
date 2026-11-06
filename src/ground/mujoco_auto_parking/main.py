#!/usr/bin/env python3
"""课程主入口：MuJoCo 无人车自动泊车。

用法：
  ./main.sh --launch      # 启动 ROS2 仿真与键盘节点
  ./main.sh --test        # 任务①仿真/键盘测试
  ./main.sh --check-model # 检查 MuJoCo 模型对象
"""

import argparse
import os
import subprocess
import sys

PACKAGE_ROOT = os.path.dirname(os.path.abspath(__file__))
PACKAGE_NAME = 'mujoco_auto_parking'


def _run_script(script_path: str, description: str) -> None:
    if not os.path.isfile(script_path):
        print(f'[ERROR] 找不到{description}: {script_path}', file=sys.stderr)
        sys.exit(1)
    print(f'[INFO] 运行{description}: {os.path.basename(script_path)}')
    result = subprocess.run([sys.executable, script_path])
    sys.exit(result.returncode)


def _run_launch(launch_file: str, description: str) -> None:
    launch_path = os.path.join(PACKAGE_ROOT, 'launch', launch_file)
    if not os.path.isfile(launch_path):
        print(f'[ERROR] 找不到 launch 文件: {launch_path}', file=sys.stderr)
        sys.exit(1)
    print(f'[INFO] 通过 ros2 launch 启动{description}')
    result = subprocess.run(
        ['ros2', 'launch', PACKAGE_NAME, launch_file]
    )
    sys.exit(result.returncode)


def run_task1_test() -> None:
    _run_script(
        os.path.join(PACKAGE_ROOT, 'test', 'test_sim_teleop.py'),
        '任务①仿真与键盘测试',
    )


def run_pytest() -> None:
    result = subprocess.run(
        [sys.executable, '-m', 'pytest', os.path.join(PACKAGE_ROOT, 'test'), '-q']
    )
    sys.exit(result.returncode)


def check_model() -> None:
    sys.path.insert(0, PACKAGE_ROOT)
    import mujoco
    from mujoco_auto_parking.simulator import MujocoSimulation

    model_path = os.path.join(PACKAGE_ROOT, 'models', 'parking_lot.xml')
    simulation = MujocoSimulation(model_path)
    print(f'模型: {model_path}')
    print(f'刚体数: {simulation.model.nbody}')
    print(f'关节数: {simulation.model.njnt}')
    print(f'执行器数: {simulation.model.nu}')
    print(
        '相机: '
        f'{simulation.object_names(mujoco.mjtObj.mjOBJ_CAMERA)}'
    )
    print(
        '安装点: '
        f'{simulation.object_names(mujoco.mjtObj.mjOBJ_SITE)}'
    )
    print('检查通过：模型可以加载。')


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='MuJoCo 无人车自动泊车')
    parser.add_argument('--launch', action='store_true', help='启动 ROS2 launch')
    parser.add_argument('--test', action='store_true', help='运行任务①测试')
    parser.add_argument('--pytest', action='store_true', help='运行全部 pytest')
    parser.add_argument('--check-model', action='store_true', help='检查 MuJoCo 模型')
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if args.test:
        run_task1_test()
    elif args.pytest:
        run_pytest()
    elif args.check_model:
        check_model()
    elif args.launch:
        _run_launch('main.launch.py', '自动泊车仿真')
    else:
        parser.print_help()
        print('\n提示：没有参数时请用 ./main.sh --launch 启动仿真。')


if __name__ == '__main__':
    main()
