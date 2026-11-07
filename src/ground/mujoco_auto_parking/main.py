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


def run_task2() -> None:
    _run_launch('task2.launch.py', '任务②感知与轨迹控制')


def run_sensor_test() -> None:
    _run_script(
        os.path.join(PACKAGE_ROOT, 'test', 'test_sensors.py'),
        '任务②传感器测试',
    )


def run_nn_test() -> None:
    _run_script(
        os.path.join(PACKAGE_ROOT, 'test', 'test_task2_nn.py'),
        '任务②神经网络测试',
    )


def run_eval_trajectory() -> None:
    _run_script(
        os.path.join(PACKAGE_ROOT, 'test', 'test_trajectory_tracking.py'),
        '任务②轨迹跟踪评估',
    )


def run_task3() -> None:
    _run_launch('task3.launch.py', '任务③建图与导航')


def run_task3_test() -> None:
    _run_script(
        os.path.join(PACKAGE_ROOT, 'test', 'test_task3_nav.py'),
        '任务③建图与导航测试',
    )


def run_train_planning() -> None:
    _run_script(
        os.path.join(PACKAGE_ROOT, 'train', 'train_neural_astar.py'),
        'Neural A* 训练',
    )


def run_training() -> None:
    env = os.environ.copy()
    env.setdefault('MUJOCO_GL', 'osmesa')
    for relative in (
        os.path.join('train', 'train_perception.py'),
        os.path.join('train', 'train_trajectory.py'),
    ):
        script = os.path.join(PACKAGE_ROOT, relative)
        print(f'[INFO] 训练: {relative}')
        result = subprocess.run([sys.executable, script], env=env)
        if result.returncode != 0:
            sys.exit(result.returncode)


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
    parser.add_argument('--launch', action='store_true', help='启动任务①仿真与键盘')
    parser.add_argument('--task2', action='store_true', help='启动任务②感知与轨迹控制')
    parser.add_argument('--task3', action='store_true', help='启动任务③建图与导航')
    parser.add_argument('--test', action='store_true', help='运行任务①测试')
    parser.add_argument('--test-sensors', action='store_true', help='运行任务②传感器测试')
    parser.add_argument('--test-nn', action='store_true', help='运行任务②神经网络测试')
    parser.add_argument('--test3', action='store_true', help='运行任务③建图与导航测试')
    parser.add_argument('--eval-trajectory', action='store_true', help='评估轨迹跟踪')
    parser.add_argument('--train', action='store_true', help='重新训练任务②网络')
    parser.add_argument('--train-planning', action='store_true', help='训练 Neural A*')
    parser.add_argument('--pytest', action='store_true', help='运行全部 pytest')
    parser.add_argument('--check-model', action='store_true', help='检查 MuJoCo 模型')
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if args.launch:
        _run_launch('main.launch.py', '自动泊车仿真')
    elif args.task2:
        run_task2()
    elif args.task3:
        run_task3()
    elif args.test:
        run_task1_test()
    elif args.test_sensors:
        run_sensor_test()
    elif args.test_nn:
        run_nn_test()
    elif args.test3:
        run_task3_test()
    elif args.eval_trajectory:
        run_eval_trajectory()
    elif args.train:
        run_training()
    elif args.train_planning:
        run_train_planning()
    elif args.pytest:
        run_pytest()
    elif args.check_model:
        check_model()
    else:
        parser.print_help()
        print('\n提示：任务①用 ./main.sh --launch；任务②用 ./main.sh --task2。')


if __name__ == '__main__':
    main()
