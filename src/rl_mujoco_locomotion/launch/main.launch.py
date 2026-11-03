from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    env_arg = DeclareLaunchArgument('env', default_value='halfcheetah')
    alg_arg = DeclareLaunchArgument('alg', default_value='ppo')
    episodes_arg = DeclareLaunchArgument('episodes', default_value='50')
    max_steps_arg = DeclareLaunchArgument('max_steps', default_value='20000')

    # main.py 是普通命令行程序（非 ROS 节点），因此用 ExecuteProcess 启动，
    # 避免 Node 自动追加 --ros-args 导致 argparse 报错。
    main_py = PathJoinSubstitution([
        FindPackageShare('rl_mujoco_locomotion'), '..', '..',
        'lib', 'rl_mujoco_locomotion', 'main.py',
    ])

    train = ExecuteProcess(
        cmd=[
            main_py, 'train',
            '--env', LaunchConfiguration('env'),
            '--alg', LaunchConfiguration('alg'),
            '--episodes', LaunchConfiguration('episodes'),
            '--max-steps', LaunchConfiguration('max_steps'),
        ],
        output='screen',
    )

    return LaunchDescription([env_arg, alg_arg, episodes_arg, max_steps_arg, train])
