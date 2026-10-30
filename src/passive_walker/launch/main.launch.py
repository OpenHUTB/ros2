"""启动无动力被动行走器的 ROS 2 节点，发布 /passive_walker/steps。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    seconds_arg = DeclareLaunchArgument('seconds', default_value='20.0')
    push_arg = DeclareLaunchArgument('push', default_value='0.3')

    # main.py 是普通命令行程序（非 ROS 节点），用 ExecuteProcess 启动，
    # 避免 Node 自动追加 --ros-args 导致 argparse 报错。
    main_py = PathJoinSubstitution([
        FindPackageShare('passive_walker'), '..', '..',
        'lib', 'passive_walker', 'main.py',
    ])

    run = ExecuteProcess(
        cmd=[
            main_py, 'node',
            '--seconds', LaunchConfiguration('seconds'),
            '--push', LaunchConfiguration('push'),
        ],
        output='screen',
    )

    return LaunchDescription([seconds_arg, push_arg, run])
