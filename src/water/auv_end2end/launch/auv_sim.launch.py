import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('auv_end2end')
    world_path = os.path.join(share_dir, 'worlds', 'underwater.world')
    rviz_config = os.path.join(share_dir, 'rviz', 'auv.rviz')

    return LaunchDescription([
        # 避障参数，可在命令行覆盖：
        # ros2 launch auv_end2end auv_sim.launch.py trigger_ratio:=0.3
        DeclareLaunchArgument(
            'obstacle_thresh', default_value='22',
            description='固定灰度阈值（0-255），关闭自适应时生效'),
        DeclareLaunchArgument(
            'adaptive_thresh', default_value='true',
            description='true 时用 Otsu 自适应阈值，抗雾效与光照变化'),
        DeclareLaunchArgument(
            'trigger_ratio', default_value='0.22',
            description='单侧暗像素占比超过该值触发避障'),
        DeclareLaunchArgument(
            'recover_ratio', default_value='0.18',
            description='双侧占比均低于该值恢复直行'),
        DeclareLaunchArgument(
            'forward_speed', default_value='4.5',
            description='直行巡航速度（m/s）'),
        DeclareLaunchArgument(
            'side_speed', default_value='2.8',
            description='横向避让速度（m/s）'),
        DeclareLaunchArgument(
            'max_avoid_duration', default_value='3.0',
            description='单次最长避让时长（s），超时进入冷却期'),
        DeclareLaunchArgument(
            'avoid_cooldown', default_value='2.0',
            description='冷却时长（s），期间强制直行'),
        DeclareLaunchArgument(
            'min_avoid_duration', default_value='0.3',
            description='最小避让时长（s），防止阈值附近抖动'),
        DeclareLaunchArgument(
            'emergency_factor', default_value='1.5',
            description='冷却期占比超过触发阈值的该倍数时立即重新避让'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='true 时启动 RViz2 显示 /auv/trajectory 轨迹'),
        DeclareLaunchArgument(
            'record_csv', default_value='',
            description='轨迹 CSV 输出路径，留空则不写文件'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='true 时启动 gzclient 图形界面，无显示环境请设为 false'),
        # 启动Gazebo仿真
        # gazebo_ros_init 是 SystemPlugin，必须在 gzserver 启动时用 -s 加载，
        # 只有这样才会发布 /clock，use_sim_time 的节点才不会停在 0 时刻
        ExecuteProcess(
            cmd=['gzserver', '--verbose',
                 '-s', 'libgazebo_ros_init.so', world_path],
            output='screen',
            name='gzserver'
        ),
        # gzclient 单独启动：没有 DISPLAY 时只会关掉界面，仿真本身继续运行
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
        # 启动视觉避障节点（节点名沿用代码里的 auv_obstacle_avoid）
        Node(
            package='auv_end2end',
            executable='auv_obstacle_avoid',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'obstacle_thresh': ParameterValue(
                    LaunchConfiguration('obstacle_thresh'), value_type=int),
                'adaptive_thresh': ParameterValue(
                    LaunchConfiguration('adaptive_thresh'), value_type=bool),
                'trigger_ratio': ParameterValue(
                    LaunchConfiguration('trigger_ratio'), value_type=float),
                'recover_ratio': ParameterValue(
                    LaunchConfiguration('recover_ratio'), value_type=float),
                'forward_speed': ParameterValue(
                    LaunchConfiguration('forward_speed'), value_type=float),
                'side_speed': ParameterValue(
                    LaunchConfiguration('side_speed'), value_type=float),
                'max_avoid_duration': ParameterValue(
                    LaunchConfiguration('max_avoid_duration'),
                    value_type=float),
                'avoid_cooldown': ParameterValue(
                    LaunchConfiguration('avoid_cooldown'), value_type=float),
                'min_avoid_duration': ParameterValue(
                    LaunchConfiguration('min_avoid_duration'),
                    value_type=float),
                'emergency_factor': ParameterValue(
                    LaunchConfiguration('emergency_factor'),
                    value_type=float),
            }]
        ),
        # 轨迹记录：发布 /auv/trajectory 供 RViz 显示，可选写 CSV
        Node(
            package='auv_end2end',
            executable='trajectory_recorder',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'csv_path': LaunchConfiguration('record_csv'),
            }]
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])
