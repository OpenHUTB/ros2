from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_battery_rth_failsafe',
            executable='battery_rth_node',
            name='battery_rth_failsafe',
            output='screen',
            parameters=[{
                # 电量阈值(%)
                'warn_percent': 25.0,
                'rth_percent': 15.0,
                'emergency_percent': 8.0,
                # 返航耗电估算
                'power_per_meter': 0.15,
                'cruise_speed_mps': 3.0,
                # 起飞点(ENU / map)
                'home_x': 0.0,
                'home_y': 0.0,
                'home_z': 0.0,
                # 迫降点, 分号分隔 "x,y,z;x,y,z"; 留空则原地垂直迫降
                'safe_zones': '',
                # 电池话题断流判据(秒)
                'battery_timeout': 3.0,
                # 话题名, 接不同仿真底座时按第 3 节的映射覆盖
                'battery_topic': '/drone/battery',
                'position_topic': '/drone/position',
                'setpoint_topic': '/drone/setpoint_position',
                'state_topic': '/drone/failsafe_state',
            }],
        ),
    ])
