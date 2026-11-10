"""启动传感器和学习控制节点。 / Launch sensor and learned controller nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model = LaunchConfiguration("model")
    duration = LaunchConfiguration("duration")
    cruise_speed = LaunchConfiguration("cruise_speed")
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="model/risk_speed_mlp.npz"),
        DeclareLaunchArgument("duration", default_value="8.0"),
        DeclareLaunchArgument("cruise_speed", default_value="1.2"),
        Node(package="mujoco_warehouse_uav_collision_braking", executable="sensor_node",
             name="warehouse_ranging", parameters=[{"duration": duration, "cruise_speed": cruise_speed}],
             output="screen"),
        Node(package="mujoco_warehouse_uav_collision_braking", executable="learned_controller",
             name="warehouse_learned_controller", parameters=[{"model": model, "duration": duration,
                                                                  "cruise_speed": cruise_speed}], output="screen"),
    ])
