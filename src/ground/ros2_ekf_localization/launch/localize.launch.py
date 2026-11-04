"""ROS2 启动文件：启动 EKF 融合定位节点。

    ros2 launch ros2_ekf_localization localize.launch.py
    ros2 launch ros2_ekf_localization localize.launch.py sigma_pos:=0.05 use_pose:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("sigma_v", default_value="0.15",
                              description="里程计线速度噪声 (m/s)"),
        DeclareLaunchArgument("sigma_w", default_value="0.15",
                              description="里程计角速度噪声 (rad/s)"),
        DeclareLaunchArgument("sigma_imu_yaw", default_value="0.05",
                              description="IMU 航向噪声 (rad)"),
        DeclareLaunchArgument("sigma_pos", default_value="0.15",
                              description="位置观测噪声 (m)"),
        DeclareLaunchArgument("publish_rate", default_value="20.0",
                              description="预测与发布频率 (Hz)"),
        DeclareLaunchArgument("use_imu", default_value="true",
                              description="是否使用 IMU 航向观测"),
        DeclareLaunchArgument("use_pose", default_value="true",
                              description="是否使用绝对位置观测"),
        DeclareLaunchArgument("odom_timeout", default_value="0.5",
                              description="里程计数据超时 (s)"),
        DeclareLaunchArgument("namespace", default_value="",
                              description="节点命名空间"),
    ]

    node = Node(
        package="ros2_ekf_localization",
        executable="ekf_localization_node",
        name="ekf_localization_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "sigma_v": LaunchConfiguration("sigma_v"),
                "sigma_w": LaunchConfiguration("sigma_w"),
                "sigma_imu_yaw": LaunchConfiguration("sigma_imu_yaw"),
                "sigma_pos": LaunchConfiguration("sigma_pos"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "use_imu": LaunchConfiguration("use_imu"),
                "use_pose": LaunchConfiguration("use_pose"),
                "odom_timeout": LaunchConfiguration("odom_timeout"),
            }
        ],
    )

    return LaunchDescription(args + [node])
