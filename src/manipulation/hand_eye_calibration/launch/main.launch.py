"""通过 ROS 2 launch 启动机械臂手眼标定仿真。"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    # 被启动的是普通命令行程序（无 rclpy 节点），用 ExecuteProcess 启动，
    # 避免 launch_ros 的 Node 自动追加 --ros-args。
    app = PathJoinSubstitution(
        [FindPackagePrefix("hand_eye_calibration"), "lib", "hand_eye_calibration", "hand_eye_calibration"]
    )
    run = ExecuteProcess(cmd=[app], output="screen")
    return LaunchDescription([run])
