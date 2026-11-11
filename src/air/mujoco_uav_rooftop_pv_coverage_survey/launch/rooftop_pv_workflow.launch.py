"""Launch any pipeline mode without adding ROS node arguments / 启动数据与训练流程。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="scene"),
        DeclareLaunchArgument("output", default_value="survey_artifacts"),
        DeclareLaunchArgument("scenario", default_value="sparse"),
        ExecuteProcess(cmd=["ros2", "run", "mujoco_uav_rooftop_pv_coverage_survey", "rooftop_pv_pipeline",
                            "--mode", LaunchConfiguration("mode"), "--output", LaunchConfiguration("output"),
                            "--scenario", LaunchConfiguration("scenario")], output="screen"),
    ])
