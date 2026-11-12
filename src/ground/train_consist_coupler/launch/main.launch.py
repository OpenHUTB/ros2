from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scenario = DeclareLaunchArgument(
        "scenario", default_value="consist_head_first",
        description="single_body / consist_sync / consist_head_first / consist_tail_first / consist_long_tail")
    playback_rate = DeclareLaunchArgument(
        "playback_rate", default_value="1.0", description="回放倍速")
    frame_step = DeclareLaunchArgument(
        "frame_step", default_value="1", description="每个周期跨过的帧数")
    hold_frame = DeclareLaunchArgument(
        "hold_frame", default_value="-1",
        description=">=0 时定格在该帧反复发布（截图/排障用），-1 表示正常回放")

    return LaunchDescription([
        scenario,
        playback_rate,
        frame_step,
        hold_frame,
        Node(
            package="train_consist_coupler",
            executable="consist_node",
            name="train_consist_coupler",
            output="screen",
            parameters=[{
                "scenario": LaunchConfiguration("scenario"),
                "playback_rate": ParameterValue(LaunchConfiguration("playback_rate"),
                                                value_type=float),
                "frame_step": ParameterValue(LaunchConfiguration("frame_step"),
                                             value_type=int),
                "hold_frame": ParameterValue(LaunchConfiguration("hold_frame"),
                                             value_type=int),
            }],
        ),
    ])
