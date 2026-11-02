from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="survey_mission_planner",
                executable="survey_mission_planner_node",
                name="survey_mission_planner",
                output="screen",
            )
        ]
    )
