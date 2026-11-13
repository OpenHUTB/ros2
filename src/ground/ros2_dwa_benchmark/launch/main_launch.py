import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    out = LaunchConfiguration("out")
    figdir = LaunchConfiguration("figdir")
    worlds = LaunchConfiguration("worlds")
    no_figures = LaunchConfiguration("no_figures")

    common = [sys.executable, "-m", "ros2_dwa_benchmark.main_benchmark",
              "--out", out, "--worlds", worlds]

    return LaunchDescription([
        DeclareLaunchArgument("out", default_value="benchmark_results.json",
                              description="where to write the JSON results"),
        DeclareLaunchArgument("figdir", default_value="figures",
                              description="where to write the figures"),
        DeclareLaunchArgument("worlds", default_value="10",
                              description="number of random worlds per density"),
        DeclareLaunchArgument("no_figures", default_value="false",
                              description="skip matplotlib plots"),
        ExecuteProcess(
            cmd=common + ["--figdir", figdir],
            output="screen",
            condition=IfCondition(no_figures, negated=True),
        ),
        ExecuteProcess(
            cmd=common + ["--no-figures"],
            output="screen",
            condition=IfCondition(no_figures),
        ),
    ])
