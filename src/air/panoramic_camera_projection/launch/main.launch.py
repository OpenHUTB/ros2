from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    str(package_root / "main.py"),
                    "--output",
                    str(package_root / "output" / "panorama.png"),
                ],
                output="screen",
            )
        ]
    )
