"""Install the AirSim survey CLI and its ROS 2 launch wrapper."""

from glob import glob

from setuptools import setup


setup(
    name="drone_lawnmower_survey",
    version="0.2.0",
    py_modules=["main", "survey_core"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/drone_lawnmower_survey"],
        ),
        ("share/drone_lawnmower_survey", ["package.xml"]),
        ("share/drone_lawnmower_survey/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    extras_require={"airsim": ["airsim==1.8.1"]},
    zip_safe=False,
    entry_points={"console_scripts": ["survey=main:main"]},
)
