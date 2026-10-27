"""ament_python与Python包安装配置。"""

from setuptools import find_packages, setup

setup(
    name="town10_facade_cruise_capture",
    version="0.1.0",
    packages=find_packages(),
    package_data={
        "town10_capture": [
            "assets/*.png",
            "assets/*.json",
            "assets/*.txt",
            "models/*.json",
        ]
    },
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/town10_facade_cruise_capture"],
        ),
        ("share/town10_facade_cruise_capture", ["package.xml"]),
        ("share/town10_facade_cruise_capture/launch", ["launch/main.launch.py"]),
    ],
    install_requires=["setuptools", "numpy>=1.24,<3", "Pillow>=9", "mujoco==3.12.0"],
    extras_require={"airsim": ["airsim==1.8.1"]},
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "mission=town10_capture.ros_node:main",
            "town10-cruise=town10_capture.cli:main",
        ]
    },
)
