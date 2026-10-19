from setuptools import setup

setup(
    name="mujoco_abandoned_park_uav_visual_steering",
    version="0.2.0",
    packages=["mujoco_abandoned_park_uav_visual_steering"],
    package_dir={"": "."},
    py_modules=["evaluation", "main", "ros_node", "train"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/mujoco_abandoned_park_uav_visual_steering"]),
        ("share/mujoco_abandoned_park_uav_visual_steering", ["package.xml"]),
        ("share/mujoco_abandoned_park_uav_visual_steering/config", ["config/ros_visual_steering.yaml"]),
        ("share/mujoco_abandoned_park_uav_visual_steering/launch", ["launch/main.launch.py"]),
        ("lib/mujoco_abandoned_park_uav_visual_steering", ["visual_steering_node"]),
    ],
    install_requires=["setuptools"],
)

