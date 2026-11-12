from glob import glob
from setuptools import find_packages, setup

setup(
    name="mujoco_abandoned_park_uav_visual_steering",
    version="0.2.0",
    packages=find_packages(),
    py_modules=["ros_node", "train", "main", "evaluation", "neural_navigation", "train_neural_navigation"],
    scripts=["visual_steering_node"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/mujoco_abandoned_park_uav_visual_steering"]),
        ("share/mujoco_abandoned_park_uav_visual_steering", ["package.xml"]),
        ("share/mujoco_abandoned_park_uav_visual_steering/launch", glob("launch/*.launch.py")),
        ("share/mujoco_abandoned_park_uav_visual_steering/config", glob("config/*.yaml")),
        ("share/mujoco_abandoned_park_uav_visual_steering/models", glob("models/*.xml")),
        ("share/mujoco_abandoned_park_uav_visual_steering/models/weights", glob("models/weights/*.pt")),
    ],
    install_requires=["setuptools"],
)
