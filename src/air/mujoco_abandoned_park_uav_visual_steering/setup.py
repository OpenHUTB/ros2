from setuptools import setup

setup(
    name="mujoco_abandoned_park_uav_visual_steering",
    version="0.2.0",
    packages=["mujoco_abandoned_park_uav_visual_steering"],
    package_dir={"": "mujoco_abandoned_park_uav_visual_steering"},
    data_files=[("share/ament_index/resource_index/packages", ["resource/mujoco_abandoned_park_uav_visual_steering"])],
    entry_points={"console_scripts": ["visual_steering_node=ros_node:ros_main"]},
)


