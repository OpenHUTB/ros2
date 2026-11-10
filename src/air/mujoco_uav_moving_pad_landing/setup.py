from setuptools import setup

package_name = "mujoco_uav_moving_pad_landing"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_dir={package_name: "."},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/landing.launch.py"]),
        ("share/" + package_name + "/assets", ["assets/moving_pad.xml"]),
    ],
    install_requires=["setuptools", "numpy", "mujoco", "opencv-python-headless", "scikit-learn", "Pillow", "joblib"],
    zip_safe=True,
    entry_points={"console_scripts": ["landing_controller = mujoco_uav_moving_pad_landing.ros_node:main"]},
)
