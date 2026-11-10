from glob import glob
from setuptools import find_packages, setup


package_name = "mujoco_musculoskeletal_elbow_nn_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/models", glob("models/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="largels",
    maintainer_email="867976558@qq.com",
    description="MuJoCo musculoskeletal elbow trajectory control",
    license="MIT",
    entry_points={
        "console_scripts": [
            "elbow_sim_node = mujoco_musculoskeletal_elbow_nn_control.ros_node:main",
            "collect_episodes = mujoco_musculoskeletal_elbow_nn_control.collector:main",
        ]
    },
)
