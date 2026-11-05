from setuptools import find_packages, setup

setup(
    name="tripod_mujoco",
    version="0.0.1",
    description="三足仿生机器人 MuJoCo 仿真模块（catkin_python_setup）",
    packages=find_packages("src"),
    package_dir={"": "src"},
    install_requires=["mujoco>=3.2", "numpy", "imageio>=2.34.0", "imageio-ffmpeg", "pillow"],
)
