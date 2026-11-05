from setuptools import find_packages, setup

setup(
    name="go2_mujoco",
    version="0.0.1",
    description="MuJoCo 机器人仿真模块（catkin_python_setup）",
    packages=find_packages("src"),
    package_dir={"": "src"},
    install_requires=["mujoco>=3.2", "mujoco-menagerie", "numpy", "imageio>=2.34.0", "imageio-ffmpeg", "pillow"],
)
