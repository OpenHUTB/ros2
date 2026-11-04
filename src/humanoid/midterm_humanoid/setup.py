from setuptools import find_packages, setup

setup(
    name="midterm_humanoid",
    version="0.0.1",
    description="简易人形机器人 MuJoCo 仿真（原地踏步 + 交替摆臂）",
    packages=find_packages("src"),
    package_dir={"": "src"},
    install_requires=["mujoco>=3.0", "numpy", "imageio", "imageio-ffmpeg"],
)
