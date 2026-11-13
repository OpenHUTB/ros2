from setuptools import setup, find_packages
setup(
    name="formation_keeping",
    version="0.0.1",
    description="多车编队保持 2D 仿真演示（pygame，非 ROS 节点）：领航者圆周轨迹、三角队形偏移与跟随者比例控制可视化。",
    maintainer="wqs159",
    maintainer_email="wqs159@users.noreply.github.com",
    license="MIT",
    packages=find_packages(),
    data_files=[
        ("share/formation_keeping", ["package.xml"]),
        ("share/formation_keeping/launch", ["launch/main.launch.py"]),
        ("share/ament_index/resource_index/packages", ["resource/formation_keeping"]),
    ],
    install_requires=["pygame>=2.0"],
    entry_points={"console_scripts": [
        "formation_keeping = formation_keeping.main:main"]},
)
