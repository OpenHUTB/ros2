from setuptools import setup

package_name = "pure_pursuit_reverse"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "main = pure_pursuit_reverse.node:main",
            "path_sim = pure_pursuit_reverse.path_sim:main",
        ],
    },
    maintainer="hehaoye",
    maintainer_email="319849457+hehaoye@users.noreply.github.com",
    description="倒车纯追踪路径跟踪（ROS 2）",
    license="MIT",
)
