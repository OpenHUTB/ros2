from setuptools import setup

package_name = "airsim_ros2_gnss"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (
            f"share/{package_name}",
            ["package.xml", "README.md", "requirements.txt"],
        ),
        (f"share/{package_name}/launch", ["launch/bringup.launch.py"]),
        (f"share/{package_name}/config", ["config/gnss.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dmjtr",
    maintainer_email="dmjtr@users.noreply.github.com",
    description="ROS 2 Humble AirSim GNSS bridge.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = airsim_ros2_gnss.main:main",
        ],
    },
)
