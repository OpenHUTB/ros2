from setuptools import setup

package_name = "uav_weather_risk"

setup(
    name=package_name,
    version="0.1.0",
    description="Multi-source weather disturbance prediction and UAV risk assessment",
    maintainer="Leclerc666",
    maintainer_email="917646819leclerc@gmail.com",
    author="Leclerc666",
    author_email="917646819leclerc@gmail.com",
    license="Apache-2.0",
    packages=[package_name, package_name + ".core"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "LICENSE"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
        ("share/" + package_name + "/config", ["config/demo.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "weather_demo = uav_weather_risk.demo_node:main",
            "weather_risk = uav_weather_risk.risk_node:main",
            "weather_risk_headless = uav_weather_risk.headless_demo:main",
        ]
    },
)
