from setuptools import setup

package_name = "uav_emergency_landing_selector"

setup(
    name=package_name,
    version="0.1.0",
    description="Constraint-aware UAV emergency landing site selection",
    maintainer="Leclerc666",
    maintainer_email="917646819leclerc@gmail.com",
    author="Leclerc666",
    author_email="917646819leclerc@gmail.com",
    license="Apache-2.0",
    packages=[package_name],
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
            "landing_site_sim = uav_emergency_landing_selector.nodes:main_simulator",
            "landing_site_selector = uav_emergency_landing_selector.nodes:main_selector",
            "landing_selector_demo = uav_emergency_landing_selector.headless:main",
        ]
    },
)
