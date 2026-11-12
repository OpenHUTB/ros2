from setuptools import setup


package_name = "vehicle_wading_check"

setup(
    name=package_name,
    version="0.1.0",
    description="Vehicle wading-depth and buoyancy-margin check",
    maintainer="jiangdong000",
    maintainer_email="jiangdong000@users.noreply.github.com",
    author="jiangdong000",
    author_email="jiangdong000@users.noreply.github.com",
    license="Apache-2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "main_check = vehicle_wading_check.nodes:main_checker",
            "main_sim = vehicle_wading_check.nodes:main_simulator",
            "main_demo = vehicle_wading_check.demo:main",
            "main_report = vehicle_wading_check.report:main",
        ]
    },
)
