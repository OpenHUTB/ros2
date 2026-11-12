from setuptools import find_packages, setup


package_name = "powerline_mission_preflight"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/demo_mission.json"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="xxbzy727749",
    maintainer_email="xxbzy727749@users.noreply.github.com",
    description="Preflight validation for UAV power-line inspection missions.",
    license="MIT",
    entry_points={"console_scripts": ["check = powerline_mission_preflight.cli:main"]},
)
