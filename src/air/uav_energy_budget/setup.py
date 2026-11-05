from setuptools import find_packages, setup

package_name="uav_energy_budget"
setup(
    name=package_name, version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/"+package_name]),
        ("share/"+package_name, ["package.xml"]),
        ("share/"+package_name+"/sample_data", [package_name+"/sample_data/mission_profile.json"]),
        ("share/"+package_name+"/launch", ["launch/main.launch.py"]),
    ],
    package_data={package_name:["sample_data/*.json"]},
    install_requires=["setuptools"], zip_safe=True,
    maintainer="useruseruser452",
    maintainer_email="useruseruser452@users.noreply.github.com",
    description="Simplified UAV mission energy and endurance estimation.", license="MIT",
    entry_points={"console_scripts":["uav_energy_budget = uav_energy_budget.main:main"]},
)
