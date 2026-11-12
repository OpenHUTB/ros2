from setuptools import setup

package_name = "rov_thruster_allocation_ls"

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
            "main = rov_thruster_allocation_ls.node:main",
            "wrench_sim = rov_thruster_allocation_ls.wrench_sim:main",
        ],
    },
    maintainer="hehaoye",
    maintainer_email="319849457+hehaoye@users.noreply.github.com",
    description="ROV 推力分配（ROS 2）",
    license="Apache-2.0",
)
