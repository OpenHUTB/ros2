from setuptools import find_packages, setup


package_name = "uav_payload_cg_envelope"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    package_data={package_name: ["sample_data/*.json"]},
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/sample_data",
            [package_name + "/sample_data/cg_profile.json"],
        ),
        (
            "share/" + package_name + "/launch",
            ["launch/main.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="useruseruser452",
    maintainer_email="useruseruser452@users.noreply.github.com",
    description="Payload center-of-gravity envelope checks for UAVs.",
    license="MIT",
    entry_points={
        "console_scripts": [
            package_name + " = " + package_name + ".main:main",
        ]
    },
)
