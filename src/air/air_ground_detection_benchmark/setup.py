from setuptools import find_packages, setup


package_name = "air_ground_detection_benchmark"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/sample_data",
            ["sample_data/sample_frames.json"],
        ),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="useruseruser452",
    maintainer_email="useruseruser452@users.noreply.github.com",
    description="Detection quality benchmark by UAV altitude and pitch.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "air_ground_detection_benchmark = air_ground_detection_benchmark.main:main",
        ]
    },
)
