from setuptools import find_packages, setup


package_name = "panoramic_camera_projection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="useruseruser452",
    maintainer_email="useruseruser452@users.noreply.github.com",
    description="Six-camera cubemap to equirectangular panorama projection.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "panorama_offline = panoramic_camera_projection.main:main",
            "panorama_node = panoramic_camera_projection.panorama_node:main",
        ]
    },
)
