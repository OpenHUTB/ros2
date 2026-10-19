from setuptools import find_packages, setup


package_name = "airlib_message_regression"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/sample_data",
            [package_name + "/sample_data/sample_messages.json"],
        ),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    package_data={package_name: ["sample_data/*.json"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="useruseruser452",
    maintainer_email="useruseruser452@users.noreply.github.com",
    description="Deterministic AirLib sensor payload contract checks.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "airlib_message_regression = airlib_message_regression.main:main",
        ]
    },
)
