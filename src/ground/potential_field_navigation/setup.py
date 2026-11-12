from setuptools import setup, find_packages
setup(
    name="potential_field_navigation",
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/potential_field_navigation", ["package.xml"]),
        ("share/potential_field_navigation/launch", ["launch/main.launch.py"]),
        ("share/ament_index/resource_index/packages", ["resource/potential_field_navigation"]),
    ],
    install_requires=["pygame>=2.0"],
    entry_points={"console_scripts": [
        "potential_field_navigation = potential_field_navigation.main:main"]},
)
