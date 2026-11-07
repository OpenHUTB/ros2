from setuptools import setup, find_packages
setup(
    name="bug2_wall_following",
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/" + "bug2_wall_following", ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/bug2_wall_following"]),
    ],
    install_requires=["pygame>=2.0"],
    entry_points={"console_scripts": [
        "bug2_wall_following = bug2_wall_following.main:main"]},
)
