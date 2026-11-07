from setuptools import setup, find_packages
setup(
    name="line_following_pid",
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/line_following_pid", ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/line_following_pid"]),
    ],
    install_requires=["pygame>=2.0"],
    entry_points={"console_scripts": [
        "line_following_pid = line_following_pid.main:main"]},
)
