from setuptools import find_packages, setup

setup(
    name="ros_topic_demo",
    version="0.0.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
)
