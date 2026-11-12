from setuptools import setup, find_packages
setup(
    name="two_link_arm_ik",
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/two_link_arm_ik", ["package.xml"]),
        ("share/two_link_arm_ik/launch", ["launch/main.launch.py"]),
        ("share/ament_index/resource_index/packages", ["resource/two_link_arm_ik"]),
    ],
    install_requires=["pygame>=2.0"],
    entry_points={"console_scripts": [
        "two_link_arm_ik = two_link_arm_ik.main:main"]},
)
