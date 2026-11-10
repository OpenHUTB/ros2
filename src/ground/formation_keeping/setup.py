from setuptools import setup, find_packages
setup(
    name="formation_keeping",
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/formation_keeping", ["package.xml"]),
        ("share/formation_keeping/launch", ["launch/main.launch.py"]),
        ("share/ament_index/resource_index/packages", ["resource/formation_keeping"]),
    ],
    install_requires=["pygame>=2.0"],
    entry_points={"console_scripts": [
        "formation_keeping = formation_keeping.main:main"]},
)
