from glob import glob
from setuptools import setup

package_name = "carla_nn_roundabout_entry"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    py_modules=["main"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"] + glob("launch/*.launch.py")),
        ("share/" + package_name + "/hook", ["hook/ament_prefix_path.dsv"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="",
    maintainer_email="",
    description="无信号环岛汇入车辆仿真",
    license="Apache-2.0",
    entry_points={"console_scripts": ["roundabout_main = main:main"]},
)
