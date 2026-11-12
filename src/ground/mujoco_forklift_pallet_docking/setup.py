from glob import glob
from setuptools import setup

package_name = "mujoco_forklift_pallet_docking"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    py_modules=["main"],
    package_data={package_name: ["model.xml", "models/*.npz"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy>=1.23", "mujoco==3.12.0", "Pillow>=9.0"],
    tests_require=["pytest"],
    zip_safe=False,
    description="相机引导叉车托盘对准与低速插入仿真",
    license="Apache-2.0",
    entry_points={"console_scripts": ["forklift_pallet_docking = main:main", "main = main:main"]},
)
