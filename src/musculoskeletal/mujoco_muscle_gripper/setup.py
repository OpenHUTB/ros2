import glob
import os

from setuptools import setup

package_name = "mujoco_muscle_gripper"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name,
         ["package.xml", "main.py", "main.sh", "main.bat", "README.md",
          "requirements.txt"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "tools"), ["tools/plot_metrics.py"]),
    ],
    install_requires=["setuptools", "numpy", "mujoco", "Pillow"],
    zip_safe=True,
    maintainer="lyzzzz0921",
    maintainer_email="lyzzzz0921@users.noreply.github.com",
    description="肌肉驱动的两指夹爪（真实接触抓取）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "muscle_gripper = mujoco_muscle_gripper.cli:main",
        ],
    },
)
