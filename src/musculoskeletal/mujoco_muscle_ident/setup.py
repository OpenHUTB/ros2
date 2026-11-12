import glob
import os
from setuptools import setup

package_name = "mujoco_muscle_ident"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "main.py", "README.md", "requirements.txt"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "tools"), ["tools/plot_metrics.py"]),
    ],
    install_requires=[
        "setuptools",
        "numpy>=1.21.5",
        "matplotlib",
        "scipy",
        "mujoco>=3.1.6"
    ],
    zip_safe=True,
    maintainer="VirtuosaHutb",
    maintainer_email="VirtuosaHutb@users.noreply.github.com",
    description="肌肉参数辨识（LM + 相关性 + 噪声/初值敏感性）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "muscle_ident = mujoco_muscle_ident.cli:main"
        ]
    },
)
