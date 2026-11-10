from glob import glob

from setuptools import find_packages, setup

package_name = "policy_learning"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/data", glob("data/*.npz")),
        (f"share/{package_name}/config", glob("config/*.rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="1woshichenwenguang1",
    maintainer_email="2507417911@qq.com",
    description="任务4：端到端策略网络（行为克隆 + DAgger）",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 作业要求：入口以 main 开头
            "main = policy_learning.main:main",
            "main_train = policy_learning.main_train:main",
            "main_policy = policy_learning.ros.main_policy_node:main",
            "main_sim = policy_learning.ros.main_sim_node:main",
        ],
    },
)
