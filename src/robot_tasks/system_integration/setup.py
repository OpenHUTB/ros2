from glob import glob

from setuptools import find_packages, setup

package_name = "system_integration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="1woshichenwenguang1",
    maintainer_email="2507417911@qq.com",
    description="任务5：四模块系统集成（通道隔离 + 速度仲裁 + 看门狗 + 急停）",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 作业要求：入口以 main 开头
            "main_manager = system_integration.ros.manager_node:main",
            "main_demo = system_integration.demo:main",
        ],
    },
)
