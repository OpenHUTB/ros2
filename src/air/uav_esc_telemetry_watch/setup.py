from setuptools import setup
pkg = "uav_esc_telemetry_watch"
setup(name=pkg, version="0.1.0", description="ESC telemetry watchdog",
      maintainer="2535010018", maintainer_email="2535010018@stu.hutb.edu.cn",
      author="2535010018", author_email="2535010018@stu.hutb.edu.cn", license="Apache-2.0",
      packages=[pkg],
      data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + pkg]),
        ("share/" + pkg, ["package.xml", "README.md", "LICENSE"]),
        ("share/" + pkg + "/launch", ["launch/main.launch.py"]),
        ("share/" + pkg + "/config", ["config/demo.yaml"]),
      ],
      install_requires=["setuptools"], zip_safe=True,
      entry_points={"console_scripts": [
        "esc_sim = uav_esc_telemetry_watch.nodes:main_sim",
        "esc_watch = uav_esc_telemetry_watch.nodes:main_watch",
      ]})
