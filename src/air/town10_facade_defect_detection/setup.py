from glob import glob

from setuptools import setup

name = "town10_facade_defect_detection"
setup(name=name, version="1.0.0", packages=[name],
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + name]),
                  ("share/" + name, ["package.xml", "README.md", "LICENSE"]),
                  ("share/" + name + "/launch", ["launch/detect.launch.py"]),
                  ("share/" + name + "/models", glob("models/*")),
                  ("share/" + name + "/samples", glob("samples/*"))],
      install_requires=["setuptools", "numpy", "opencv-python"],
      zip_safe=False, description="Town10 facade image defect detection",
      license="AGPL-3.0",
      entry_points={"console_scripts": ["facade_detect = " + name + ".ros_node:main",
                                        "facade_detect_files = " + name + ".main:main"]})
