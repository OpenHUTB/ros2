# Copyright 2026 xxbzy727749
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Package the UAV power-line inspection preview tool."""

from setuptools import find_packages, setup


package_name = "uav_powerline_inspection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="xxbzy727749",
    maintainer_email="xxbzy727749@users.noreply.github.com",
    description="Preview alternating UAV viewpoints for power-line inspection.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["preview = uav_powerline_inspection.cli:main"]},
)
