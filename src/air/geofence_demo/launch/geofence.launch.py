# Copyright 2026 Toeting
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

"""启动禁飞区检测节点，可通过 launch 参数指定期望 frame_id."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """构建 geofence_node 的启动描述."""
    expected_frame_id = LaunchConfiguration('expected_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'expected_frame_id', default_value='map',
            description='期望的坐标系 frame_id，用于一致性校验'),
        Node(
            package='geofence_demo',
            executable='geofence_node',
            name='geofence_node',
            output='screen',
            parameters=[{
                'expected_frame_id': expected_frame_id,
            }],
        ),
    ])
