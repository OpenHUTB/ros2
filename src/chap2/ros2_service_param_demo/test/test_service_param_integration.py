# Copyright 2026 Gao2006-12
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

"""集成测试：服务调用与参数更新是否按预期生效."""

import time

import pytest

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros2_service_param_demo.uav_mode_node import UavModeNode
from std_srvs.srv import SetBool


pytestmark = pytest.mark.ros_integration

def test_service_call_switches_mode_to_flying():
    """通过真实服务调用把模式切换到 FLYING，断言服务端模式随之改变."""
    rclpy.init()
    server = UavModeNode()
    client_node = Node('uav_mode_test_client')
    client = client_node.create_client(SetBool, '/uav/set_mode')
    try:
        assert client.wait_for_service(timeout_sec=10.0), '找不到服务 /uav/set_mode'

        request = SetBool.Request()
        request.data = True
        future = client.call_async(request)

        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(server, timeout_sec=0.05)
            rclpy.spin_once(client_node, timeout_sec=0.05)

        assert future.done(), '服务调用超时'
        response = future.result()
        assert response.success, '服务返回 success=False'
        assert server.mode == 'FLYING', '服务端模式未切换为 FLYING'
    finally:
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


def test_publish_rate_parameter_is_applied():
    """更新 publish_rate 后立即生效，非法值应被拒绝且原值保持不变."""
    rclpy.init()
    node = UavModeNode()
    try:
        accepted = node.set_parameters([
            Parameter('publish_rate', Parameter.Type.DOUBLE, 5.0)])
        assert accepted[0].successful, '合法的 publish_rate 应被接受'
        assert node.publish_rate == 5.0

        rejected = node.set_parameters([
            Parameter('publish_rate', Parameter.Type.DOUBLE, -1.0)])
        assert not rejected[0].successful, '负数 publish_rate 应被拒绝'
        assert node.publish_rate == 5.0, '被拒绝后原值不应改变'
    finally:
        node.destroy_node()
        rclpy.shutdown()
