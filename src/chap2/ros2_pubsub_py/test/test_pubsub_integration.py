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

"""最小集成测试：同时运行 talker 与 listener，断言 listener 至少收到一条消息."""

import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ros2_pubsub_py.listener import ListenerNode
from ros2_pubsub_py.talker import TalkerNode


def test_listener_receives_at_least_one_message():
    """同时运行 talker 与 listener 后，listener 应至少收到一条 /chatter 消息."""
    rclpy.init()
    talker = TalkerNode()
    listener = ListenerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(talker)
    executor.add_node(listener)

    try:
        deadline = time.monotonic() + 10.0
        while listener.received_count == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert listener.received_count > 0, 'listener 未收到 /chatter 消息'
    finally:
        executor.shutdown()
        talker.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()
