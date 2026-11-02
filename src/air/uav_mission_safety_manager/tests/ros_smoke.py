"""Run after sourcing the workspace: python3 tests/ros_smoke.py (no simulator)."""
import time
from concurrent.futures import Future
from unittest.mock import patch

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger

from uav_mission_safety_manager.main import MissionNode
from uav_mission_safety_manager.state_machine import State


class PendingAdapter:
    def submit(self, command, **data):
        return Future()

    def sample(self):
        return Future()


def main():
    rclpy.init(args=['--ros-args', '-p', 'use_airsim:=true'])
    executor = SingleThreadedExecutor()
    with patch('uav_mission_safety_manager.main.AirSimAdapter', return_value=PendingAdapter()):
        node = MissionNode()
    observer = rclpy.create_node('mission_smoke_observer')
    executor.add_node(node)
    executor.add_node(observer)
    messages = []
    observer.create_subscription(String, '/uav_mission_safety_manager/status',
                                 lambda msg: messages.append(msg.data), 10)
    service = observer.create_client(Trigger, '/uav_mission_safety_manager/return_home')
    try:
        node.controller.connected = True
        node.controller.home = (10., 20., 0.)
        node.controller.position = (30., 20., -10.)
        node.machine.state = State.EXECUTING
        assert service.wait_for_service(timeout_sec=3)
        result = service.call_async(Trigger.Request())
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert result.done() and result.result().success, 'service blocked on RPC'
        assert node.controller.pending is not None, 'test requires an unfinished RPC'
        assert len(messages) >= 3, 'status timer blocked on RPC'
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert node.machine.state == State.FAILSAFE, 'timeout check blocked on RPC'
        print('PASS: actual ROS service, status timer and safety timeout remain responsive')
    finally:
        executor.shutdown()
        observer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
