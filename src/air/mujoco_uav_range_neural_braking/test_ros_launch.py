"""在 ROS 2 环境中验证真实 launch 进程和话题，其他环境明确跳过。"""

import math
from pathlib import Path
import shutil
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="该测试需要 ROS 2 运行环境")
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray

from ros_messages import TOPIC_BRAKING_ACCEL, TOPIC_FRONT_RANGE, TOPIC_STATE


@pytest.mark.parametrize("controller", ["reference", "range", "mlp"])
def test_launch_publishes_topics_and_exits_after_simulation(controller, tmp_path):
    ros2 = shutil.which("ros2")
    if ros2 is None:
        pytest.skip("未找到 ros2 命令")
    launch_file = Path(__file__).resolve().parent / "launch" / "ros_node.launch.py"
    ranges, accelerations, states = [], [], []
    context = Context()
    rclpy.init(context=context)
    subscriber = Node(f"uav_launch_test_{controller}", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(subscriber)
    subscriber.create_subscription(Float32, TOPIC_FRONT_RANGE, lambda msg: ranges.append(msg.data), 10)
    subscriber.create_subscription(Float32, TOPIC_BRAKING_ACCEL, lambda msg: accelerations.append(msg.data), 10)
    subscriber.create_subscription(Float64MultiArray, TOPIC_STATE, lambda msg: states.append(list(msg.data)), 10)
    process = None
    log_path = tmp_path / "launch.log"
    try:
        with log_path.open("w", encoding="utf-8") as log:
            # 使用绝对 launch 路径并从其他目录启动，检查资源定位和解释器参数。
            process = subprocess.Popen(
                [ros2, "launch", str(launch_file), f"controller:={controller}", f"python3:={sys.executable}"],
                cwd=tmp_path, stdout=log, stderr=subprocess.STDOUT,
            )
            deadline = time.monotonic() + 30
            while process.poll() is None and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            if process.poll() is None:
                pytest.fail("ROS 节点未在仿真结束后正常退出")
        output = log_path.read_text(encoding="utf-8")
        assert process.returncode == 0, output
        assert len(states) >= 20, output
        assert len(ranges) >= 20 and len(accelerations) >= 20
        assert all(len(state) == 4 and all(math.isfinite(value) for value in state) for state in states)
        assert all(left[0] < right[0] for left, right in zip(states, states[1:]))
        assert 9.0 <= states[-1][0] < 10.0
        assert all(-2.51 <= value <= 1.21 for value in accelerations)
        assert all(math.isfinite(value) for value in ranges)
        assert "仿真时长已结束" in output
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        executor.shutdown()
        subscriber.destroy_node()
        if context.ok():
            context.shutdown()
