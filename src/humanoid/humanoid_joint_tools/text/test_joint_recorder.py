import json
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from humanoid_joint_tools.record import JointRecorder

JOINT_NAMES = ['left_shoulder_joint', 'right_shoulder_joint']
FRAME_COUNT = 10

@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

def call_service(executor, client, timeout=5.0):
    assert client.wait_for_service(timeout_sec=timeout), f'{client.srv_name} not available'
    future = client.call_async(Trigger.Request())
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert future.done(), f'{client.srv_name} did not respond'
    return future.result()

def test_record_and_save(ros_context, tmp_path):
    filename = str(tmp_path / 'traj.json')
    recorder = JointRecorder()
    recorder.set_parameters([Parameter('filename', Parameter.Type.STRING, filename)])
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)

    start_client = recorder.create_client(Trigger, '/start_record')
    stop_client = recorder.create_client(Trigger, '/stop_record')
    publisher = recorder.create_publisher(JointState, '/joint_states_raw', 20)

    deadline = time.time() + 2.0
    while recorder.count_subscribers('/joint_states_raw') == 0 and time.time() < deadline:
        executor.spin_once(timeout_sec=0.05)

    start_response = call_service(executor, start_client)
    assert start_response.success, start_response.message

    for i in range(FRAME_COUNT):
        msg = JointState()
        msg.name = JOINT_NAMES
        msg.position = [0.1 * i, -0.1 * i]
        publisher.publish(msg)
        executor.spin_once(timeout_sec=0.05)

    stop_response = call_service(executor, stop_client)
    assert stop_response.success, stop_response.message

    frames = json.loads(open(filename).read())
    assert len(frames) == FRAME_COUNT
    assert frames[0]['name'] == JOINT_NAMES
    assert frames[-1]['position'] == [0.1 * (FRAME_COUNT - 1), -0.1 * (FRAME_COUNT - 1)]
    times = [frame['time'] for frame in frames]
    assert times == sorted(times), 'recorded timestamps must be monotonic'

    executor.remove_node(recorder)
    recorder.destroy_node()

def test_stop_without_start(ros_context):
    recorder = JointRecorder()
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    stop_client = recorder.create_client(Trigger, '/stop_record')

    stop_response = call_service(executor, stop_client)
    assert not stop_response.success
    assert stop_response.message == 'No recording in progress.'

    executor.remove_node(recorder)
    recorder.destroy_node()
