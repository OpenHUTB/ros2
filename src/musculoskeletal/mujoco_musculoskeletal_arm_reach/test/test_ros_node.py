from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip('mujoco')

from mujoco_musculoskeletal_arm_reach.ros_node import ArmSimulationNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warning(self, message):
        self.messages.append(('warning', message))

    def error(self, message):
        self.messages.append(('error', message))


class FakeNode:
    def __init__(self, parameters):
        self.parameters = parameters
        self.publishers = []
        self.subscriptions = []
        self.publisher_topics = []
        self.subscription_topics = []
        self.logger = FakeLogger()

    def declare_parameter(self, name, value):
        self.parameters.setdefault(name, value)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, _message_type, topic, _depth):
        publisher = FakePublisher()
        self.publishers.append(publisher)
        self.publisher_topics.append(topic)
        return publisher

    def create_subscription(self, _type, topic, callback, _depth):
        self.subscriptions.append(callback)
        self.subscription_topics.append(topic)
        return callback

    def create_timer(self, _period, callback):
        return FakeTimer(callback)

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: 'stamp')
        )


class FakeJointState:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.name = []
        self.position = []
        self.velocity = []


class FakePoint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class FakeArray:
    def __init__(self):
        self.data = []


def make_implementation():
    model_path = Path(__file__).parents[1] / 'models' / 'arm.xml'
    node = FakeNode({
        'model_file': str(model_path),
        'publish_rate_hz': 100.0,
        'payload_kg': 0.0,
        'target_x_m': 0.38,
        'target_z_m': -0.17,
        'shoulder_kp': 32.0,
        'shoulder_kd': 2.0,
        'elbow_kp': 17.0,
        'elbow_kd': 1.6,
        'coactivation': 0.025,
    })
    implementation = ArmSimulationNode(
        node,
        FakeJointState,
        FakePoint,
        FakeArray,
        model_path.parents[1],
    )
    return node, implementation


def test_tick_publishes_arm_state_and_muscle_messages():
    node, implementation = make_implementation()

    implementation._tick()

    assert implementation.simulation.data.time == pytest.approx(0.01)
    assert len(node.publishers) == 4
    assert node.publisher_topics == [
        'joint_states',
        'end_effector_position',
        'muscle_forces',
        'muscle_activations',
    ]
    assert node.subscription_topics == ['target_position']
    joint_message = node.publishers[0].messages[0]
    assert joint_message.name == ['shoulder_joint', 'elbow_joint']
    assert len(joint_message.position) == 2
    assert len(node.publishers[1].messages[0].__dict__) == 3
    assert len(node.publishers[2].messages[0].data) == 6
    assert len(node.publishers[3].messages[0].data) == 6


def test_nonfinite_target_is_rejected_without_replacing_target():
    node, implementation = make_implementation()
    original_target = implementation.target
    message = FakePoint()
    message.x = float('nan')
    message.z = -0.10

    node.subscriptions[0](message)

    assert implementation.target == original_target
    assert node.logger.messages[-1][0] == 'warning'
    assert 'finite' in node.logger.messages[-1][1]


def test_tick_failure_safe_stops_and_cancels_timer(monkeypatch):
    node, implementation = make_implementation()

    def fail_step(_activations):
        raise RuntimeError('forced simulation failure')

    monkeypatch.setattr(implementation.simulation, 'step', fail_step)
    implementation._tick()

    assert implementation.timer.cancelled
    assert all(value == 0.0 for value in implementation.simulation.data.ctrl)
    assert node.logger.messages[-1][0] == 'error'
