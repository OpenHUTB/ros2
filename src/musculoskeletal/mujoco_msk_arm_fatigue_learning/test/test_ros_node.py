from pathlib import Path
from types import SimpleNamespace

import pytest

from mujoco_msk_arm_fatigue_learning.policy import LearningPolicy
from mujoco_msk_arm_fatigue_learning.ros_node import FatigueLearningNode


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
    def __init__(self, parameters=None):
        self.parameters = dict(parameters or {})
        self.publishers = []
        self.publisher_topics = []
        self.subscriptions = []
        self.subscription_topics = []
        self.logger = FakeLogger()

    def declare_parameter(self, name, value):
        self.parameters.setdefault(name, value)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, _type, topic, _depth):
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


class FakeFloat:
    def __init__(self):
        self.data = 0.0


def make_implementation(parameters=None):
    repo_root = Path(__file__).resolve().parents[4]
    model_path = (
        repo_root
        / 'src'
        / 'musculoskeletal'
        / 'mujoco_musculoskeletal_arm_reach'
        / 'models'
        / 'arm.xml'
    )
    node = FakeNode(parameters)
    implementation = FatigueLearningNode(
        node,
        FakeJointState,
        FakePoint,
        FakeArray,
        FakeFloat,
        model_path,
    )
    return node, implementation


def test_interfaces_and_tick_publish_fatigue_state():
    node, implementation = make_implementation()
    implementation._tick()
    assert node.subscription_topics == ['target_position']
    assert node.publisher_topics == [
        'joint_states',
        'end_effector_position',
        'muscle_forces',
        'muscle_activations',
        'fatigue/state',
        'fatigue/compensation_gain',
        'fatigue/tracking_error',
    ]
    assert all(
        len(publisher.messages) == 1
        for publisher in node.publishers
    )
    assert len(node.publishers[4].messages[0].data) == 6
    assert len(node.publishers[5].messages[0].data) == 6


def test_learning_disabled_keeps_unit_gain():
    node, implementation = make_implementation({
        'learning_enabled': False,
    })
    implementation._tick()
    assert all(
        value == pytest.approx(1.0)
        for value in node.publishers[5].messages[0].data
    )


def test_nonfinite_target_is_rejected():
    node, implementation = make_implementation()
    original = implementation.target
    message = FakePoint()
    message.x = float('nan')
    message.z = -0.17
    node.subscriptions[0](message)
    assert implementation.target == original
    assert node.logger.messages[-1][0] == 'warning'


def test_policy_file_overrides_yaml_learning_parameters():
    policy_path = Path(__file__).with_name('_node_policy.json')
    try:
        policy = LearningPolicy(
            learning_rate=12.5,
            gain_decay_rate=0.4,
            maximum_gain=1.6,
        )
        policy.save(policy_path)
        node, implementation = make_implementation({
            'learning_rate': 1.0,
            'policy_file': str(policy_path),
        })
        assert implementation.compensator.learning_rate == pytest.approx(
            12.5
        )
        assert implementation.compensator.gain_decay_rate == pytest.approx(
            0.4
        )
        assert implementation.compensator.maximum_gain == pytest.approx(1.6)
    finally:
        policy_path.unlink(missing_ok=True)
