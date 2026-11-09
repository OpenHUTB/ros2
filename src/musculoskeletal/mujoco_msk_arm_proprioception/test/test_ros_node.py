from types import SimpleNamespace

import numpy as np
import pytest

from mujoco_msk_arm_proprioception.ros_node import ArmProprioceptionNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeTimer:
    def __init__(self, callback):
        self.callback = callback


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warning(self, message):
        self.messages.append(('warning', message))


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


class FakeJointState:
    def __init__(self):
        self.name = ['shoulder_joint', 'elbow_joint']
        self.position = [-0.2, 1.5]
        self.velocity = [0.1, -0.05]


class FakeArray:
    def __init__(self):
        self.data = []


def make_implementation():
    node = FakeNode()
    node.parameters['delay_s'] = 0.0
    implementation = ArmProprioceptionNode(
        node,
        FakeJointState,
        FakeArray,
    )
    return node, implementation


def feed_state(node, implementation):
    node.subscriptions[0](FakeJointState())
    force_message = FakeArray()
    force_message.data = [20.0] * 6
    node.subscriptions[1](force_message)
    activation_message = FakeArray()
    activation_message.data = [0.2] * 6
    node.subscriptions[2](activation_message)
    implementation._tick()


def test_interfaces_match_documented_topics():
    node, _ = make_implementation()
    assert node.subscription_topics == [
        'joint_states',
        'muscle_forces',
        'muscle_activations',
    ]
    assert node.publisher_topics == [
        'proprioception/spindle_length',
        'proprioception/spindle_velocity',
        'proprioception/golgi_force',
        'proprioception/tendon_extension',
    ]


def test_tick_publishes_four_six_value_messages():
    node, implementation = make_implementation()
    feed_state(node, implementation)
    assert len(node.publishers) == 4
    assert all(len(publisher.messages) == 1 for publisher in node.publishers)
    assert all(
        len(publisher.messages[0].data) == 6
        for publisher in node.publishers
    )


def test_tick_waits_until_all_inputs_arrive():
    node, implementation = make_implementation()
    node.subscriptions[0](FakeJointState())
    assert not implementation.inputs_ready
    implementation._tick()
    assert all(not publisher.messages for publisher in node.publishers)


def test_missing_joint_is_rejected():
    node, implementation = make_implementation()
    message = FakeJointState()
    message.name = ['shoulder_joint']
    message.position = [0.0]
    message.velocity = [0.0]
    node.subscriptions[0](message)
    assert node.logger.messages[-1][0] == 'warning'


def test_missing_velocity_defaults_to_zero():
    node, implementation = make_implementation()
    message = FakeJointState()
    message.velocity = []
    node.subscriptions[0](message)
    force_message = FakeArray()
    force_message.data = [20.0] * 6
    node.subscriptions[1](force_message)
    activation_message = FakeArray()
    activation_message.data = [0.2] * 6
    node.subscriptions[2](activation_message)
    implementation._tick()
    assert all(
        len(publisher.messages) == 1
        for publisher in node.publishers
    )


def test_malformed_force_callback_is_isolated_and_disables_ready():
    node, implementation = make_implementation()
    message = FakeArray()
    message.data = ['invalid'] * 6
    node.subscriptions[1](message)
    assert not implementation.inputs_ready
    assert node.logger.messages[-1][0] == 'warning'


def test_malformed_activation_callback_is_isolated():
    node, implementation = make_implementation()
    message = FakeArray()
    message.data = ['invalid'] * 6
    node.subscriptions[2](message)
    assert not implementation.inputs_ready
    assert node.logger.messages[-1][0] == 'warning'


def test_non_numeric_joint_callback_is_isolated():
    node, implementation = make_implementation()
    message = FakeJointState()
    message.position = ['invalid', 1.5]
    node.subscriptions[0](message)
    assert not implementation.inputs_ready
    assert node.logger.messages[-1][0] == 'warning'


def test_invalid_muscle_order_is_rejected_at_startup():
    node = FakeNode({
        'muscle_names': [
            'shoulder_extensor',
            'shoulder_flexor',
            'elbow_flexor',
            'elbow_extensor',
            'biarticular_flexor',
            'biarticular_extensor',
        ],
    })
    with pytest.raises(ValueError, match='muscle order'):
        ArmProprioceptionNode(
            node,
            FakeJointState,
            FakeArray,
        )


def test_invalid_moment_arm_count_is_rejected_at_startup():
    node = FakeNode({'moment_arms_flat': [0.0] * 10})
    with pytest.raises(ValueError, match='12 coefficients'):
        ArmProprioceptionNode(
            node,
            FakeJointState,
            FakeArray,
        )


def test_physical_parameters_are_loaded_from_node_configuration():
    node = FakeNode({
        'moment_arms_flat': [0.01, -0.02] * 6,
        'resting_lengths_m': [0.11] * 6,
        'tendon_stiffness_n_m': [900.0] * 6,
        'max_forces_n': [75.0] * 6,
    })
    implementation = ArmProprioceptionNode(
        node,
        FakeJointState,
        FakeArray,
    )
    assert np.allclose(
        implementation.model.moment_arms,
        np.array([0.01, -0.02] * 6).reshape(6, 2),
    )
    assert np.allclose(implementation.model.resting_lengths_m, 0.11)
    assert np.allclose(
        implementation.model.tendon_stiffness_n_m,
        900.0,
    )
    assert np.allclose(implementation.model.max_forces_n, 75.0)
