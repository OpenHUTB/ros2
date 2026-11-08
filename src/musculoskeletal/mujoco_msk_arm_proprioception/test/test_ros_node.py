from types import SimpleNamespace

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
    def __init__(self):
        self.parameters = {}
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
