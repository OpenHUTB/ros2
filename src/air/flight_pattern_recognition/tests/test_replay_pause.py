"""A paused VM must not turn a recorded flight into a DDS queue-flooding burst."""
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

try:
    from ros_pattern import Replay
except ImportError:
    Replay = None


@unittest.skipUnless(Replay is not None, 'Requires the sourced ROS2 environment')
class ReplayPauseTest(unittest.TestCase):
    def test_long_pause_keeps_timestamps_and_resumes_without_burst(self):
        sent = []
        replay = SimpleNamespace(finish_time=None, start=0., i=0, done=False,
            scheduled=[(0., {'timestamp_ns': 10}), (.05, {'timestamp_ns': 20}),
                       (.1, {'timestamp_ns': 30})],
            pub=SimpleNamespace(publish=lambda msg: sent.append(json.loads(msg.data))),
            status=SimpleNamespace(publish=lambda msg: None))
        with patch('ros_pattern.time.monotonic', return_value=600.):
            Replay.tick(replay)
        self.assertEqual(sent, [{'timestamp_ns': 10}])
        with patch('ros_pattern.time.monotonic', return_value=600.06):
            Replay.tick(replay)
        self.assertEqual(sent, [{'timestamp_ns': 10}, {'timestamp_ns': 20}])
        self.assertFalse(replay.done)
