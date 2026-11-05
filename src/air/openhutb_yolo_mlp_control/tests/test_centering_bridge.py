"""Exercise actual bridge stop and watchdog callbacks without an RPC server."""
from types import SimpleNamespace
from unittest.mock import Mock
import threading
import unittest

import numpy as np
from geometry_msgs.msg import Twist
from openhutb_yolo_mlp_control.airsim_bridge_node import AirSimBridgeNode


class BridgeTests(unittest.TestCase):
    def bridge(self, hold=True):
        return SimpleNamespace(
            enable_api_control=True, hold_heading=hold, _command_epoch=0,
            _takeoff_future=None, _cmd_lock=threading.Lock(),
            _filtered_cmd=np.ones(3), _latest_cmd=(1, 1, 1),
            _accumulated_delta=np.ones(3), _cmd_event=threading.Event(),
            _first_cmd_logged=True, command_smoothing=0.35,
            _pose_control=False, command_duration=0.05,
            _last_cmd_time=0, command_timeout=0.5,
        )

    def test_zero_clears_filtered_and_queued_motion(self):
        b = self.bridge()
        AirSimBridgeNode.cmd_vel_callback(b, Twist())
        np.testing.assert_array_equal(b._accumulated_delta, np.zeros(3))
        self.assertEqual(b._latest_cmd, (0, 0, 0))
        self.assertTrue(b._cmd_event.is_set())

    def test_nonfinite_command_becomes_stop(self):
        b = self.bridge()
        msg = Twist()
        msg.linear.y = float('nan')
        AirSimBridgeNode.cmd_vel_callback(b, msg)
        self.assertEqual(b._latest_cmd, (0, 0, 0))

    def test_repeated_zero_does_not_starve_hold_dispatch(self):
        b = self.bridge()
        AirSimBridgeNode.cmd_vel_callback(b, Twist())
        epoch = b._command_epoch
        AirSimBridgeNode.cmd_vel_callback(b, Twist())
        self.assertEqual(b._command_epoch, epoch)

    def test_watchdog_requests_native_hold(self):
        b = self.bridge()
        AirSimBridgeNode.command_watchdog(b)
        self.assertEqual(b._latest_cmd, (0, 0, 0))
        self.assertTrue(b._cmd_event.is_set())

    def test_existing_trajectory_watchdog_keeps_behavior(self):
        b = self.bridge(False)
        AirSimBridgeNode.command_watchdog(b)
        self.assertIsNone(b._latest_cmd)

    def test_worker_native_hold_keeps_heading(self):
        b = self.bridge()
        b._stop_worker = Mock()
        b._stop_worker.is_set.side_effect = [False, False, True]
        b._latest_cmd = (0, 0, 0)
        b._accumulated_delta[:] = 0
        b._cmd_event.set()
        b.control_client = Mock()
        p = SimpleNamespace(x_val=1, y_val=2, z_val=-3)
        b.control_client.getMultirotorState.return_value = SimpleNamespace(
            kinematics_estimated=SimpleNamespace(position=p))
        b.get_logger = Mock()
        b.min_dispatch_distance = 0
        b.max_dispatch_distance = 0.1
        b.min_position_speed = 0.1
        b.max_position_speed = 1
        b._heading_degrees = 90
        AirSimBridgeNode._control_worker_loop(b)
        args, kwargs = b.control_client.moveToPositionAsync.call_args
        self.assertEqual(args[:3], (1, 2, -3))
        self.assertFalse(kwargs['yaw_mode'].is_rate)
        self.assertEqual(kwargs['yaw_mode'].yaw_or_rate, 90)

    def test_slow_pose_worker_consumes_all_accumulated_displacement(self):
        b = self.bridge()
        b._pose_control = True
        b._stop_worker = Mock()
        b._stop_worker.is_set.side_effect = [False, False, True]
        b._latest_cmd = (0, 0.4, 0)
        b._accumulated_delta[:] = (0, 0.08, 0)
        b.control_client = Mock()
        p = SimpleNamespace(x_val=0, y_val=0, z_val=-3)
        b.control_client.getMultirotorState.return_value = SimpleNamespace(
            kinematics_estimated=SimpleNamespace(position=p))
        b.control_client.simGetVehiclePose.return_value = SimpleNamespace(
            position=SimpleNamespace(x_val=0, y_val=0, z_val=-3))
        b.get_logger = Mock()
        b.min_dispatch_distance = 0
        b.max_dispatch_distance = 0.1
        b.min_position_speed = 0.1
        b.max_position_speed = 1
        AirSimBridgeNode._control_worker_loop(b)
        pose = b.control_client.simSetVehiclePose.call_args[0][0]
        self.assertAlmostEqual(pose.position.y_val, 0.08)
        np.testing.assert_allclose(b._accumulated_delta, np.zeros(3))

    def test_blocked_capture_does_not_block_commands_and_coalesces_requests(self):
        b = self.bridge()
        b._stop_worker = threading.Event()
        b._image_event = threading.Event()
        entered, release = threading.Event(), threading.Event()
        calls = []

        def slow_capture():
            calls.append(1)
            entered.set()
            release.wait(timeout=2)

        b.publish_image = slow_capture
        worker = threading.Thread(target=AirSimBridgeNode._image_worker_loop,
                                  args=(b,), daemon=True)
        worker.start()
        try:
            AirSimBridgeNode.request_image(b)
            self.assertTrue(entered.wait(timeout=1))
            for _ in range(100):
                AirSimBridgeNode.request_image(b)
            # Exercise the actual velocity callback while capture is blocked.
            msg = Twist()
            msg.linear.y = 0.8
            AirSimBridgeNode.cmd_vel_callback(b, msg)
            self.assertGreater(b._latest_cmd[1], 0)
            self.assertTrue(b._cmd_event.is_set())
            AirSimBridgeNode.cmd_vel_callback(b, Twist())
            self.assertEqual(b._latest_cmd, (0, 0, 0))
            self.assertEqual(len(calls), 1)
            b._stop_worker.set()
        finally:
            release.set()
            b._stop_worker.set()
            b._image_event.set()
            worker.join(timeout=2)
        self.assertFalse(worker.is_alive())



if __name__ == '__main__':
    unittest.main()
