"""Offline regression; mocks do not certify flight performance."""
import contextlib
import importlib.util
import io
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('winter_circle', Path(__file__).parents[1] / 'winter_circle.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class Regression(unittest.TestCase):
    def test_rpc_true(self):
        f = Mock(); f.get.return_value = True
        m.checked(f, 'Move'); f.get.assert_called_once(); f.join.assert_not_called()

    def test_rpc_false(self):
        f = Mock(); f.get.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'Move rejected'):
            m.checked(f, 'Move')

    def test_rpc_error(self):
        f = Mock(); f.get.side_effect = RuntimeError('Disconnected')
        with self.assertRaisesRegex(RuntimeError, 'Disconnected'):
            m.checked(f, 'Move')

    def test_deadline_accounts_for_work(self):
        sleeps = []
        self.assertAlmostEqual(m.wait_deadline(1, clock=lambda: 1.04, sleep=sleeps.append), 1.1)
        self.assertAlmostEqual(sleeps[0], 0.06)

    def test_overrun_has_no_extra_sleep(self):
        sleeps = []
        self.assertEqual(m.wait_deadline(1, clock=lambda: 1.27, sleep=sleeps.append), 1.27)
        self.assertEqual(sleeps, [])

    def test_preview_no_import_or_log(self):
        with tempfile.TemporaryDirectory() as d:
            log = Path(d) / 'log.csv'
            with patch.object(sys, 'argv', ['winter_circle.py', '--log', str(log)]), patch.dict(sys.modules, {'airsim': None}), contextlib.redirect_stdout(io.StringIO()):
                m.main()
            self.assertFalse(log.exists())

    def test_connection_error_names_endpoint(self):
        client = Mock(); client.confirmConnection.side_effect = OSError('refused')
        fake = NS(MultirotorClient=Mock(return_value=client))
        with patch.object(sys, 'argv', ['winter_circle.py', '--fly', '--host', '192.0.2.1']), patch.dict(sys.modules, {'airsim': fake}), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, '192.0.2.1:41451.*Play'):
                m.main()
        client.enableApiControl.assert_not_called()

    def test_velocity_cap_and_directions(self):
        for x,y in [(2,0), (0,2), (-2,0), (0,-2), (4,3)]:
            vx,vy = m.command(x,y,0,0,2,0.25,0.5,0.5)
            self.assertLessEqual(math.hypot(vx,vy), 0.5 + 1e-12)
        self.assertEqual(m.command(2,0,0,0,2,0.25,0.5,0.5), (0,0.5))

    def test_center_guard(self):
        with self.assertRaisesRegex(RuntimeError, 'center'):
            m.command(0,0,0,0,2,0.25,0.5,0.5)

    def test_collision_timestamp(self):
        m.check_collision(NS(has_collided=True,time_stamp=10,object_name='ground'),10)
        with self.assertRaisesRegex(RuntimeError, 'New collision'):
            m.check_collision(NS(has_collided=True,time_stamp=11,object_name='wall'),10)

    def test_failed_takeoff_logs_and_attempts_bounded_hover(self):
        with tempfile.TemporaryDirectory() as d:
            log = Path(d) / 'track.csv'
            initial = Mock()
            initial.getMultirotorState.return_value = NS(timestamp=10, landed_state=0, kinematics_estimated=NS(position=NS(x_val=0,y_val=0,z_val=0)))
            initial.simGetCollisionInfo.return_value = NS(time_stamp=0,has_collided=False)
            startup = Mock(); startup.takeoffAsync.return_value.get.return_value = False
            cleanup = Mock(); cleanup.hoverAsync.return_value.get.return_value = True
            fake = NS(MultirotorClient=Mock(side_effect=[initial,startup,cleanup]), LandedState=NS(Landed=0))
            with patch.object(sys,'argv',['winter_circle.py','--fly','--log',str(log)]), patch.dict(sys.modules,{'airsim':fake}), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError,'Takeoff rejected'):
                    m.main()
            self.assertEqual(fake.MultirotorClient.call_args.kwargs['timeout_value'], 3)
            cleanup.hoverAsync.return_value.get.assert_called_once()
            self.assertIn('radius_error_m',log.read_text())
            # Same path must not truncate experimental evidence or arm again.
            initial.reset_mock()
            fake.MultirotorClient.side_effect=[initial]
            initial.getMultirotorState.return_value=NS(timestamp=10,landed_state=0,kinematics_estimated=NS(position=NS(x_val=0,y_val=0,z_val=0)))
            initial.simGetCollisionInfo.return_value=NS(time_stamp=0,has_collided=False)
            with patch.object(sys,'argv',['winter_circle.py','--fly','--log',str(log)]), patch.dict(sys.modules,{'airsim':fake}), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(FileExistsError):
                    m.main()
            initial.enableApiControl.assert_not_called()

if __name__ == '__main__':
    unittest.main()
