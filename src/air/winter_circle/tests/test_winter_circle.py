"""Offline regression; mocks do not certify flight performance."""
import contextlib
import csv
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

class CircleReplay(unittest.TestCase):
    def test_half_and_full_lap_outputs_and_interval_csv(self):
        # Fake RPC runs the real main() path. This is not AirSim flight evidence.
        for laps, angles in [(0.5, [math.pi, math.pi/2, 0]),
                             (1.0, [math.pi, math.pi/2, 0, -math.pi/2, -math.pi])]:
            with self.subTest(laps=laps), tempfile.TemporaryDirectory() as d:
                def state(x, y, z, landed=1):
                    return NS(timestamp=20, landed_state=landed,
                              kinematics_estimated=NS(position=NS(x_val=x,y_val=y,z_val=z)))
                initial, startup, control, cleanup = [Mock() for _ in range(4)]
                initial.getMultirotorState.return_value = state(0,0,0,0)
                collision = NS(time_stamp=0,has_collided=False)
                initial.simGetCollisionInfo.return_value = collision
                initial.armDisarm.return_value = True
                startup.takeoffAsync.return_value.get.return_value = True
                startup.moveToZAsync.return_value.get.return_value = True
                control.getMultirotorState.side_effect = [state(0,0,-3)] + [
                    state(2+2*math.cos(a),2*math.sin(a),-3) for a in angles]
                control.simGetCollisionInfo.return_value = collision
                control.moveByVelocityZAsync.return_value.get.return_value = True
                cleanup.hoverAsync.return_value.get.return_value = True
                fake = NS(MultirotorClient=Mock(side_effect=[initial,startup,control,cleanup]),
                          LandedState=NS(Landed=0), DrivetrainType=NS(MaxDegreeOfFreedom=0),
                          YawMode=Mock())
                log = Path(d)/'replay.csv'
                output = io.StringIO()
                with patch.object(sys,'argv',['winter_circle.py','--fly','--laps',str(laps),'--log',str(log)]), patch.dict(sys.modules,{'airsim':fake}), patch.object(m,'wait_deadline',side_effect=lambda deadline: deadline+0.1), contextlib.redirect_stdout(output):
                    m.main()
                label = 'Partial arc complete' if laps < 1 else 'Circle complete'
                self.assertIn('%s: %.2f/%.2f laps' % (label,laps,laps),output.getvalue())
                if laps < 1: self.assertNotIn('Circle complete',output.getvalue())
                with log.open(newline='') as fh: rows = list(csv.DictReader(fh))
                self.assertEqual(len(rows),len(angles))
                self.assertTrue(all(float(row['loop_interval_s']) >= 0 for row in rows))
                self.assertAlmostEqual(float(rows[-1]['progress_rad']),2*math.pi*laps)
                self.assertEqual(control.moveByVelocityZAsync.call_count,len(angles)-1)
                cleanup.hoverAsync.return_value.get.assert_called_once()

if __name__ == '__main__':
    unittest.main()
