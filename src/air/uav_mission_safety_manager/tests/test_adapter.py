from threading import Event, get_ident
from concurrent.futures import Future
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from uav_mission_safety_manager.airsim_adapter import AirSimAdapter


class TestAdapter(unittest.TestCase):
    def test_cancellation_timeout_never_dispatches_landing(self):
        adapter = AirSimAdapter()
        active = Future()
        adapter._active = active
        adapter._flight_started = Event()
        adapter._flight_started.set()
        calls = []
        client = SimpleNamespace(cancelLastTask=lambda **kw: calls.append('cancel'))
        try:
            with patch.object(adapter, '_client', return_value=client), patch(
                    'uav_mission_safety_manager.airsim_adapter.wait',
                    return_value=SimpleNamespace(not_done={active})):
                with self.assertRaisesRegex(RuntimeError, 'did not finish cancellation'):
                    adapter.submit('land').result(timeout=2)
            self.assertEqual(calls, ['cancel'])
            self.assertIs(adapter._active, active)
        finally:
            active.set_result(False)
            adapter.close()

    def test_cancelled_move_finishes_before_landing_starts(self):
        moving, cancelled, completed = Event(), Event(), Event()
        order = []

        class Client:
            def __init__(self, **kwargs):
                self.owner = get_ident()

            def moveToPositionAsync(self, *args, **kwargs):
                assert get_ident() == self.owner
                class Completion:
                    def get(inner):
                        moving.set()
                        if not cancelled.wait(3):
                            raise RuntimeError('movement was never cancelled')
                        order.append('move_finished')
                        completed.set()
                        return False
                return Completion()

            def cancelLastTask(self, **kwargs):
                order.append('cancel')
                cancelled.set()

            def landAsync(self, **kwargs):
                assert get_ident() == self.owner
                assert completed.is_set(), 'land raced the previous flight'
                order.append('land')
                return SimpleNamespace(get=lambda: True)

        fake = SimpleNamespace(MultirotorClient=Client)
        adapter = AirSimAdapter()
        with patch.dict('sys.modules', airsim=fake):
            try:
                move = adapter.submit('goto', target=(0, 0, -5), speed=0.5).result(timeout=2)
                self.assertTrue(moving.wait(2))
                land = adapter.submit('land').result(timeout=2)
                self.assertTrue(land.result(timeout=2))
                self.assertIs(move.result(), False)
                self.assertEqual(order, ['cancel', 'move_finished', 'land'])
            finally:
                cancelled.set()
                adapter.close()

    def test_rpc_command_and_telemetry_use_independent_workers(self):
        entered, release = Event(), Event()
        clients = []

        class Client:
            def __init__(self, **kwargs):
                clients.append(self)

            def cancelLastTask(self, **kwargs):
                pass

            def landAsync(self, **kwargs):
                entered.set()
                if not release.wait(3):
                    raise RuntimeError('test did not release RPC')
                class NoJoin:
                    def get(self):
                        return True
                    def join(self):
                        raise AssertionError('Flight completion must not block RPC worker')
                return NoJoin()

            def getMultirotorState(self, **kwargs):
                return SimpleNamespace(landed_state=0, kinematics_estimated=SimpleNamespace(
                    position=SimpleNamespace(x_val=0, y_val=0, z_val=-5),
                    linear_velocity=SimpleNamespace(x_val=0, y_val=0, z_val=0)))

        fake = SimpleNamespace(MultirotorClient=Client, LandedState=SimpleNamespace(Landed=1))
        adapter = AirSimAdapter()
        with patch.dict('sys.modules', airsim=fake):
            try:
                command = adapter.submit('land')
                self.assertTrue(entered.wait(2))
                data = adapter.sample().result(timeout=2)
                self.assertEqual(data['position'], (0., 0., -5.))
                self.assertFalse(command.result(timeout=2).done())
                release.set()
                self.assertTrue(command.result(timeout=2).result(timeout=2))
            finally:
                release.set()
                adapter.close()

    def test_landing_wait_does_not_block_telemetry_or_disarm_worker(self):
        entered, release = Event(), Event()
        calls = []

        class Client:
            def __init__(self, **kwargs):
                self.owner = get_ident()

            def cancelLastTask(self, **kwargs):
                pass

            def landAsync(self, **kwargs):
                assert get_ident() == self.owner
                assert kwargs['timeout_sec'] == 120
                class Completion:
                    def get(inner):
                        entered.set()
                        if not release.wait(3):
                            raise RuntimeError('test did not release landing')
                        return True
                return Completion()

            def armDisarm(self, armed, vehicle_name=''):
                assert get_ident() == self.owner
                calls.append(armed)
                return True

            def getMultirotorState(self, **kwargs):
                assert get_ident() == self.owner
                return SimpleNamespace(landed_state=1, kinematics_estimated=SimpleNamespace(
                    position=SimpleNamespace(x_val=0, y_val=0, z_val=0),
                    linear_velocity=SimpleNamespace(x_val=0, y_val=0, z_val=0)))

        fake = SimpleNamespace(MultirotorClient=Client, LandedState=SimpleNamespace(Landed=0))
        adapter = AirSimAdapter()
        with patch.dict('sys.modules', airsim=fake):
            try:
                flight = adapter.submit('land').result(timeout=2)
                self.assertTrue(entered.wait(2))
                self.assertFalse(flight.done())
                self.assertEqual(adapter.sample().result(timeout=2)['velocity'], (0., 0., 0.))
                adapter.submit('disarm').result(timeout=2)
                self.assertEqual(calls, [False])
                release.set()
                self.assertTrue(flight.result(timeout=2))
            finally:
                release.set()
                adapter.close()


if __name__ == '__main__':
    unittest.main()
