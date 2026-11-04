import json
import os
import tempfile
import unittest
from concurrent.futures import Future

from uav_mission_safety_manager.controller import FlightController
from uav_mission_safety_manager.flight_recorder import FlightRecorder
from uav_mission_safety_manager.state_machine import SafetyMachine, State


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeAdapter:
    def __init__(self):
        self.calls = []
        self.reads = []

    def submit(self, command, **data):
        future = Future()
        self.calls.append((command, data, future))
        return future

    def sample(self):
        future = Future()
        self.reads.append(future)
        return future


class TestFlightRecorder(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.recorder = FlightRecorder(max_events=5, clock=self.clock)

    def test_ring_buffer_drops_oldest(self):
        for index in range(7):
            self.recorder.record('command', 'event_%d' % index)
        self.assertEqual(len(self.recorder.events), 5)
        self.assertEqual(self.recorder.dropped, 2)
        self.assertEqual([e['detail'] for e in self.recorder.events],
                         ['event_2', 'event_3', 'event_4', 'event_5', 'event_6'])
        self.assertEqual([e['seq'] for e in self.recorder.events], [2, 3, 4, 5, 6])

    def test_rejects_invalid_capacity(self):
        with self.assertRaises(ValueError):
            FlightRecorder(max_events=0)

    def test_state_change_and_failsafe_reason(self):
        self.recorder.state_changed(State.IDLE, State.CONNECTING, 'connect')
        self.clock.now = 1.5
        self.recorder.state_changed(State.CONNECTING, State.FAILSAFE, 'failsafe',
                                    'altitude 60.0m exceeds limit')
        kinds = [e['kind'] for e in self.recorder.events]
        self.assertEqual(kinds, ['state', 'state', 'failsafe'])
        self.assertEqual(self.recorder.events[1]['state'], 'FAILSAFE')
        self.assertIn('altitude 60.0m exceeds limit', self.recorder.events[1]['detail'])
        summary = self.recorder.summary()
        self.assertEqual(summary['failsafe_reasons'], ['altitude 60.0m exceeds limit'])
        self.assertEqual(summary['final_state'], 'FAILSAFE')

    def test_state_durations_follow_clock(self):
        self.recorder.state_changed(State.IDLE, State.ARMING, 'connected')
        self.clock.now = 2.0
        self.recorder.state_changed(State.ARMING, State.TAKEOFF, 'armed')
        self.clock.now = 5.0
        self.recorder.state_changed(State.TAKEOFF, State.EXECUTING, 'airborne')
        summary = self.recorder.summary(timestamp=5.0)
        self.assertEqual(summary['state_durations_s']['ARMING'], 2.0)
        self.assertEqual(summary['state_durations_s']['TAKEOFF'], 3.0)
        self.assertEqual(summary['kinds'], {'state': 3})

    def test_export_csv_and_json(self):
        self.recorder.state_changed(State.IDLE, State.CONNECTING, 'connect')
        self.recorder.record('mission_loaded', 'waypoints=2', state='CONNECTING')
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = self.recorder.write_csv(os.path.join(tmp, 'events.csv'))
            json_path = self.recorder.write_json(os.path.join(tmp, 'events.json'))
            with open(csv_path, encoding='utf-8') as handle:
                lines = [line.rstrip('\n') for line in handle]
            self.assertEqual(lines[0].split(','), FlightRecorder.CSV_HEADER)
            self.assertEqual(len(lines), 3)
            self.assertEqual(lines[1].split(',')[2], 'state')
            with open(json_path, encoding='utf-8') as handle:
                payload = json.load(handle)
            self.assertEqual(payload['summary']['events'], 2)
            self.assertEqual(len(payload['events']), 2)

    def test_state_machine_callback_records_transitions(self):
        machine = SafetyMachine(max_altitude=10, on_transition=self.recorder.state_changed)
        machine.transition('connect')
        machine.check_safety(11, 0)
        states = [e['state'] for e in self.recorder.events if e['kind'] == 'state']
        self.assertEqual(states, ['CONNECTING', 'FAILSAFE'])
        machine.transition('land')
        machine.transition('landed')
        self.assertEqual(self.recorder.events[-1]['state'], 'LANDED')

    def test_controller_records_mission_milestones(self):
        adapter = FakeAdapter()
        recorder = FlightRecorder(max_events=50, clock=self.clock)
        controller = FlightController(adapter, clock=self.clock, recorder=recorder)
        controller.connected = True
        controller.home = (0.0, 0.0, 0.0)
        controller.position = (0.0, 0.0, 0.0)
        controller.machine.state = State.EXECUTING
        controller.request('mission', waypoints=[{'x': 1.0, 'y': 0.0, 'z': -2.0}])
        self.assertIn('mission_loaded', [e['kind'] for e in recorder.events])
        # Complete the dispatched goto RPC before cancelling.
        adapter.calls[-1][2].set_result('ok')
        controller.tick()
        controller.request('cancel')
        self.assertIn('mission_canceled', [e['kind'] for e in recorder.events])


if __name__ == '__main__':
    unittest.main()
