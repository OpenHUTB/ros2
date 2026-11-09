import unittest
from uav_mission_safety_manager.state_machine import SafetyMachine, State

class TestSafetyMachine(unittest.TestCase):
    def test_mission_lifecycle(self):
        m = SafetyMachine();
        for event in ('connect','connected','armed','airborne'): m.transition(event)
        self.assertEqual(m.state, State.EXECUTING)
        m.transition('return'); m.transition('at_home'); m.transition('landed')
        self.assertEqual(m.state, State.LANDED)
    def test_safety(self):
        m = SafetyMachine(max_altitude=10); m.transition('connect')
        m.check_safety(11, 0); self.assertEqual(m.state, State.FAILSAFE)

if __name__ == '__main__': unittest.main()
