"""Validation schedule and evidence checks, without ROS or CARLA."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

path = Path(__file__).resolve().parents[1] / 'tools/visual_validation/scenario.py'
spec = importlib.util.spec_from_file_location('validation_scenario', path)
scenario = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scenario)


class ValidationTests(unittest.TestCase):
    def test_target_first_then_distractor_and_two_loss_intervals(self):
        self.assertTrue(scenario.phase(0)['hide_b'])
        self.assertFalse(scenario.phase(6)['hide_b'])
        for now in (15.5, 32):
            self.assertTrue(scenario.phase(now)['hide_a'])
            self.assertFalse(scenario.phase(now)['hide_b'])
        for now in (14, 17, 36, 44):
            self.assertFalse(scenario.phase(now)['hide_a'])

    def test_output_rejects_git_checkout(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / '.git').mkdir()
            with self.assertRaises(ValueError):
                scenario.output_directory(root / 'data')
            self.assertFalse((root / 'data').exists())

    def test_atomic_protocol_and_guard_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / 'ready.json'
            scenario.write_json(p, {'epoch': 1})
            self.assertEqual(scenario.read_json(p), {'epoch': 1})
            self.assertFalse(p.with_suffix('.json.tmp').exists())
        result = scenario.summarize([
            {'status': 'tracking', 'velocity_ned': [1, 0, 0]},
            {'status': 'reselect_required', 'velocity_ned': [0, 0, 0]},
            {'status': 'target_lost', 'velocity_ned': [0, .1, 0]}],
            [{'t': 108, 'boxes': [[1, 2, 3, 4, .9, 'car']]*2}], 100)
        self.assertEqual(result['guard_nonzero'], 1)
        self.assertEqual(result['intervals']['two_visible']['car_counts'], [2])
        self.assertEqual(result['intervals']['a_hidden']['frames'], 0)
