"""Offline CLI regression tests; fixtures are synthetic, not flight evidence."""
import csv
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'plot_tracking.py'

class LogSelectionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.logs = self.root / 'logs'
        self.logs.mkdir()
        self.out = self.root / 'plots'

    def log(self, path):
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['t', 'x_ref', 'y_ref', 'z_ref', 'x', 'y', 'z', 'dist', 'phase'])
            writer.writerows([[0, 1, 0, 1, 0, 0, 1, 1, 'track'],
                              [1, 1, 0, 1, 1, 0, 1, 0, 'settle']])
        return path

    def run_plot(self, *extra):
        return subprocess.run([sys.executable, str(SCRIPT), '--log-dir', str(self.logs),
                               '--out', str(self.out)] + list(extra),
                              capture_output=True, text=True, timeout=60)

    def test_explicit_log_works_without_discovered_logs(self):
        path = self.log(self.root / 'custom.csv')
        result = self.run_plot('--pid-log', str(path))
        self.assertEqual(result.returncode, 0, result.stderr)
        for name in ['trajectory_rectangle.png', 'error_rectangle.png']:
            self.assertTrue((self.out / name).is_file(), result.stdout)

    def test_missing_explicit_log_does_not_fall_back(self):
        self.log(self.logs / 'pid_rectangle_kp1.00_ki0.05_kd0.20.csv')
        result = self.run_plot('--pid-log', str(self.root / 'missing.csv'))
        self.assertEqual(result.returncode, 2)
        self.assertIn('PID log does not exist', result.stderr)
        self.assertEqual(list(self.out.glob('*.png')), [])

    def test_automatic_discovery_still_plots(self):
        path = self.log(self.logs / 'pid_rectangle_kp1.00_ki0.05_kd0.20.csv')
        result = self.run_plot()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('pid log : ' + str(path), result.stdout)
        self.assertTrue((self.out / 'trajectory_rectangle.png').is_file())

    def test_no_logs_reports_failure(self):
        result = self.run_plot()
        self.assertEqual(result.returncode, 2)
        self.assertIn('no pid log found', result.stderr)

    def test_metrics_file_is_not_selected_as_trajectory(self):
        (self.logs / 'pid_rectangle_kp1.00_ki0.05_kd0.20_metrics.csv').write_text(
            'trajectory_rms_all_m,0.1\n', encoding='utf-8')
        result = self.run_plot()
        self.assertEqual(result.returncode, 2)
        self.assertIn('no pid log found', result.stderr)

    def test_explicit_log_overrides_discovered_log(self):
        self.log(self.logs / 'pid_rectangle_kp1.00_ki0.05_kd0.20.csv')
        path = self.log(self.root / 'chosen.csv')
        result = self.run_plot('--pid-log', str(path))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('pid log : ' + str(path), result.stdout)

if __name__ == '__main__':
    unittest.main()
