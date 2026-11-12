#!/usr/bin/env python3
"""Check the actual offline entry point and its required safety dependency."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'


class TestEvalPolicy(unittest.TestCase):
    def test_self_test_runs_safety_checks(self):
        run = subprocess.run([sys.executable, str(SCRIPT_DIR / 'eval_policy.py'), '--self-test'],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
        self.assertEqual(run.returncode, 0, run.stderr.decode('utf-8', errors='replace'))
        self.assertIn('self-test ok', run.stdout.decode('utf-8', errors='replace'))
        self.assertNotIn('跳过', run.stdout.decode('utf-8', errors='replace'))

    def test_missing_safety_module_fails_instead_of_reporting_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name in ('eval_policy.py', 'lidar_features.py'):
                shutil.copyfile(str(SCRIPT_DIR / name), os.path.join(tmp, name))
            env = os.environ.copy()
            env.pop('PYTHONPATH', None)
            run = subprocess.run([sys.executable, os.path.join(tmp, 'eval_policy.py'), '--self-test'],
                                 cwd=tmp, env=env, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, timeout=30)
            self.assertNotEqual(run.returncode, 0)
            self.assertIn('safety_shield', run.stderr.decode('utf-8', errors='replace'))


if __name__ == '__main__':
    unittest.main()
