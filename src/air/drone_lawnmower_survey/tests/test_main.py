import contextlib
import io
import math
import os
import sys
import unittest
from unittest import mock


MODULE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if MODULE_DIR not in sys.path:
    sys.path.insert(0, MODULE_DIR)

import main as survey_main


class MainTests(unittest.TestCase):
    def test_invalid_speeds_are_rejected(self):
        for speed in (0, -1, math.nan, math.inf, -math.inf):
            with self.subTest(speed=speed):
                with self.assertRaises(ValueError):
                    survey_main.validate_speed(speed)

    def test_default_mode_only_previews_the_path(self):
        output = io.StringIO()
        with mock.patch.object(survey_main, "execute_mission") as execute_mission:
            with contextlib.redirect_stdout(output):
                result = survey_main.main([])

        self.assertEqual(result, 0)
        execute_mission.assert_not_called()
        self.assertIn("Preview only", output.getvalue())

    def test_invalid_ports_are_rejected_before_execution(self):
        for port in (0, 65536):
            with self.subTest(port=port):
                with contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as context:
                        survey_main.main(["--port", str(port), "--execute"])

                self.assertEqual(context.exception.code, 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
