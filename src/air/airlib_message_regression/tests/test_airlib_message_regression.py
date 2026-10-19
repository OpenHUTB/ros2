import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from airlib_message_regression.main import DEFAULT_INPUT, validate_dataset, validate_message


class AirLibMessageRegressionTests(unittest.TestCase):
    def setUp(self):
        self.messages = json.loads(DEFAULT_INPUT.read_text(encoding="utf-8"))["messages"]

    def test_sample_messages_pass(self):
        count, failures = validate_dataset(self.messages)
        self.assertEqual(count, 4)
        self.assertEqual(failures, [])

    def test_invalid_imu_quaternion_is_rejected(self):
        message = copy.deepcopy(self.messages[0])
        message["orientation"] = [1.0, 0.0, 0.0, 1.0]
        errors = validate_message(message)
        self.assertTrue(any("quaternion norm" in error for error in errors))

    def test_imu_non_list_vector_is_reported_without_exception(self):
        message = copy.deepcopy(self.messages[0])
        message["linear_acceleration"] = "down"
        errors = validate_message(message)
        self.assertTrue(any("must contain 3 numbers" in error for error in errors))

    def test_invalid_gps_range_is_rejected(self):
        message = copy.deepcopy(self.messages[1])
        message["latitude"] = 91.0
        errors = validate_message(message)
        self.assertTrue(any("latitude" in error for error in errors))

    def test_lidar_payload_requires_flat_xyzi_points(self):
        message = copy.deepcopy(self.messages[2])
        message["points"] = [1.0, 2.0, 3.0]
        errors = validate_message(message)
        self.assertTrue(any("four values per point" in error for error in errors))

    def test_invalid_lidar_count_is_reported_without_exception(self):
        message = copy.deepcopy(self.messages[2])
        message["fields"][0]["count"] = "one"
        errors = validate_message(message)
        self.assertTrue(any("count must be a positive integer" in error for error in errors))

    def test_wrong_lidar_field_name_is_rejected(self):
        message = copy.deepcopy(self.messages[2])
        message["fields"][3]["name"] = "reflectivity"
        errors = validate_message(message)
        self.assertTrue(any("x/y/z/intensity" in error for error in errors))

    def test_non_unit_odometry_quaternion_is_rejected(self):
        message = copy.deepcopy(self.messages[3])
        message["orientation"] = [1.0, 0.0, 0.0, 1.0]
        errors = validate_message(message)
        self.assertTrue(any("quaternion norm" in error for error in errors))

    def test_boolean_timestamp_is_rejected(self):
        message = copy.deepcopy(self.messages[0])
        message["timestamp_ns"] = True
        errors = validate_message(message)
        self.assertTrue(any("timestamp_ns" in error for error in errors))

    def test_empty_main_input_is_rejected(self):
        import io
        import tempfile
        from contextlib import redirect_stdout
        from airlib_message_regression.main import main

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "empty.json"
            path.write_text('{"messages": []}', encoding="utf-8")
            output = io.StringIO()
            with redirect_stdout(output):
                exit_code = main(["--input", str(path)])
        self.assertEqual(exit_code, 1)
        self.assertIn("non-empty list", output.getvalue())

    def test_non_list_messages_is_rejected_without_exception(self):
        import io
        import tempfile
        from contextlib import redirect_stdout
        from airlib_message_regression.main import main

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text('{"messages": {"type": "imu"}}', encoding="utf-8")
            output = io.StringIO()
            with redirect_stdout(output):
                exit_code = main(["--input", str(path)])
        self.assertEqual(exit_code, 1)
        self.assertIn("non-empty list", output.getvalue())


if __name__ == "__main__":
    unittest.main(verbosity=2)
