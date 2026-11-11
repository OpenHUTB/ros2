import csv

import pytest

from mujoco_motor_sensor_recorder.recorder import record_csv


def test_record_csv_is_reproducible_and_standalone(tmp_path):
    source = tmp_path / "state.csv"
    source.write_text(
        "time_s,speed_mps,left_temperature_c,right_temperature_c,left_torque_nm,right_torque_nm,ambient_c\n"
        "0.0,0.2,25.0,25.1,0.18,-0.36,24.0\n"
        "0.1,0.3,25.2,25.3,0.0,0.18,24.0\n",
        encoding="utf-8",
    )
    first, second = tmp_path / "one.csv", tmp_path / "two.csv"
    assert record_csv(source, first, seed=5) == 2
    assert record_csv(source, second, seed=5) == 2
    assert first.read_bytes() == second.read_bytes()
    with first.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 2
    assert set(rows[0]) == {"time_s", "speed_mps", "left_temperature_c", "right_temperature_c", "left_current_a", "right_current_a", "ambient_c"}


def test_recorder_rejects_unrelated_csv(tmp_path):
    source = tmp_path / "wrong.csv"
    source.write_text("x,y\n1,2\n", encoding="utf-8")
    with pytest.raises(ValueError, match="input CSV"):
        record_csv(source, tmp_path / "out.csv")
