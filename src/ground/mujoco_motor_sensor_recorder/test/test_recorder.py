import csv
from pathlib import Path

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


def test_zero_torque_current_stays_near_idle_current(tmp_path):
    source = tmp_path / "state.csv"
    source.write_text(
        "time_s,speed_mps,left_temperature_c,right_temperature_c,left_torque_nm,right_torque_nm,ambient_c\n"
        "0.0,0.0,25.0,25.0,0.0,0.0,24.0\n",
        encoding="utf-8",
    )
    output = tmp_path / "sensors.csv"
    assert record_csv(source, output, seed=7) == 1
    with output.open(newline="", encoding="utf-8") as stream:
        row = next(csv.DictReader(stream))
    assert float(row["left_current_a"]) == pytest.approx(0.25, abs=0.12)
    assert float(row["right_current_a"]) == pytest.approx(0.25, abs=0.12)


def test_recorder_rejects_same_input_and_output_path(tmp_path):
    source = tmp_path / "state.csv"
    source.write_text(
        "time_s,speed_mps,left_temperature_c,right_temperature_c,left_torque_nm,right_torque_nm,ambient_c\n"
        "0.0,0.0,25.0,25.0,0.0,0.0,24.0\n",
        encoding="utf-8",
    )
    original = source.read_bytes()
    with pytest.raises(ValueError, match="must be different"):
        record_csv(source, source)
    assert source.read_bytes() == original


def test_sample_csv_has_stable_six_decimal_output(tmp_path):
    source = Path(__file__).resolve().parents[1] / "config" / "motor_state.csv"
    output = tmp_path / "samples.csv"
    assert record_csv(source, output, seed=7) == 5
    assert output.read_text(encoding="utf-8").splitlines() == [
        "time_s,speed_mps,left_temperature_c,right_temperature_c,left_current_a,right_current_a,ambient_c",
        "0.000000,0.200000,24.969294,25.161372,1.240956,2.237397,24.000000",
        "0.100000,0.240000,24.988398,25.174404,1.405588,2.044744,24.000000",
        "0.200000,0.290000,25.324425,25.429868,1.488013,1.924080,24.000000",
        "0.300000,0.310000,25.200072,25.702630,1.659144,1.825508,24.000000",
        "0.400000,0.340000,25.397036,25.590733,1.769971,1.675717,24.000000",
    ]
