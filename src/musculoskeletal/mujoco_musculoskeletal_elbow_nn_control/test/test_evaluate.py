import csv

from mujoco_musculoskeletal_elbow_nn_control.evaluate import evaluate
from mujoco_musculoskeletal_elbow_nn_control.mlp import ActivationMLP


def test_grouped_evaluation_preserves_trajectory_and_load(tmp_path):
    fields = ["episode_id", "target_angle_rad", "angle_rad", "angular_velocity_rad_s", "flexor_force_n", "extensor_force_n", "flexor_activation", "extensor_activation", "load_kg"]
    path = tmp_path / "sine_load_0p5.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        writer.writerow({"episode_id":"sine_load_0p5", "target_angle_rad":1, "angle_rad":.9, "angular_velocity_rad_s":0, "flexor_force_n":1, "extensor_force_n":.5, "flexor_activation":.3, "extensor_activation":.2, "load_kg":.5})
    model = ActivationMLP(seed=1); model.save(tmp_path / "model.json")
    report = evaluate(tmp_path, tmp_path / "model.json")
    assert report["sine_load_0p5"]["trajectory"] == "sine" and report["sine_load_0p5"]["load_kg"] == .5
