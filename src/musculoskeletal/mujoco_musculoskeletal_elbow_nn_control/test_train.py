import csv

from mujoco_musculoskeletal_elbow_nn_control.train import train_model


def test_train_model_on_episode_csvs(tmp_path):
    fields = ["episode_id", "target_angle_rad", "angle_rad", "angular_velocity_rad_s", "flexor_force_n", "extensor_force_n", "flexor_activation", "extensor_activation"]
    for index in range(5):
        path = tmp_path / f"episode_{index}.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
            for step in range(8):
                writer.writerow({"episode_id": f"episode_{index}", "target_angle_rad": 0.8 + index * 0.01, "angle_rad": 0.7 + index * 0.01, "angular_velocity_rad_s": 0.1, "flexor_force_n": 1.0, "extensor_force_n": 0.5, "flexor_activation": 0.3, "extensor_activation": 0.1})
    metrics = train_model(tmp_path, tmp_path / "model.json", seed=7)
    assert set(metrics["partitions"]) == {"train", "validation", "test"}
    assert (tmp_path / "model.json").exists() and (tmp_path / "model.metrics.json").exists()
