from pathlib import Path
import json
import os
import sys

import numpy as np
import pytest

from mujoco_forklift_pallet_docking import policy as policy_module
from mujoco_forklift_pallet_docking.policy import ImagePolicy, teacher_action


def _simulation_or_skip():
    pytest.importorskip("mujoco")
    from mujoco_forklift_pallet_docking.simulation import ForkliftDockingSimulation

    try:
        sim = ForkliftDockingSimulation()
    except (ImportError, RuntimeError) as exc:
        message = str(exc).lower()
        if any(token in message for token in ("egl", "gl", "render", "display", "opengl")):
            pytest.skip(f"MuJoCo rendering backend unavailable: {exc}")
        raise
    return sim


def test_policy_returns_bounded_throttle_and_brake():
    output = ImagePolicy().predict(np.zeros((2, 64, 64), dtype=np.uint8))
    assert output.shape == (2, 2)
    assert np.all((output >= 0.0) & (output <= 1.0))


def test_teacher_brakes_before_stopping_distance():
    assert teacher_action(0.0, 2.0)[1] == 1.0
    assert teacher_action(5.0, 2.0)[0] == 1.0
    assert np.array_equal(teacher_action(0.10, 2.0), [0.0, 1.0])
    assert np.array_equal(teacher_action(0.100001, 2.0), [1.0, 0.0])


def test_camera_observes_pallet_from_different_distances():
    far = _simulation_or_skip()
    near = _simulation_or_skip()
    far.data.qpos[0], near.data.qpos[0] = -5.0, -3.0
    import mujoco
    mujoco.mj_forward(far.model, far.data)
    mujoco.mj_forward(near.model, near.data)
    try:
        assert not np.array_equal(far.image(), near.image())
    finally:
        far.close()
        near.close()


def test_manual_keys_drive_the_physics_stepper():
    sim = _simulation_or_skip()
    sim.data.qvel[0] = 0.0
    try:
        initial_x = sim.x
        assert sim.manual_step("w")
        assert sim.steps == 1
        assert sim.data.ctrl[0] == 5.0
        for _ in range(20):
            sim.manual_step("w")
        assert sim.x > initial_x
        assert sim.manual_step("s")
        assert sim.data.ctrl[0] == -10.0
        for _ in range(40):
            sim.manual_step("s")
        assert sim.speed >= 0.0
        assert sim.data.ctrl[0] == 0.0
        assert sim.manual_step(" ")
        assert sim.data.ctrl[0] == 0.0
        assert not sim.manual_step("q")
        assert sim.steps == 63
    finally:
        sim.close()


def test_packaged_policy_checkpoint_round_trip(tmp_path: Path):
    policy = ImagePolicy(seed=21)
    images = np.zeros((1, 64, 64), dtype=np.uint8)
    checkpoint = Path(policy_module.__file__).parent / "models" / "pallet_docking_policy.npz"
    loaded = ImagePolicy.load(str(checkpoint))
    output = loaded.predict(images)
    assert output.shape == (1, 2)
    assert np.all(np.isfinite(output))
    assert np.all((output >= 0.0) & (output <= 1.0))

    saved = tmp_path / "policy.npz"
    policy.save(str(saved))
    restored = ImagePolicy.load(str(saved))
    assert np.array_equal(policy.predict(images), restored.predict(images))


def test_training_improves_over_untrained_policy(tmp_path):
    rng = np.random.default_rng(4)
    # Separate the classes by a clear margin so this tests gradient learning
    # without depending on the convergence path near a brightness threshold.
    levels = np.concatenate((
        rng.integers(0, 49, 32, dtype=np.uint8),
        rng.integers(207, 256, 32, dtype=np.uint8),
    ))
    images = np.broadcast_to(levels[:, None, None], (64, 64, 64)).copy()
    labels = levels < 128
    actions = np.stack((labels, ~labels), axis=1).astype(float)
    policy = ImagePolicy(seed=3)
    before = np.mean((policy.predict(images) - actions) ** 2)
    policy.train(images, actions, epochs=2000, lr=0.03)
    after = np.mean((policy.predict(images) - actions) ** 2)
    assert after < before * 0.7
    checkpoint = tmp_path / "trained.npz"
    policy.save(str(checkpoint))
    assert np.array_equal(policy.predict(images), ImagePolicy.load(str(checkpoint)).predict(images))


def test_evaluate_reports_safe_docking_metrics_from_multiple_starts(monkeypatch, capsys):
    pytest.importorskip("mujoco")
    # Probe renderer availability narrowly; policy and metric assertions still fail normally.
    probe = _simulation_or_skip()
    probe.close()
    import main
    brake_scale = os.environ.get("FORKLIFT_TEST_BRAKE_SCALE", "1.0")
    monkeypatch.setattr(sys, "argv", ["main.py", "--mode", "evaluate", "--brake-scale", brake_scale])
    main.main()
    report = json.loads(capsys.readouterr().out)
    assert report["brake_scale"] == float(brake_scale)
    results = report["scenarios"]
    assert [result["start_x_m"] for result in results] == [-5.0, -4.0, -3.0]
    for result in results:
        assert 0.0 < result["insertion_depth_m"] <= 0.5
        assert result["final_speed_mps"] <= 0.025
        assert result["over_inserted"] is False


def test_packaged_policy_beats_untrained_on_camera_samples():
    probe = _simulation_or_skip()
    probe.close()
    from mujoco_forklift_pallet_docking.simulation import collect_training_data

    images, actions = collect_training_data(samples=200, seed=99)
    checkpoint = Path(policy_module.__file__).parent / "models" / "pallet_docking_policy.npz"
    trained = ImagePolicy.load(str(checkpoint))
    untrained = ImagePolicy(seed=17)
    trained_mse = np.mean((trained.predict(images) - actions) ** 2)
    untrained_mse = np.mean((untrained.predict(images) - actions) ** 2)
    assert trained_mse < 0.8 * untrained_mse
