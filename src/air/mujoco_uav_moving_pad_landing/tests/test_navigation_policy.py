from mujoco_uav_moving_pad_landing.navigation_policy import (
    MuJoCoGridPlant,
    collect_policy_samples,
    evaluation_scenarios,
    train_navigation_policy,
)


def test_navigation_policy_samples_cover_all_scenes_and_group_episodes():
    features, labels, groups = collect_policy_samples(episodes_per_scene=4)

    assert features.shape[1] == 10
    assert labels.shape[0] == groups.shape[0]
    assert len(evaluation_scenarios()) == 3
    assert len(set(groups.tolist())) == 12


def test_muJoCo_plant_moves_between_free_grid_cells():
    scenario = evaluation_scenarios()[0]
    plant = MuJoCoGridPlant(scenario)
    try:
        start = plant.pose
        measured = plant.move_to((start[0] + 1, start[1]))
        assert measured == (start[0] + 1, start[1])
    finally:
        del plant


def test_navigation_policy_trains_with_episode_isolated_holdout(tmp_path):
    result = train_navigation_policy(tmp_path / "navigation_policy.joblib", episodes_per_scene=4)

    assert result["scenes"] == 3
    assert result["train_episodes"] + result["test_episodes"] == 12
    assert 0.0 <= result["test_action_accuracy"] <= 1.0
    assert (tmp_path / "navigation_policy.joblib").is_file()
