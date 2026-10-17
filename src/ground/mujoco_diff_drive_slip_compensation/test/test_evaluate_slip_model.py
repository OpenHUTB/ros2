from pathlib import Path

from mujoco_diff_drive_slip_compensation.evaluate_slip_model import evaluate
from mujoco_diff_drive_slip_compensation.slip_model import train_model
from test_slip_model import _write_episode


def test_evaluation_uses_only_test_episodes_and_writes_scope(tmp_path: Path):
    data = tmp_path / 'episodes'
    data.mkdir()
    for split, start in (('train', 100), ('validation', 200), ('test', 300)):
        for index in range(2):
            _write_episode(data, split, start + index, count=24)
    models = tmp_path / 'models'
    train_model(data, models, window_size=6, epochs=10, seed=91)

    output = tmp_path / 'results.json'
    result = evaluate(data, models / 'slip_model.json', output)

    assert result['samples'] == 38
    assert len(result['episodes']) == 2
    assert 'held-out test-episode' in result['evaluation_scope']
    assert output.is_file()
