"""以可手算误差和异常时间验证统计边界。"""
import numpy as np
import pytest
from uav_hover_log_eval.metrics import load_log, summarize


def test_hand_computed_error_and_short_episode():
    result = summarize(np.array([[0., 2., 1.], [0.02, 0., 1.]]))
    assert result['rmse_m'] == 1.0
    assert result['mae_m'] == 1.0
    assert not result['complete_8s_episode']
    assert not result['tail_within_tolerance']
    assert result['recovery_s'] is None


def test_recovery_requires_continuous_hold():
    times = np.arange(0, 8.001, 0.02)
    error = np.where(times < 3.4, 0.3, 0.)
    result = summarize(np.column_stack((times, 1.5 + error, np.full(len(times), 1.5))))
    assert result['recovery_s'] == pytest.approx(0.4)
    assert result['complete_8s_episode']
    assert result['tail_within_tolerance']


def test_invalid_csv_and_time(tmp_path):
    path = tmp_path / 'bad.csv'
    path.write_text('time_s,height_m,target_m\n0,1,1\n0,nan,1\n', encoding='utf-8')
    with pytest.raises(ValueError):
        load_log(path)
    with pytest.raises(ValueError):
        summarize(np.array([[1., 1., 1.], [0., 1., 1.]]))
