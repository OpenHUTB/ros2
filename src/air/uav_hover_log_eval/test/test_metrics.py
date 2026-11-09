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
    assert result['error_left_tolerance']
    assert result['disturbance_peak_m'] == pytest.approx(0.3)


def test_tolerance_changes_recovery_and_flag():
    times = np.arange(0, 8.001, 0.02)
    error = np.where(times < 3.4, 0.3, 0.)
    data = np.column_stack((times, 1.5 + error, np.full(len(times), 1.5)))
    loose = summarize(data, tolerance=0.35)
    assert not loose['error_left_tolerance']
    assert loose['recovery_s'] == pytest.approx(0.0)
    assert loose['tail_ratio'] == 1.0


def test_gap_breaks_complete_episode():
    data = np.array([[0., 1.5, 1.5], [0.02, 1.5, 1.5], [8., 1.5, 1.5]])
    assert not summarize(data)['complete_8s_episode']


def test_invalid_csv_and_time(tmp_path):
    path = tmp_path / 'bad.csv'
    path.write_text('time_s,height_m,target_m\n0,1,1\n0,nan,1\n', encoding='utf-8')
    with pytest.raises(ValueError):
        load_log(path)
    with pytest.raises(ValueError):
        summarize(np.array([[1., 1., 1.], [0., 1., 1.]]))


@pytest.mark.parametrize('times', [[0.02, 8.0], list(range(9))])
def test_sparse_logs_do_not_prove_coverage(times):
    data = np.column_stack((times, np.full(len(times), 1.5), np.full(len(times), 1.5)))
    result = summarize(data)
    assert not result['complete_8s_episode']
    assert not result['tail_covered']
    assert not result['tail_within_tolerance']
    assert result['recovery_s'] is None


def test_missing_tail_samples_break_coverage_and_restart_hold():
    times = np.arange(0, 8.001, 0.02)
    times = times[(times <= 7.2) | (times >= 7.8)]
    data = np.column_stack((times, np.full(len(times), 1.5), np.full(len(times), 1.5)))
    result = summarize(data, disturbance_end=7.0)
    assert not result['tail_covered']
    assert result['tail_ratio'] == 1.0
    assert not result['tail_within_tolerance']
    assert not result['complete_8s_episode']
    assert result['recovery_s'] is None


def test_max_gap_can_match_a_slower_sampling_rate():
    times = np.arange(0, 8.001, 0.1)
    data = np.column_stack((times, np.full(len(times), 1.5), np.full(len(times), 1.5)))
    result = summarize(data, max_gap_s=0.11)
    assert result['complete_8s_episode']
    assert result['tail_covered']
    assert result['recovery_s'] == pytest.approx(0.0)
    for invalid in (0.0, -1.0, float('nan')):
        with pytest.raises(ValueError):
            summarize(data, max_gap_s=invalid)
