"""Unit tests for the pure spiral-search logic (no ROS required)."""
import math

import pytest

from uav_spiral_search.spiral_search import Settings, SpiralSearchController


def make_controller(**overrides):
    return SpiralSearchController(Settings(**overrides))


def test_settings_rejects_invalid_values():
    with pytest.raises(ValueError):
        Settings(max_radius=1.0, start_radius=2.0)
    with pytest.raises(ValueError):
        Settings(confirm_frames=0)
    with pytest.raises(ValueError):
        Settings(confirm_frames=2.5)
    with pytest.raises(ValueError):
        Settings(search_height=3.0)
    with pytest.raises(ValueError):
        Settings(hover_height=0.0)
    with pytest.raises(ValueError):
        Settings(cruise_speed=3.0, max_speed=2.0)
    with pytest.raises(ValueError):
        Settings(spiral_spacing=float('nan'))


def test_waits_for_fresh_odometry():
    controller = make_controller()
    command, state, report = controller.step(1.0)
    assert command == (0.0, 0.0, 0.0)
    assert state == 'odom_stale'


def test_climbs_to_search_height_before_searching():
    controller = make_controller(search_height=-3.0, climb_tolerance=0.3)
    controller.update_pose(1.0, 2.0, -1.0, 0.5, 1.0)
    command, state, _ = controller.step(1.0)
    assert state == 'climbing'
    # NED：目标高度更负（更高），垂直速度必须为负。
    assert command[0] == 0.0 and command[1] == 0.0 and command[2] < 0.0

    controller.update_pose(1.0, 2.0, -2.9, 0.5, 1.05)
    _, state, _ = controller.step(1.05)
    assert state == 'searching'
    assert controller.origin == (1.0, 2.0)


def test_spiral_expands_monotonically_until_exhausted():
    controller = make_controller(
        search_height=-3.0, spiral_spacing=2.0, start_radius=2.0,
        max_radius=6.0, cruise_speed=1.5, max_speed=2.0)
    now = 0.0
    controller.update_pose(0.0, 0.0, -3.0, 0.0, now)
    _, state, _ = controller.step(now)
    assert state == 'searching'

    radii = []
    for _ in range(6000):
        now += 0.05
        # 理想跟踪：把无人机直接放到当前参考点上。
        if controller.reference is not None:
            ref_x, ref_y = controller.reference
            controller.update_pose(ref_x, ref_y, -3.0, 0.0, now)
        else:
            controller.update_pose(0.0, 0.0, -3.0, 0.0, now)
        command, state, report = controller.step(now)
        if state != 'searching':
            break
        radii.append(controller.radius(controller.theta))
        assert math.hypot(*command[:2]) <= 2.0 + 1e-9

    assert state == 'exhausted'
    assert radii == sorted(radii), '搜索半径必须单调外扩'
    assert report['radius_m'] >= 6.0


def test_reference_only_advances_when_drone_keeps_up():
    controller = make_controller(
        search_height=-3.0, start_radius=2.0, spiral_spacing=4.0,
        max_radius=30.0, cruise_speed=2.0, max_tracking_error=2.5)
    now = 0.0
    controller.update_pose(0.0, 0.0, -3.0, 0.0, now)
    controller.step(now)

    # 无人机停在原地不动：参考点超前达到上限后不再推进。
    theta_before = None
    for _ in range(400):
        now += 0.05
        controller.update_pose(0.0, 0.0, -3.0, 0.0, now)
        controller.step(now)
        if theta_before is not None and controller.theta <= theta_before:
            break
        theta_before = controller.theta
    ref_x, ref_y = controller.reference
    assert math.hypot(ref_x, ref_y) <= 2.5 + 2.0 * 0.1 + 1e-6


def test_locks_after_consecutive_detections():
    controller = make_controller(
        search_height=-3.0, hover_height=-5.0,
        assumed_target_distance=10.0, confirm_frames=3,
        position_gain=1.2, max_speed=2.0)
    now = 0.0
    controller.update_pose(0.0, 0.0, -3.0, 0.0, now)
    controller.step(now)

    controller.update_detection(True, now)
    controller.update_detection(True, now)
    _, state, _ = controller.step(now + 0.05)
    assert state == 'searching', '两帧不足以确认目标'

    controller.update_detection(True, now + 0.1)
    command, state, report = controller.step(now + 0.1)
    assert state == 'locked'
    # 位置 (0,0)、航向 0、检测框居中 (cx=0)、假设距离 10 m -> 目标估计在正北 10 m 处。
    assert report['target_estimate_ned'] == [10.0, 0.0]
    # 悬停点在 (10, 0, -5)，无人机从 (0,0) 出发，水平速度饱和。
    assert command[0] == pytest.approx(2.0)
    assert report['search_time_s'] == pytest.approx(0.1, abs=0.05)


def test_detection_offset_corrects_bearing_estimate():
    controller = make_controller(
        search_height=-3.0, assumed_target_distance=10.0,
        confirm_frames=1, camera_half_fov=math.pi / 4)
    now = 0.0
    controller.update_pose(0.0, 0.0, -3.0, 0.0, now)
    controller.step(now)
    # 目标在画面右缘 (cx=1)：方位应偏东半视场角 45°。
    controller.update_detection(True, now, cx=1.0)
    _, state, report = controller.step(now + 0.05)
    assert state == 'locked'
    ex, ey = report['target_estimate_ned']
    # 报告中的估计值保留两位小数。
    assert ex == pytest.approx(10.0 * math.cos(math.pi / 4), abs=0.01)
    assert ey == pytest.approx(10.0 * math.sin(math.pi / 4), abs=0.01)


def test_single_frame_loss_resets_confirmation():
    controller = make_controller(confirm_frames=3)
    now = 0.0
    controller.update_pose(0.0, 0.0, -3.0, 0.0, now)
    controller.step(now)
    controller.update_detection(True, now)
    controller.update_detection(True, now)
    controller.update_detection(False, now)
    assert controller.confirm_count == 0
    controller.update_detection(True, now)
    _, state, _ = controller.step(now + 0.05)
    assert state == 'searching'


def test_stale_detection_does_not_count_as_consecutive():
    controller = make_controller(confirm_frames=3, detection_timeout=1.0)
    now = 0.0
    controller.update_pose(0.0, 0.0, -3.0, 0.0, now)
    controller.step(now)
    controller.update_detection(True, now)
    # 图像流中断超过 detection_timeout：确认计数必须被清零。
    later = now + 1.2
    controller.update_pose(0.0, 0.0, -3.0, 0.0, later)
    controller.step(later)
    assert controller.confirm_count == 0


def test_report_contains_mission_metrics():
    controller = make_controller(search_height=-3.0)
    now = 0.0
    controller.update_pose(0.0, 0.0, -3.0, 0.0, now)
    _, _, report = controller.step(now)
    for key in ('state', 'search_time_s', 'radius_m', 'path_len_m',
                'confirm_frames', 'target_estimate_ned', 'velocity_ned'):
        assert key in report
    assert report['radius_m'] == pytest.approx(2.0)
