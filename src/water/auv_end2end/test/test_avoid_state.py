"""避障状态机的单元测试，不依赖 ROS 与 Gazebo."""

from auv_end2end.avoid_logic import (
    DIR_LEFT,
    DIR_NONE,
    DIR_RIGHT,
    AvoidStateMachine,
)


def make_fsm(**overrides):
    """按默认参数构造状态机，可局部覆盖."""
    params = dict(
        trigger_ratio=0.22,
        recover_ratio=0.18,
        max_avoid_duration=3.0,
        cooldown=2.0,
        min_avoid_duration=0.3,
        emergency_factor=1.5,
    )
    params.update(overrides)
    return AvoidStateMachine(**params)


def test_enter_avoid_to_the_right():
    """左侧占比更高时应向右避让."""
    fsm = make_fsm()
    out = fsm.update(0.0, 0.40, 0.05)
    assert out.is_avoiding is True
    assert out.direction == DIR_RIGHT
    assert out.event == AvoidStateMachine.EVENT_ENTER_RIGHT


def test_enter_avoid_to_the_left():
    """右侧占比更高时应向左避让."""
    fsm = make_fsm()
    out = fsm.update(0.0, 0.05, 0.40)
    assert out.is_avoiding is True
    assert out.direction == DIR_LEFT
    assert out.event == AvoidStateMachine.EVENT_ENTER_LEFT


def test_hysteresis_band_keeps_avoiding():
    """占比落在滞环带内不应切换状态."""
    fsm = make_fsm()
    fsm.update(0.0, 0.40, 0.05)
    out = fsm.update(0.5, 0.20, 0.20)
    assert out.is_avoiding is True
    assert out.event is None


def test_min_avoid_duration_prevents_early_recovery():
    """刚进入避让就变亮时，最小避让时长内仍保持避让."""
    fsm = make_fsm()
    fsm.update(0.0, 0.40, 0.05)
    early = fsm.update(0.1, 0.02, 0.02)
    assert early.is_avoiding is True
    assert early.event is None

    late = fsm.update(0.4, 0.02, 0.02)
    assert late.is_avoiding is False
    assert late.event == AvoidStateMachine.EVENT_CLEARED


def test_clear_after_obstacle_gone():
    """障碍消失且超过最小避让时长后恢复直行."""
    fsm = make_fsm()
    fsm.update(0.0, 0.40, 0.05)
    out = fsm.update(1.0, 0.02, 0.02)
    assert out.is_avoiding is False
    assert out.is_cooldown is False
    assert out.event == AvoidStateMachine.EVENT_CLEARED


def test_timeout_enters_cooldown():
    """持续避让超过上限时进入冷却期."""
    fsm = make_fsm()
    fsm.update(0.0, 0.40, 0.05)
    out = fsm.update(3.1, 0.40, 0.40)
    assert out.is_avoiding is False
    assert out.is_cooldown is True
    assert out.direction == DIR_NONE
    assert out.event == AvoidStateMachine.EVENT_TIMEOUT


def test_cooldown_blocks_retrigger_and_then_ends():
    """冷却期内低于紧急阈值的障碍不应触发避让，冷却结束后恢复检测."""
    fsm = make_fsm()
    fsm.update(0.0, 0.40, 0.05)
    fsm.update(3.1, 0.40, 0.40)

    blocked = fsm.update(3.5, 0.30, 0.30)
    assert blocked.is_avoiding is False
    assert blocked.is_cooldown is True
    assert blocked.event is None

    ended = fsm.update(5.2, 0.10, 0.10)
    assert ended.is_cooldown is False
    assert ended.event == AvoidStateMachine.EVENT_COOLDOWN_END


def test_emergency_retrigger_during_cooldown():
    """冷却期内画面显著恶化时应立即重新避让."""
    fsm = make_fsm()
    fsm.update(0.0, 0.40, 0.05)
    fsm.update(3.1, 0.40, 0.40)

    # 触发阈值 0.22，紧急阈值 0.33
    out = fsm.update(3.5, 0.70, 0.70)
    assert out.is_avoiding is True
    assert out.is_cooldown is False
    assert out.event == AvoidStateMachine.EVENT_EMERGENCY


def test_no_trigger_below_threshold():
    """占比低于触发阈值时保持直行."""
    fsm = make_fsm()
    out = fsm.update(0.0, 0.20, 0.15)
    assert out.is_avoiding is False
    assert out.event is None
