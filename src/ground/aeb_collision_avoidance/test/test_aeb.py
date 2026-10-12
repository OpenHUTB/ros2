import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from aeb_collision_avoidance.aeb_core import AEBConfig, AEBController, SafetyState
from aeb_collision_avoidance.simulator import run_simulation


def test_safe_state_has_no_braking():
    decision = AEBController().decide(distance=100.0, ego_speed=10.0)
    assert decision.state is SafetyState.SAFE
    assert decision.brake == 0.0


def test_warning_state_precedes_braking():
    decision = AEBController().decide(distance=14.0, ego_speed=10.0)
    assert decision.state is SafetyState.WARNING
    assert 0.0 < decision.brake < 8.0


def test_braking_state_commands_max_brake():
    decision = AEBController().decide(distance=8.0, ego_speed=10.0)
    assert decision.state is SafetyState.BRAKING
    assert decision.brake == 8.0
    assert decision.target_speed == 0.0


def test_stop_distance_has_highest_priority():
    decision = AEBController().decide(distance=0.5, ego_speed=0.2)
    assert decision.state is SafetyState.STOPPED
    assert decision.brake == 8.0


def test_required_distance_grows_with_speed():
    controller = AEBController()
    assert controller.required_stopping_distance(20.0) > controller.required_stopping_distance(10.0)


def test_braking_is_latched_once_triggered():
    controller = AEBController()
    first = controller.decide(distance=8.0, ego_speed=10.0)
    assert first.state is SafetyState.BRAKING
    # 车速下降会让 TTC 变大，此处必须保持制动，不能退回预制动
    second = controller.decide(distance=7.0, ego_speed=1.0)
    assert second.state is SafetyState.BRAKING
    assert second.brake == 8.0


def test_latch_releases_when_hazard_clears():
    controller = AEBController()
    controller.decide(distance=8.0, ego_speed=10.0)
    cleared = controller.decide(distance=200.0, ego_speed=10.0)
    assert cleared.state is SafetyState.SAFE
    assert not controller.latched


def test_latch_holds_at_low_speed_near_obstacle():
    """The release threshold must not collapse while the vehicle is nearly stopped."""
    controller = AEBController()
    controller.decide(distance=8.0, ego_speed=10.0)
    creeping = controller.decide(distance=2.0, ego_speed=0.3)
    assert creeping.state is SafetyState.BRAKING


def test_warning_state_occurs_before_braking():
    result = run_simulation()
    assert SafetyState.WARNING in result.states
    assert result.states.index(SafetyState.WARNING) < result.states.index(SafetyState.BRAKING)


def test_demo_stops_without_collision():
    result = run_simulation()
    assert not result.collision
    assert SafetyState.BRAKING in result.states
    assert result.stopped
    assert result.final_speed <= 0.1
    assert result.final_distance > 0.0


def test_high_speed_scenario_still_avoids_collision():
    result = run_simulation(initial_distance=50.0, initial_speed=15.0, duration=12.0)
    assert not result.collision
    assert result.stopped


def test_brake_command_is_not_chattering():
    """The latched controller must not toggle between braking and warning."""
    result = run_simulation()
    braking = [state is SafetyState.BRAKING for state in result.states]
    first = braking.index(True)
    # 一旦开始紧急制动，后续不应再回落到非制动状态（停车保持除外）
    assert all(braking[first:]), "brake command chattered after emergency braking started"


def test_without_braking_the_vehicle_collides():
    """Falsifiable check: with AEB disabled the same scenario must collide."""
    disabled = AEBConfig(warning_ttc=0.0, braking_ttc=0.0, stop_distance=0.0, max_brake=0.0)
    result = run_simulation(controller=AEBController(disabled), duration=8.0)
    assert result.collision
