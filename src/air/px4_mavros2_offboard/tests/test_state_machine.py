from px4_mavros2_offboard.state_machine import (
    ABORTED,
    ARMING,
    DONE,
    FAILSAFE,
    HOVER,
    LANDING,
    PRESTREAM,
    REQUEST_OFFBOARD,
    TAKEOFF,
    OffboardStateMachine,
    OffboardTelemetry,
)


def telemetry(now, mode="OFFBOARD", armed=True, altitude=0.0, age=0.1):
    return OffboardTelemetry(
        now=now,
        connected=True,
        mode=mode,
        armed=armed,
        altitude=altitude,
        state_age=age,
    )


def test_sequence_requires_setpoint_prestream():
    machine = OffboardStateMachine(altitude=3.0, prestream_seconds=2.0)
    assert machine.start(0.0, (0.0, 0.0, 0.0))
    step = machine.step(telemetry(1.0, mode="MANUAL", armed=False))
    assert step.state == PRESTREAM
    assert step.send_setpoint
    assert not step.request_offboard
    step = machine.step(telemetry(2.1, mode="MANUAL", armed=False))
    assert step.state == REQUEST_OFFBOARD
    assert step.request_offboard


def test_mode_and_arm_are_confirmed_before_takeoff():
    machine = OffboardStateMachine(altitude=3.0, prestream_seconds=0.1)
    machine.start(0.0, (0.0, 0.0, 0.0))
    step = machine.step(telemetry(0.2, mode="OFFBOARD", armed=False))
    assert step.state == ARMING
    assert step.request_arm
    step = machine.step(telemetry(0.3, mode="OFFBOARD", armed=True))
    assert step.state == TAKEOFF


def test_full_takeoff_hover_land_sequence():
    machine = OffboardStateMachine(
        altitude=2.0,
        prestream_seconds=0.1,
        hover_seconds=1.0,
    )
    machine.start(0.0, (1.0, 2.0, 0.0))
    machine.step(telemetry(0.2, mode="OFFBOARD", armed=False))
    machine.step(telemetry(0.3, mode="OFFBOARD", armed=True))
    step = machine.step(telemetry(1.0, altitude=1.9))
    assert step.state == HOVER
    step = machine.step(telemetry(2.1, altitude=2.0))
    assert step.state == LANDING
    assert step.request_land
    step = machine.step(telemetry(4.0, mode="AUTO.LAND", armed=False, altitude=0.1))
    assert step.state == DONE
    assert step.done


def test_stale_mavros_state_enters_failsafe():
    machine = OffboardStateMachine(state_timeout=1.0)
    machine.start(0.0, (0.0, 0.0, 0.0))
    step = machine.step(telemetry(2.0, age=2.0))
    assert step.state == FAILSAFE
    assert step.request_land


def test_operator_abort_requests_land_and_ends_after_disarm():
    machine = OffboardStateMachine()
    machine.start(0.0, (0.0, 0.0, 0.0))
    assert machine.abort(1.0)
    step = machine.step(telemetry(1.1, armed=True, altitude=1.0))
    assert step.state == FAILSAFE
    assert step.request_land
    step = machine.step(
        telemetry(2.0, mode="AUTO.LAND", armed=False, altitude=0.1)
    )
    assert step.state == ABORTED
    assert step.done


def test_start_is_rejected_while_sequence_is_active():
    machine = OffboardStateMachine()
    assert machine.start(0.0, (0.0, 0.0, 0.0))
    assert not machine.start(1.0, (1.0, 1.0, 1.0))
