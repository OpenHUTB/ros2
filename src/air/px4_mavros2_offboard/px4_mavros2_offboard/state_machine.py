"""Pure state machine for a guarded PX4 Offboard takeoff-hover-land demo."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


IDLE = "IDLE"
PRESTREAM = "PRESTREAM"
REQUEST_OFFBOARD = "REQUEST_OFFBOARD"
ARMING = "ARMING"
TAKEOFF = "TAKEOFF"
HOVER = "HOVER"
LANDING = "LANDING"
FAILSAFE = "FAILSAFE"
DONE = "DONE"
ABORTED = "ABORTED"
TERMINAL_STATES = {DONE, ABORTED}
ACTIVE_STATES = {
    PRESTREAM,
    REQUEST_OFFBOARD,
    ARMING,
    TAKEOFF,
    HOVER,
    LANDING,
    FAILSAFE,
}


@dataclass(frozen=True)
class OffboardTelemetry:
    """Current MAVROS state and local position used for one state-machine step."""

    now: float
    connected: bool
    mode: str
    armed: bool
    altitude: float
    state_age: float


@dataclass(frozen=True)
class OffboardStep:
    """Commands that the ROS node should execute for this tick."""

    state: str
    send_setpoint: bool = False
    request_offboard: bool = False
    request_arm: bool = False
    request_land: bool = False
    done: bool = False
    reason: str = ""
    target: Optional[Tuple[float, float, float]] = None


class OffboardStateMachine:
    """Run prestream, OFFBOARD, arm, takeoff, hover and land in that order."""

    def __init__(
        self,
        altitude: float = 3.0,
        prestream_seconds: float = 2.0,
        offboard_timeout: float = 10.0,
        arm_timeout: float = 8.0,
        takeoff_timeout: float = 25.0,
        hover_seconds: float = 10.0,
        landing_timeout: float = 30.0,
        state_timeout: float = 1.5,
        altitude_tolerance: float = 0.3,
    ) -> None:
        if altitude <= 0.0:
            raise ValueError("altitude must be positive")
        if min(
            prestream_seconds,
            offboard_timeout,
            arm_timeout,
            takeoff_timeout,
            landing_timeout,
            state_timeout,
        ) <= 0.0:
            raise ValueError("timeouts must be positive")
        self.altitude = float(altitude)
        self.prestream_seconds = float(prestream_seconds)
        self.offboard_timeout = float(offboard_timeout)
        self.arm_timeout = float(arm_timeout)
        self.takeoff_timeout = float(takeoff_timeout)
        self.hover_seconds = float(hover_seconds)
        self.landing_timeout = float(landing_timeout)
        self.state_timeout = float(state_timeout)
        self.altitude_tolerance = float(altitude_tolerance)
        self.state = IDLE
        self.reason = ""
        self.start_position: Optional[Tuple[float, float, float]] = None
        self.stage_started = 0.0

    @property
    def target(self) -> Optional[Tuple[float, float, float]]:
        if self.start_position is None:
            return None
        return (
            self.start_position[0],
            self.start_position[1],
            self.start_position[2] + self.altitude,
        )

    @property
    def active(self) -> bool:
        return self.state in ACTIVE_STATES

    def start(
        self,
        now: float,
        position: Tuple[float, float, float],
    ) -> bool:
        """Start a new sequence from a measured local ENU position."""
        if self.active:
            return False
        if len(position) != 3 or not all(math.isfinite(value) for value in position):
            raise ValueError("start position must contain three finite values")
        self.start_position = tuple(float(value) for value in position)
        self.stage_started = float(now)
        self.state = PRESTREAM
        self.reason = "starting setpoint prestream"
        return True

    def abort(self, now: float, reason: str = "operator abort") -> bool:
        """Enter failsafe from an active sequence."""
        if not self.active or self.state == FAILSAFE:
            return False
        self._enter(FAILSAFE, now, reason)
        return True

    def step(self, telemetry: OffboardTelemetry) -> OffboardStep:
        """Advance the state machine and return commands for this tick."""
        if self.state in TERMINAL_STATES or self.state == IDLE:
            return OffboardStep(
                state=self.state,
                done=self.state in TERMINAL_STATES,
                reason=self.reason,
                target=self.target,
            )

        if not telemetry.connected or telemetry.state_age > self.state_timeout:
            if self.state != FAILSAFE:
                self._enter(
                    FAILSAFE,
                    telemetry.now,
                    "MAVROS state is missing or stale",
                )
            return self._failsafe_step(telemetry)

        mode = telemetry.mode.upper()
        if self.state == PRESTREAM:
            if telemetry.now - self.stage_started >= self.prestream_seconds:
                self._enter(
                    REQUEST_OFFBOARD,
                    telemetry.now,
                    "setpoint prestream complete",
                )
            else:
                return self._step(telemetry, send_setpoint=True)

        if self.state == REQUEST_OFFBOARD:
            if mode == "OFFBOARD":
                self._enter(ARMING, telemetry.now, "PX4 entered OFFBOARD")
            elif telemetry.now - self.stage_started > self.offboard_timeout:
                self._enter(FAILSAFE, telemetry.now, "OFFBOARD mode timeout")
                return self._failsafe_step(telemetry)
            else:
                return self._step(
                    telemetry,
                    send_setpoint=True,
                    request_offboard=True,
                )

        if self.state == ARMING:
            if telemetry.armed:
                self._enter(TAKEOFF, telemetry.now, "vehicle armed")
            elif telemetry.now - self.stage_started > self.arm_timeout:
                self._enter(FAILSAFE, telemetry.now, "arming timeout")
                return self._failsafe_step(telemetry)
            else:
                return self._step(
                    telemetry,
                    send_setpoint=True,
                    request_arm=True,
                )

        target = self.target
        if self.state == TAKEOFF:
            if target is not None and (
                telemetry.altitude >= target[2] - self.altitude_tolerance
            ):
                self._enter(HOVER, telemetry.now, "takeoff altitude reached")
                return self._step(telemetry, send_setpoint=True)
            if telemetry.now - self.stage_started > self.takeoff_timeout:
                self._enter(FAILSAFE, telemetry.now, "takeoff timeout")
                return self._failsafe_step(telemetry)
            return self._step(telemetry, send_setpoint=True)

        if self.state == HOVER:
            if telemetry.now - self.stage_started >= self.hover_seconds:
                self._enter(LANDING, telemetry.now, "hover time completed")
                return self._step(
                    telemetry,
                    send_setpoint=mode != "AUTO.LAND",
                    request_land=True,
                )
            return self._step(telemetry, send_setpoint=True)

        if self.state == LANDING:
            if self._landed(telemetry):
                self._enter(DONE, telemetry.now, "landing completed")
                return OffboardStep(
                    state=DONE,
                    done=True,
                    reason=self.reason,
                    target=target,
                )
            if telemetry.now - self.stage_started > self.landing_timeout:
                self._enter(FAILSAFE, telemetry.now, "landing timeout")
                return self._failsafe_step(telemetry)
            return self._step(
                telemetry,
                send_setpoint=mode != "AUTO.LAND",
                request_land=True,
            )

        return self._failsafe_step(telemetry)

    def _failsafe_step(self, telemetry: OffboardTelemetry) -> OffboardStep:
        if self._landed(telemetry):
            self._enter(ABORTED, telemetry.now, self.reason or "failsafe landed")
            return OffboardStep(
                state=ABORTED,
                done=True,
                reason=self.reason,
                target=self.target,
            )
        mode = telemetry.mode.upper()
        return self._step(
            telemetry,
            send_setpoint=telemetry.connected and mode != "AUTO.LAND",
            request_land=telemetry.connected,
        )

    def _landed(self, telemetry: OffboardTelemetry) -> bool:
        if self.start_position is None:
            return False
        return (
            not telemetry.armed
            and telemetry.altitude
            <= self.start_position[2] + self.altitude_tolerance
        )

    def _enter(self, state: str, now: float, reason: str) -> None:
        self.state = state
        self.stage_started = float(now)
        self.reason = reason

    def _step(
        self,
        telemetry: OffboardTelemetry,
        send_setpoint: bool = False,
        request_offboard: bool = False,
        request_arm: bool = False,
        request_land: bool = False,
    ) -> OffboardStep:
        del telemetry
        return OffboardStep(
            state=self.state,
            send_setpoint=send_setpoint,
            request_offboard=request_offboard,
            request_arm=request_arm,
            request_land=request_land,
            reason=self.reason,
            target=self.target,
        )
