"""Deterministic patrol and fire-response state machine.

This module is intentionally free of ROS imports. The node supplies clock,
pose, alert and bounding-box observations, and receives body-frame velocity
commands that can be published to ``geometry_msgs/Twist``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum

from .pose_utils import normalize_angle


class ExplorerState(str, Enum):
    PATROL = 'PATROL'
    CONFIRM = 'CONFIRM'
    EVIDENCE = 'EVIDENCE'
    RETURN_HOME = 'RETURN_HOME'
    HOVER = 'HOVER'
    LAND = 'LAND'


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


@dataclass
class DetectionInfo:
    center_x: float = 0.5
    center_y: float = 0.5
    confidence: float = 0.0
    visible: bool = False


@dataclass
class ExplorerCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_z: float = 0.0
    state: ExplorerState = ExplorerState.PATROL
    land: bool = False
    waypoint_index: int = 0
    reason: str = ''


@dataclass
class ExplorerConfig:
    pattern: str = 'lawnmower'
    search_width: float = 12.0
    search_height: float = 8.0
    lane_spacing: float = 2.5
    spiral_turns: float = 3.0
    search_altitude: float = 3.0
    patrol_speed: float = 0.8
    return_speed: float = 1.0
    vertical_speed: float = 0.4
    max_yaw_rate: float = 0.6
    yaw_gain: float = 1.2
    waypoint_tolerance: float = 0.8
    confirm_hover_sec: float = 2.0
    alert_grace_sec: float = 3.0
    evidence_sec: float = 12.0
    evidence_yaw_rate: float = 0.5
    return_home_tolerance: float = 0.6
    max_mission_sec: float = 300.0
    land_after_return: bool = True
    home_x: float = 0.0
    home_y: float = 0.0
    home_z: float = 0.0


class ExplorerCore:
    """Run patrol, confirmation, evidence collection and return-to-home."""

    def __init__(self, config: ExplorerConfig | None = None) -> None:
        self.config = config or ExplorerConfig()
        self.state = ExplorerState.PATROL
        self.waypoints: list[Pose2D] = []
        self._waypoint_index = 0
        self._confirm_since: float | None = None
        self._alert_lost_since: float | None = None
        self._evidence_since: float | None = None
        self._mission_start: float | None = None
        self._land_dispatched = False
        self._rebuild_waypoints()

    def set_home(self, x: float, y: float, z: float) -> None:
        self.config.home_x = float(x)
        self.config.home_y = float(y)
        self.config.home_z = float(z)
        self._waypoint_index = 0
        self._rebuild_waypoints()

    def update(
        self,
        now: float,
        pose: Pose2D,
        alert: bool,
        detection: DetectionInfo | None = None,
        abort: bool = False,
    ) -> ExplorerCommand:
        if self._mission_start is None:
            self._mission_start = float(now)
        detection = detection or DetectionInfo()

        if abort and self.state not in (
            ExplorerState.RETURN_HOME,
            ExplorerState.LAND,
            ExplorerState.HOVER,
        ):
            self.state = ExplorerState.RETURN_HOME
            return self._return_home_command(pose, 'operator abort')

        if (
            self.config.max_mission_sec > 0.0
            and self.state
            in (
                ExplorerState.PATROL,
                ExplorerState.CONFIRM,
                ExplorerState.EVIDENCE,
            )
            and now - self._mission_start >= self.config.max_mission_sec
        ):
            self.state = ExplorerState.RETURN_HOME
            return self._return_home_command(pose, 'mission time limit reached')

        if self.state == ExplorerState.PATROL:
            if alert:
                self.state = ExplorerState.CONFIRM
                self._confirm_since = float(now)
                self._alert_lost_since = None
                return self._hover_to_fire(pose, detection, 'fire alert received')
            return self._patrol_command(pose)

        if self.state == ExplorerState.CONFIRM:
            return self._confirm_command(now, pose, alert, detection)

        if self.state == ExplorerState.EVIDENCE:
            return self._evidence_command(now, pose)

        if self.state == ExplorerState.RETURN_HOME:
            return self._return_home_command(pose)

        return self._terminal_command()

    def _confirm_command(
        self,
        now: float,
        pose: Pose2D,
        alert: bool,
        detection: DetectionInfo,
    ) -> ExplorerCommand:
        if alert:
            self._alert_lost_since = None
        else:
            if self._alert_lost_since is None:
                self._alert_lost_since = float(now)
            elif (
                now - self._alert_lost_since >= self.config.alert_grace_sec
            ):
                self.state = ExplorerState.PATROL
                self._confirm_since = None
                self._alert_lost_since = None
                return self._patrol_command(pose)

        if (
            alert
            and self._confirm_since is not None
            and now - self._confirm_since >= self.config.confirm_hover_sec
        ):
            self.state = ExplorerState.EVIDENCE
            self._evidence_since = float(now)
            return self._evidence_command(now, pose)

        return self._hover_to_fire(pose, detection, 'hover confirmation')

    def _evidence_command(
        self, now: float, pose: Pose2D
    ) -> ExplorerCommand:
        if (
            self._evidence_since is not None
            and now - self._evidence_since >= self.config.evidence_sec
        ):
            self.state = ExplorerState.RETURN_HOME
            return self._return_home_command(pose, 'evidence collection finished')

        return ExplorerCommand(
            linear_z=self._altitude_correction(pose),
            angular_z=self.config.evidence_yaw_rate,
            state=self.state,
            waypoint_index=self._waypoint_index,
            reason='orbiting and recording evidence',
        )

    def _patrol_command(self, pose: Pose2D) -> ExplorerCommand:
        if not self.waypoints:
            return ExplorerCommand(
                linear_z=self._altitude_correction(pose),
                state=self.state,
                waypoint_index=self._waypoint_index,
                reason='no patrol waypoints',
            )

        waypoint = self.waypoints[self._waypoint_index]
        dx = waypoint.x - pose.x
        dy = waypoint.y - pose.y
        distance = math.hypot(dx, dy)

        if distance <= self.config.waypoint_tolerance:
            self._waypoint_index = (
                self._waypoint_index + 1
            ) % len(self.waypoints)
            waypoint = self.waypoints[self._waypoint_index]
            dx = waypoint.x - pose.x
            dy = waypoint.y - pose.y
            distance = math.hypot(dx, dy)

        if distance < 1e-6:
            return ExplorerCommand(
                linear_z=self._altitude_correction(pose),
                state=self.state,
                waypoint_index=self._waypoint_index,
                reason='waypoint reached',
            )

        desired_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - pose.yaw)
        speed = min(
            self.config.patrol_speed,
            max(0.15, 0.8 * distance),
        )

        if abs(heading_error) > 1.0:
            linear_x = linear_y = 0.0
            angular_z = math.copysign(
                self.config.max_yaw_rate * 0.7, heading_error
            )
            reason = 'turning towards patrol waypoint'
        else:
            world_vx = speed * math.cos(desired_yaw)
            world_vy = speed * math.sin(desired_yaw)
            linear_x, linear_y = self._world_to_body(
                world_vx, world_vy, pose.yaw
            )
            angular_z = max(
                -self.config.max_yaw_rate,
                min(
                    self.config.max_yaw_rate,
                    heading_error * self.config.yaw_gain,
                ),
            )
            reason = 'following patrol pattern'

        return ExplorerCommand(
            linear_x=linear_x,
            linear_y=linear_y,
            linear_z=self._altitude_correction(pose),
            angular_z=angular_z,
            state=self.state,
            waypoint_index=self._waypoint_index,
            reason=reason,
        )

    def _hover_to_fire(
        self,
        pose: Pose2D,
        detection: DetectionInfo,
        reason: str,
    ) -> ExplorerCommand:
        angular_z = 0.0
        if detection.visible:
            angular_z = (0.5 - detection.center_x) * 2.0
            angular_z = max(
                -self.config.max_yaw_rate,
                min(self.config.max_yaw_rate, angular_z),
            )
        return ExplorerCommand(
            linear_z=self._altitude_correction(pose),
            angular_z=angular_z,
            state=self.state,
            waypoint_index=self._waypoint_index,
            reason=reason,
        )

    def _return_home_command(
        self, pose: Pose2D, reason: str = 'returning home'
    ) -> ExplorerCommand:
        dx = self.config.home_x - pose.x
        dy = self.config.home_y - pose.y
        distance = math.hypot(dx, dy)

        if distance <= self.config.return_home_tolerance:
            if self.config.land_after_return:
                self.state = ExplorerState.LAND
                land = not self._land_dispatched
                self._land_dispatched = True
                return ExplorerCommand(
                    state=self.state,
                    waypoint_index=self._waypoint_index,
                    land=land,
                    reason='home reached, landing',
                )
            self.state = ExplorerState.HOVER
            return ExplorerCommand(
                state=self.state,
                waypoint_index=self._waypoint_index,
                reason='home reached, hovering',
            )

        desired_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - pose.yaw)
        speed = min(
            self.config.return_speed,
            max(0.2, 0.8 * distance),
        )

        if abs(heading_error) > 1.0:
            linear_x = linear_y = 0.0
            angular_z = math.copysign(
                self.config.max_yaw_rate * 0.8, heading_error
            )
        else:
            world_vx = speed * math.cos(desired_yaw)
            world_vy = speed * math.sin(desired_yaw)
            linear_x, linear_y = self._world_to_body(
                world_vx, world_vy, pose.yaw
            )
            angular_z = max(
                -self.config.max_yaw_rate,
                min(
                    self.config.max_yaw_rate,
                    heading_error * self.config.yaw_gain,
                ),
            )

        return ExplorerCommand(
            linear_x=linear_x,
            linear_y=linear_y,
            linear_z=self._vertical_correction(
                self.config.home_z - pose.z
            ),
            angular_z=angular_z,
            state=self.state,
            waypoint_index=self._waypoint_index,
            reason=reason,
        )

    def _terminal_command(self) -> ExplorerCommand:
        return ExplorerCommand(
            state=self.state,
            waypoint_index=self._waypoint_index,
            reason='mission complete',
        )

    def _altitude_correction(self, pose: Pose2D) -> float:
        target_z = self.config.home_z + self.config.search_altitude
        return self._vertical_correction(target_z - pose.z)

    def _vertical_correction(self, error: float) -> float:
        return max(
            -self.config.vertical_speed,
            min(self.config.vertical_speed, 0.8 * error),
        )

    @staticmethod
    def _world_to_body(
        world_vx: float, world_vy: float, yaw: float
    ) -> tuple[float, float]:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_vx = world_vx * cos_yaw + world_vy * sin_yaw
        body_vy = -world_vx * sin_yaw + world_vy * cos_yaw
        return body_vx, body_vy

    def _rebuild_waypoints(self) -> None:
        pattern = self.config.pattern.strip().lower()
        if pattern == 'perimeter':
            points = self._perimeter_points()
        elif pattern == 'spiral':
            points = self._spiral_points()
        else:
            points = self._lawnmower_points()

        target_z = self.config.home_z + self.config.search_altitude
        self.waypoints = [
            Pose2D(x=self.config.home_x + x, y=self.config.home_y + y, z=target_z)
            for x, y in points
        ]
        if not self.waypoints:
            self.waypoints = [
                Pose2D(
                    x=self.config.home_x,
                    y=self.config.home_y,
                    z=target_z,
                )
            ]
        if self._waypoint_index >= len(self.waypoints):
            self._waypoint_index = 0

    def _lawnmower_points(self) -> list[tuple[float, float]]:
        width = max(1.0, self.config.search_width)
        height = max(1.0, self.config.search_height)
        spacing = max(0.5, self.config.lane_spacing)
        rows = max(1, math.ceil(height / spacing))
        points: list[tuple[float, float]] = []
        for row in range(rows + 1):
            ratio = row / float(rows)
            y = -height / 2.0 + height * ratio
            if row % 2 == 0:
                points.append((-width / 2.0, y))
                points.append((width / 2.0, y))
            else:
                points.append((width / 2.0, y))
                points.append((-width / 2.0, y))
        return points

    def _perimeter_points(self) -> list[tuple[float, float]]:
        half_w = max(0.5, self.config.search_width / 2.0)
        half_h = max(0.5, self.config.search_height / 2.0)
        return [
            (-half_w, -half_h),
            (half_w, -half_h),
            (half_w, half_h),
            (-half_w, half_h),
        ]

    def _spiral_points(self) -> list[tuple[float, float]]:
        radius = max(1.0, min(
            self.config.search_width, self.config.search_height
        ) / 2.0)
        turns = max(1.0, self.config.spiral_turns)
        samples = max(12, int(turns * 16))
        points: list[tuple[float, float]] = []
        for index in range(samples):
            ratio = index / float(samples - 1)
            current_radius = radius * ratio
            angle = 2.0 * math.pi * turns * ratio
            points.append((
                current_radius * math.cos(angle),
                current_radius * math.sin(angle),
            ))
        return points

    @staticmethod
    def supported_patterns() -> Sequence[str]:
        return ('lawnmower', 'perimeter', 'spiral')
