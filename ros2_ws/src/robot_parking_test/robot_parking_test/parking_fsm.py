"""Pure arrival reverse-parking FSM used by the ROS test node and unit tests."""
from __future__ import annotations

import math
from dataclasses import dataclass

from robot_parking_test.parking_geometry import field_to_base, unit_from_deg, wrap_pi


TERMINAL_STATES = {"SUCCESS", "FAULT", "ABORT"}


@dataclass(frozen=True)
class ParkingConfig:
    corner_direction_deg: float = 45.0
    target_yaw_deg: float = -135.0
    parking_depth_offset_m: float = 0.05
    latch_stable_sec: float = 1.0
    latch_center_tol_m: float = 0.03
    latch_yaw_tol_rad: float = math.radians(3.0)
    position_tol_m: float = 0.03
    yaw_tol_rad: float = math.radians(5.0)
    settle_hold_sec: float = 0.35
    total_timeout_sec: float = 6.0
    approach_timeout_sec: float = 6.0
    approach_standoff_m: float = 0.30
    approach_pos_tol_m: float = 0.05
    approach_max_speed: float = 0.14
    approach_min_speed: float = 0.035
    approach_kp: float = 0.8
    approach_yaw_kp: float = 1.0
    approach_max_omega: float = 0.35
    turn_timeout_sec: float = 2.0
    reverse_timeout_sec: float = 3.0
    correction_timeout_sec: float = 1.0
    max_reverse_speed: float = 0.10
    min_reverse_speed: float = 0.035
    reverse_kp: float = 0.55
    turn_kp: float = 1.2
    max_turn_omega: float = 0.35
    reverse_yaw_kp: float = 0.8
    reverse_max_omega: float = 0.16
    reverse_yaw_fault_rad: float = math.radians(18.0)
    lateral_error_threshold_m: float = 0.05
    lateral_correction_speed: float = 0.045
    lateral_correction_sec: float = 0.35
    lateral_stop_sec: float = 0.15
    max_blind_reverse_m: float = 0.05


@dataclass(frozen=True)
class ParkingTarget:
    marker_center_x: float
    marker_center_y: float
    goal_x: float
    goal_y: float
    yaw: float
    pair_key: tuple[int, int]
    score: float = 0.0


@dataclass(frozen=True)
class BaseCommandValue:
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


class ArrivalParkingFsm:
    def __init__(self, cfg: ParkingConfig) -> None:
        self.cfg = cfg
        self.state = "IDLE"
        self.reason = ""
        self.started = False
        self._state_t = 0.0
        self._start_t = 0.0
        self._approach_t = 0.0
        self._turn_t = 0.0
        self._reverse_t = 0.0
        self._settle_t = 0.0
        self._last_candidate: ParkingTarget | None = None
        self._candidate_since = 0.0
        self.latched_target: ParkingTarget | None = None
        self._correction_used = False
        self._lat_phase = "idle"
        self._lat_t = 0.0
        self._target_lost_pose: tuple[float, float, float] | None = None

    def reset(self, now: float = 0.0) -> None:
        self.__init__(self.cfg)
        self._state_t = now

    def start(self, now: float) -> None:
        self.started = True
        self._start_t = now
        if self.state == "IDLE":
            self._enter("ACQUIRE", now, "start requested")

    def abort(self, now: float, reason: str = "abort requested") -> None:
        self.started = False
        self._enter("ABORT", now, reason)

    def fault(self, now: float, reason: str) -> None:
        self._enter("FAULT", now, reason)

    def tick(
        self,
        now: float,
        pose: tuple[float, float, float] | None,
        target: ParkingTarget | None,
        target_fresh: bool,
    ) -> BaseCommandValue:
        if self.state in TERMINAL_STATES:
            return BaseCommandValue()
        if not self.started:
            self.state = "IDLE"
            return BaseCommandValue()
        if pose is None and self.state == "ACQUIRE":
            return BaseCommandValue()
        if pose is None:
            self.fault(now, "pose unavailable")
            return BaseCommandValue()
        active_parking = self.state in ("TURN_FOR_REVERSE", "STRAIGHT_REVERSE", "SETTLE")
        if active_parking and now - self._start_t > self.cfg.total_timeout_sec:
            self.fault(now, "parking total timeout")
            return BaseCommandValue()

        if self.state == "ACQUIRE":
            if target is None:
                return BaseCommandValue()
            if self._target_is_stable(target, now):
                self.latched_target = target
                self._enter("READY", now, "arrival target latched")
            return BaseCommandValue()

        if self.latched_target is None:
            self._enter("ACQUIRE", now, "waiting for arrival target")
            return BaseCommandValue()

        if self.state == "READY":
            self._start_t = now
            self._approach_t = now
            self._enter("APPROACH_REVERSE_START", now, "move to reverse start pose")
            return BaseCommandValue()

        if self.state == "APPROACH_REVERSE_START":
            return self._step_approach(now, pose)

        if self.state == "TURN_FOR_REVERSE":
            return self._step_turn(now, pose)

        if self.state == "STRAIGHT_REVERSE":
            return self._step_reverse(now, pose, target_fresh)

        if self.state == "SETTLE":
            if self._within_goal(pose):
                if now - self._settle_t >= self.cfg.settle_hold_sec:
                    self._enter("SUCCESS", now, "parking settled")
                return BaseCommandValue()
            self._enter("STRAIGHT_REVERSE", now, "settle drifted")
            return BaseCommandValue()

        return BaseCommandValue()

    def _target_is_stable(self, target: ParkingTarget, now: float) -> bool:
        if self._last_candidate is None or target.pair_key != self._last_candidate.pair_key:
            self._last_candidate = target
            self._candidate_since = now
            return False
        dc = math.hypot(
            target.marker_center_x - self._last_candidate.marker_center_x,
            target.marker_center_y - self._last_candidate.marker_center_y,
        )
        dyaw = abs(wrap_pi(target.yaw - self._last_candidate.yaw))
        if dc > self.cfg.latch_center_tol_m or dyaw > self.cfg.latch_yaw_tol_rad:
            self._last_candidate = target
            self._candidate_since = now
            return False
        self._last_candidate = target
        return now - self._candidate_since >= self.cfg.latch_stable_sec

    def _reverse_start_pose(self) -> tuple[float, float, float]:
        target = self.latched_target
        fx = math.cos(target.yaw)
        fy = math.sin(target.yaw)
        return (
            target.goal_x + fx * self.cfg.approach_standoff_m,
            target.goal_y + fy * self.cfg.approach_standoff_m,
            target.yaw,
        )

    def _step_approach(self, now: float, pose: tuple[float, float, float]) -> BaseCommandValue:
        if now - self._approach_t > self.cfg.approach_timeout_sec:
            self.fault(now, "approach reverse start timeout")
            return BaseCommandValue()
        sx, sy, syaw = self._reverse_start_pose()
        bx, by = field_to_base(sx, sy, pose[0], pose[1], pose[2])
        yaw_err = wrap_pi(syaw - pose[2])
        dist = math.hypot(bx, by)
        if dist <= self.cfg.approach_pos_tol_m and abs(yaw_err) <= self.cfg.yaw_tol_rad:
            self._turn_t = now
            self._enter("TURN_FOR_REVERSE", now, "reverse start pose reached")
            return BaseCommandValue()

        if dist <= self.cfg.approach_pos_tol_m:
            vx = 0.0
            vy = 0.0
        else:
            speed = _clamp(
                dist * self.cfg.approach_kp,
                self.cfg.approach_min_speed,
                self.cfg.approach_max_speed,
            )
            vx = speed * bx / dist
            vy = speed * by / dist
        omega = _clamp(
            self.cfg.approach_yaw_kp * yaw_err,
            -self.cfg.approach_max_omega,
            self.cfg.approach_max_omega,
        )
        return BaseCommandValue(vx, vy, omega)

    def _step_turn(self, now: float, pose: tuple[float, float, float]) -> BaseCommandValue:
        if now - self._turn_t > self.cfg.turn_timeout_sec:
            self.fault(now, "turn timeout")
            return BaseCommandValue()
        yaw_err = wrap_pi(self.latched_target.yaw - pose[2])
        if abs(yaw_err) <= self.cfg.yaw_tol_rad:
            self._reverse_t = now
            self._enter("STRAIGHT_REVERSE", now, "yaw aligned")
            return BaseCommandValue()
        omega = _clamp(self.cfg.turn_kp * yaw_err, -self.cfg.max_turn_omega, self.cfg.max_turn_omega)
        return BaseCommandValue(0.0, 0.0, omega)

    def _step_reverse(
        self,
        now: float,
        pose: tuple[float, float, float],
        target_fresh: bool,
    ) -> BaseCommandValue:
        if now - self._reverse_t > self.cfg.reverse_timeout_sec:
            self.fault(now, "reverse timeout")
            return BaseCommandValue()
        if not target_fresh:
            if self._target_lost_pose is None:
                self._target_lost_pose = pose
            blind = math.hypot(pose[0] - self._target_lost_pose[0], pose[1] - self._target_lost_pose[1])
            if blind > self.cfg.max_blind_reverse_m:
                self.fault(now, "arrival detections stale beyond blind reverse limit")
                return BaseCommandValue()
        else:
            self._target_lost_pose = None

        target = self.latched_target
        bx, by = field_to_base(target.goal_x, target.goal_y, pose[0], pose[1], pose[2])
        yaw_err = wrap_pi(target.yaw - pose[2])
        pos_err = math.hypot(target.goal_x - pose[0], target.goal_y - pose[1])
        if pos_err <= self.cfg.position_tol_m and abs(yaw_err) <= self.cfg.yaw_tol_rad:
            self._settle_t = now
            self._enter("SETTLE", now, "goal tolerance reached")
            return BaseCommandValue()
        if abs(yaw_err) > self.cfg.reverse_yaw_fault_rad:
            self.fault(now, "yaw drift too large during reverse")
            return BaseCommandValue()
        if bx > self.cfg.position_tol_m:
            self._approach_t = now
            self._enter("APPROACH_REVERSE_START", now, "target moved in front; re-approach")
            return BaseCommandValue()

        if abs(by) > self.cfg.lateral_error_threshold_m and not self._correction_used:
            if self._lat_phase == "idle":
                self._lat_phase = "stop"
                self._lat_t = now
                return BaseCommandValue()
            if self._lat_phase == "stop":
                if now - self._lat_t < self.cfg.lateral_stop_sec:
                    return BaseCommandValue()
                self._lat_phase = "pulse"
                self._lat_t = now
            if self._lat_phase == "pulse":
                if now - self._lat_t < min(self.cfg.lateral_correction_sec, self.cfg.correction_timeout_sec):
                    return BaseCommandValue(0.0, _sign(by) * self.cfg.lateral_correction_speed, 0.0)
                self._lat_phase = "idle"
                self._correction_used = True
                return BaseCommandValue()

        vx_mag = min(self.cfg.max_reverse_speed, max(self.cfg.min_reverse_speed, abs(bx) * self.cfg.reverse_kp))
        vx = -vx_mag
        omega = _clamp(self.cfg.reverse_yaw_kp * yaw_err, -self.cfg.reverse_max_omega, self.cfg.reverse_max_omega)
        if vx > 0.0:
            self.fault(now, "internal safety blocked forward command")
            return BaseCommandValue()
        return BaseCommandValue(vx, 0.0, omega)

    def _within_goal(self, pose: tuple[float, float, float]) -> bool:
        target = self.latched_target
        if target is None:
            return False
        pos_err = math.hypot(target.goal_x - pose[0], target.goal_y - pose[1])
        yaw_err = abs(wrap_pi(target.yaw - pose[2]))
        return pos_err <= self.cfg.position_tol_m and yaw_err <= self.cfg.yaw_tol_rad

    def _enter(self, state: str, now: float, reason: str) -> None:
        self.state = state
        self.reason = reason
        self._state_t = now


def target_from_marker_center(
    center_x: float,
    center_y: float,
    pair_key: tuple[int, int],
    score: float,
    cfg: ParkingConfig,
) -> ParkingTarget:
    ux, uy = unit_from_deg(cfg.corner_direction_deg)
    return ParkingTarget(
        marker_center_x=center_x,
        marker_center_y=center_y,
        goal_x=center_x + ux * cfg.parking_depth_offset_m,
        goal_y=center_y + uy * cfg.parking_depth_offset_m,
        yaw=math.radians(cfg.target_yaw_deg),
        pair_key=tuple(sorted(pair_key)),
        score=score,
    )


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


def _sign(value: float) -> float:
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0
