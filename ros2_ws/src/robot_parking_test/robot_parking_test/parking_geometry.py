"""Geometry helpers for the independent arrival parking test.

The wide-camera projection mirrors robot_perception.world_model_node:
fisheye undistortion -> optional measured wide ground homography -> base_link xy
-> field xy through /localization/pose. Arrival floor markers are printed on the
floor, so no standing-object height correction is applied here.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import combinations
from typing import Iterable

try:
    import cv2
    import numpy as np

    _CV2_AVAILABLE = True
except Exception:  # noqa: BLE001
    cv2 = None
    np = None
    _CV2_AVAILABLE = False


@dataclass(frozen=True)
class CameraProjectionConfig:
    top_fx: float = 0.0
    top_fy: float = 0.0
    top_cx: float = 0.0
    top_cy: float = 0.0
    fisheye_model: bool = True
    dist_coeffs: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    image_rotated_180: bool = True
    wide_homography_path: str = ""
    cam_height_m: float = 0.885
    cam_pitch_deg: float = 88.0
    cam_offset_x: float = -0.15
    cam_offset_y: float = 0.0
    cam_yaw_deg: float = 90.0


@dataclass(frozen=True)
class ArrivalObservation:
    index: int
    u: float
    v: float
    confidence: float
    field_x: float
    field_y: float
    width: float = 0.0
    height: float = 0.0
    base_x: float | None = None
    base_y: float | None = None


@dataclass(frozen=True)
class PairScoreConfig:
    center_hint_x: float = 1.8
    center_hint_y: float = 1.8
    center_hint_scale_m: float = 0.65
    floor_pair_spacing_m: float = 0.36
    floor_pair_spacing_tolerance_m: float = 0.18
    floor_pair_min_spacing_m: float = 0.15
    floor_pair_max_spacing_m: float = 0.65


@dataclass(frozen=True)
class ArrivalPair:
    first_index: int
    second_index: int
    center_x: float
    center_y: float
    spacing_m: float
    confidence: float
    score: float
    min_width_px: float = 0.0
    min_height_px: float = 0.0
    mean_area_px: float = 0.0

    @property
    def key(self) -> tuple[int, int]:
        return tuple(sorted((self.first_index, self.second_index)))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(theta: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(theta * 0.5), math.cos(theta * 0.5)


def unit_from_deg(deg: float) -> tuple[float, float]:
    rad = math.radians(deg)
    return math.cos(rad), math.sin(rad)


def field_to_base(
    field_x: float,
    field_y: float,
    robot_x: float,
    robot_y: float,
    robot_theta: float,
) -> tuple[float, float]:
    dx = field_x - robot_x
    dy = field_y - robot_y
    ct = math.cos(robot_theta)
    st = math.sin(robot_theta)
    return ct * dx + st * dy, -st * dx + ct * dy


def base_to_field(
    base_x: float,
    base_y: float,
    robot_x: float,
    robot_y: float,
    robot_theta: float,
) -> tuple[float, float]:
    ct = math.cos(robot_theta)
    st = math.sin(robot_theta)
    return robot_x + base_x * ct - base_y * st, robot_y + base_x * st + base_y * ct


class WideGroundProjector:
    """Project wide-camera pixels to field coordinates.

    If wide_homography_path is set, this uses the measured calibration path from
    world_model_node. Otherwise it falls back to the same idealized extrinsic ray
    intersection, with the target plane fixed to z=0 because arrival markers lie
    on the floor.
    """

    def __init__(self, cfg: CameraProjectionConfig) -> None:
        self.cfg = cfg
        self.can_project = (
            cfg.top_fx != 0.0
            and cfg.top_fy != 0.0
            and cfg.top_cx != 0.0
            and cfg.top_cy != 0.0
        )
        self._fish_k = None
        self._fish_d = None
        self._wide_h = None
        if _CV2_AVAILABLE and self.can_project:
            self._fish_k = np.array(
                [[cfg.top_fx, 0.0, cfg.top_cx], [0.0, cfg.top_fy, cfg.top_cy], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            dist = tuple(float(v) for v in cfg.dist_coeffs[:4])
            self._fish_d = np.array(dist, dtype=np.float64).reshape(4, 1)
            if cfg.wide_homography_path:
                try:
                    self._wide_h = np.asarray(np.load(cfg.wide_homography_path)["H"], dtype=np.float64)
                except Exception:  # noqa: BLE001
                    self._wide_h = None
        self._r_base_cam = _matmul3(
            _rz(math.radians(cfg.cam_yaw_deg)),
            _build_r_base_cam(cfg.cam_pitch_deg),
        )

    @property
    def has_homography(self) -> bool:
        return self._wide_h is not None and self._fish_k is not None and self._fish_d is not None

    def _undistorted_ray(self, u: float, v: float) -> tuple[float, float, float] | None:
        if not self.can_project:
            return None
        if self.cfg.fisheye_model and self._fish_k is not None and self._fish_d is not None:
            if not _CV2_AVAILABLE:
                return None
            und = cv2.fisheye.undistortPoints(
                np.array([[[float(u), float(v)]]], dtype=np.float64),
                self._fish_k,
                self._fish_d,
            )
            xn, yn = und[0, 0]
            if self.cfg.image_rotated_180:
                xn, yn = -xn, -yn
            return float(xn), float(yn), 1.0
        return (
            (float(u) - self.cfg.top_cx) / self.cfg.top_fx,
            (float(v) - self.cfg.top_cy) / self.cfg.top_fy,
            1.0,
        )

    def pixel_to_base(self, u: float, v: float) -> tuple[float, float] | None:
        if self.has_homography:
            und = cv2.fisheye.undistortPoints(
                np.array([[[float(u), float(v)]]], dtype=np.float64),
                self._fish_k,
                self._fish_d,
            )
            xn, yn = und[0, 0]
            if self.cfg.image_rotated_180:
                xn, yn = -xn, -yn
            p = cv2.perspectiveTransform(
                np.array([[[float(xn), float(yn)]]], dtype=np.float64),
                self._wide_h,
            )[0, 0]
            return float(p[0]), float(p[1])
        return None

    def pixel_to_field(
        self,
        u: float,
        v: float,
        robot_pose: tuple[float, float, float],
    ) -> tuple[float, float, float | None, float | None] | None:
        rx, ry, rtheta = robot_pose
        base = self.pixel_to_base(u, v)
        if base is not None:
            fx, fy = base_to_field(base[0], base[1], rx, ry, rtheta)
            return fx, fy, base[0], base[1]
        ray_cam = self._undistorted_ray(u, v)
        if ray_cam is None:
            return None
        r_field_cam = _matmul3(_rz(rtheta), self._r_base_cam)
        ray = _matvec3(r_field_cam, ray_cam)
        off = _matvec3(_rz(rtheta), (self.cfg.cam_offset_x, self.cfg.cam_offset_y, 0.0))
        cam = (rx + off[0], ry + off[1], self.cfg.cam_height_m + off[2])
        if ray[2] >= -1e-6:
            return None
        t = -cam[2] / ray[2]
        if t <= 0.0:
            return None
        fx = cam[0] + t * ray[0]
        fy = cam[1] + t * ray[1]
        bx, by = field_to_base(fx, fy, rx, ry, rtheta)
        return fx, fy, bx, by


@dataclass(frozen=True)
class BodyProjectionConfig:
    body_ground_homography_path: str = ""
    body_px_scale_x: float = 2.5625
    body_px_scale_y: float = 2.566667
    body_workspace_m: tuple[float, float, float, float] = (0.20, 0.75, -0.45, 0.45)


class BodyGroundProjector:
    """Project body-camera pixels to field coordinates through body_ground.npz.

    Unlike world_model_node's object path, this does not apply standing-object
    height correction or front-face radial trim because arrival markers are flat
    on the floor.
    """

    def __init__(self, cfg: BodyProjectionConfig) -> None:
        self.cfg = cfg
        self._body_h = None
        if _CV2_AVAILABLE and cfg.body_ground_homography_path:
            try:
                self._body_h = np.asarray(np.load(cfg.body_ground_homography_path)["H"], dtype=np.float64)
            except Exception:  # noqa: BLE001
                self._body_h = None

    @property
    def can_project(self) -> bool:
        return self._body_h is not None

    def pixel_to_field(
        self,
        u: float,
        v: float,
        robot_pose: tuple[float, float, float],
    ) -> tuple[float, float, float, float] | None:
        if self._body_h is None:
            return None
        pt = np.array(
            [[[float(u) * self.cfg.body_px_scale_x, float(v) * self.cfg.body_px_scale_y]]],
            dtype=np.float64,
        )
        out = cv2.perspectiveTransform(pt, self._body_h)[0][0]
        bx, by = float(out[0]), float(out[1])
        x0, x1, y0, y1 = self.cfg.body_workspace_m
        if not (x0 <= bx <= x1 and y0 <= by <= y1):
            return None
        fx, fy = base_to_field(bx, by, *robot_pose)
        return fx, fy, bx, by


def score_floor_pairs(
    observations: Iterable[ArrivalObservation],
    cfg: PairScoreConfig,
) -> list[ArrivalPair]:
    pairs: list[ArrivalPair] = []
    obs = list(observations)
    for a, b in combinations(obs, 2):
        spacing = math.hypot(a.field_x - b.field_x, a.field_y - b.field_y)
        if spacing < cfg.floor_pair_min_spacing_m or spacing > cfg.floor_pair_max_spacing_m:
            continue
        cx = 0.5 * (a.field_x + b.field_x)
        cy = 0.5 * (a.field_y + b.field_y)
        center_err = math.hypot(cx - cfg.center_hint_x, cy - cfg.center_hint_y)
        center_score = _gaussian_score(center_err, cfg.center_hint_scale_m)
        spacing_score = _gaussian_score(
            spacing - cfg.floor_pair_spacing_m,
            max(1e-6, cfg.floor_pair_spacing_tolerance_m),
        )
        conf_score = max(0.0, min(1.0, 0.5 * (a.confidence + b.confidence)))
        score = 0.45 * center_score + 0.35 * spacing_score + 0.20 * conf_score
        mean_area = 0.5 * (max(0.0, a.width * a.height) + max(0.0, b.width * b.height))
        pairs.append(
            ArrivalPair(
                first_index=a.index,
                second_index=b.index,
                center_x=cx,
                center_y=cy,
                spacing_m=spacing,
                confidence=conf_score,
                score=score,
                min_width_px=min(a.width, b.width),
                min_height_px=min(a.height, b.height),
                mean_area_px=mean_area,
            )
        )
    pairs.sort(key=lambda p: p.score, reverse=True)
    return pairs


def _gaussian_score(error: float, scale: float) -> float:
    if scale <= 0.0:
        return 0.0
    return math.exp(-0.5 * (error / scale) ** 2)


def _build_r_base_cam(pitch_deg: float) -> list[list[float]]:
    p = math.radians(pitch_deg)
    cp = math.cos(p)
    sp = math.sin(p)
    return [
        [0.0, -sp, cp],
        [-1.0, 0.0, 0.0],
        [0.0, -cp, -sp],
    ]


def _rz(theta: float) -> list[list[float]]:
    c = math.cos(theta)
    s = math.sin(theta)
    return [
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _matmul3(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _matvec3(m: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )
