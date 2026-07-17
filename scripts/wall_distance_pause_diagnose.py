#!/usr/bin/env python3
"""Interactive wall-distance diagnostic for the live main pipeline.

Run this alongside the normal robot pipeline. Press SPACE to pause the base,
measure real distances to nearby walls, and compare them against the current
wall raw segments projected into base_link.
"""

from __future__ import annotations

import argparse
import csv
import math
import select
import statistics
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.msg import BaseCommand, MissionState
from std_msgs.msg import Bool, Float32MultiArray, String


DIRS: dict[str, tuple[float, float]] = {
    "front": (1.0, 0.0),
    "left": (0.0, 1.0),
    "right": (0.0, -1.0),
    "back": (-1.0, 0.0),
}


@dataclass
class SegmentBase:
    index: int
    field: tuple[float, float, float, float]
    base: tuple[float, float, float, float]
    mid_x: float
    mid_y: float
    length: float
    angle_deg: float


@dataclass
class DirectionObservation:
    direction: str
    distance: float
    method: str
    segment_index: int


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def chunks4(data: Iterable[float]) -> list[tuple[float, float, float, float]]:
    vals = list(data)
    return [
        (float(vals[i]), float(vals[i + 1]), float(vals[i + 2]), float(vals[i + 3]))
        for i in range(0, len(vals) - 3, 4)
    ]


def cross(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * by - ay * bx


class WallDistancePauseDiagnoser(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("wall_distance_pause_diagnose")
        self.args = args
        self.lock = threading.RLock()
        self.paused = False
        self.snapshot_id = 0

        self.pose: tuple[float, float, float] | None = None
        self.pose_time = 0.0
        self.raw_segments: list[tuple[float, float, float, float]] = []
        self.raw_time = 0.0
        self.snapped_segments: list[tuple[float, float, float, float]] = []
        self.snapped_time = 0.0
        self.wall_field_correction: list[float] = []
        self.wall_field_correction_time = 0.0
        self.wall_map_transform: list[float] = []
        self.wall_map_transform_time = 0.0
        self.motion_mode = ""
        self.motion_mode_time = 0.0
        self.is_stationary: bool | None = None
        self.stationary_time = 0.0
        self.mission_state = ""
        self.mission_time = 0.0

        self.pause_pub = self.create_publisher(Bool, "/debug/pause_motion", 10)
        self.stop_pub = self.create_publisher(BaseCommand, "/base_command", 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(
            Float32MultiArray,
            "/localization/wall_raw_segments",
            self.on_raw_segments,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32MultiArray,
            "/localization/wall_segments",
            self.on_snapped_segments,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32MultiArray,
            "/localization/wall_field_correction",
            self.on_wall_field_correction,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/localization/wall_map_transform",
            self.on_wall_map_transform,
            10,
        )
        self.create_subscription(String, "/localization/motion_mode", self.on_motion_mode, 10)
        self.create_subscription(Bool, "/localization/is_stationary", self.on_stationary, 10)
        self.create_subscription(MissionState, "/mission_state", self.on_mission_state, 10)

        stop_period = 1.0 / max(1.0, float(args.stop_rate_hz))
        self.create_timer(stop_period, self.publish_stop_if_paused)

        out_dir = Path(args.csv_dir).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = out_dir / f"wall_distance_pause_diagnose_{stamp}.csv"
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(
            [
                "row_type",
                "snapshot_id",
                "time_sec",
                "pose_x",
                "pose_y",
                "pose_yaw_rad",
                "motion_mode",
                "is_stationary",
                "mission_state",
                "raw_count",
                "snapped_count",
                "wall_corr_dx",
                "wall_corr_dy",
                "wall_corr_dtheta",
                "map_dx",
                "map_dy",
                "map_dtheta",
                "segment_index",
                "direction",
                "method",
                "observed_m",
                "measured_m",
                "error_m",
                "status",
                "field_x0",
                "field_y0",
                "field_x1",
                "field_y1",
                "base_x0",
                "base_y0",
                "base_x1",
                "base_y1",
                "base_mid_x",
                "base_mid_y",
                "base_len",
                "base_angle_deg",
            ]
        )
        self.get_logger().info(f"wall distance pause diagnose ready; csv={self.csv_path}")

    def destroy_node(self) -> bool:
        try:
            self.csv_file.flush()
            self.csv_file.close()
        finally:
            return super().destroy_node()

    def on_pose(self, msg: PoseStamped) -> None:
        with self.lock:
            self.pose = (
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                yaw_from_pose(msg),
            )
            self.pose_time = time.time()

    def on_raw_segments(self, msg: Float32MultiArray) -> None:
        with self.lock:
            self.raw_segments = chunks4(msg.data)
            self.raw_time = time.time()

    def on_snapped_segments(self, msg: Float32MultiArray) -> None:
        with self.lock:
            self.snapped_segments = chunks4(msg.data)
            self.snapped_time = time.time()

    def on_wall_field_correction(self, msg: Float32MultiArray) -> None:
        with self.lock:
            self.wall_field_correction = [float(v) for v in msg.data]
            self.wall_field_correction_time = time.time()

    def on_wall_map_transform(self, msg: Float32MultiArray) -> None:
        with self.lock:
            self.wall_map_transform = [float(v) for v in msg.data]
            self.wall_map_transform_time = time.time()

    def on_motion_mode(self, msg: String) -> None:
        with self.lock:
            self.motion_mode = str(msg.data)
            self.motion_mode_time = time.time()

    def on_stationary(self, msg: Bool) -> None:
        with self.lock:
            self.is_stationary = bool(msg.data)
            self.stationary_time = time.time()

    def on_mission_state(self, msg: MissionState) -> None:
        with self.lock:
            self.mission_state = str(msg.state)
            self.mission_time = time.time()

    def set_paused(self, paused: bool) -> None:
        with self.lock:
            self.paused = bool(paused)
        if paused and self.args.no_stop:
            self.get_logger().warn("pause requested but --no-stop is set; not publishing /debug/pause_motion")
        self.publish_pause(paused)
        if not paused:
            # Leave the base in a clean stop state; the main pipeline can publish again immediately.
            if self.args.legacy_base_command_stop:
                self.publish_stop_once()

    def publish_stop_if_paused(self) -> None:
        with self.lock:
            paused = self.paused
        if paused:
            self.publish_pause(True)
        if paused and not self.args.no_stop and self.args.legacy_base_command_stop:
            self.publish_stop_once()

    def publish_pause(self, paused: bool) -> None:
        msg = Bool()
        msg.data = bool(paused and not self.args.no_stop)
        self.pause_pub.publish(msg)

    def publish_stop_once(self) -> None:
        msg = BaseCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.vx = 0.0
        msg.vy = 0.0
        msg.omega = 0.0
        self.stop_pub.publish(msg)

    def field_line_to_base(
        self,
        pose: tuple[float, float, float],
        line: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        rx, ry, rth = pose
        ct, st = math.cos(rth), math.sin(rth)
        x0, y0, x1, y1 = line

        def inv(fx: float, fy: float) -> tuple[float, float]:
            dx = fx - rx
            dy = fy - ry
            return dx * ct + dy * st, -dx * st + dy * ct

        bx0, by0 = inv(x0, y0)
        bx1, by1 = inv(x1, y1)
        return bx0, by0, bx1, by1

    def segment_bases(self, pose: tuple[float, float, float], raw: list[tuple[float, float, float, float]]) -> list[SegmentBase]:
        out: list[SegmentBase] = []
        for idx, line in enumerate(raw):
            bx0, by0, bx1, by1 = self.field_line_to_base(pose, line)
            length = math.hypot(bx1 - bx0, by1 - by0)
            if length < 1e-6:
                continue
            mid_x = 0.5 * (bx0 + bx1)
            mid_y = 0.5 * (by0 + by1)
            angle = math.degrees(math.atan2(by1 - by0, bx1 - bx0))
            out.append(
                SegmentBase(
                    index=idx,
                    field=line,
                    base=(bx0, by0, bx1, by1),
                    mid_x=mid_x,
                    mid_y=mid_y,
                    length=length,
                    angle_deg=angle,
                )
            )
        return out

    def observed_distance(
        self,
        segments: list[SegmentBase],
        direction: str,
    ) -> DirectionObservation | None:
        vx, vy = DIRS[direction]
        ray_candidates: list[tuple[float, SegmentBase, str]] = []
        normal_candidates: list[tuple[float, SegmentBase, str]] = []
        margin = float(self.args.ray_segment_margin)
        min_len = float(self.args.min_segment_length)
        min_align = math.cos(math.radians(float(self.args.normal_angle_deg)))

        for seg in segments:
            if seg.length < min_len:
                continue
            bx0, by0, bx1, by1 = seg.base
            dx = bx1 - bx0
            dy = by1 - by0
            denom = cross(vx, vy, dx, dy)
            if abs(denom) > 1e-6:
                t = cross(bx0, by0, dx, dy) / denom
                u = cross(bx0, by0, vx, vy) / denom
                if t > 0.0 and -margin <= u <= 1.0 + margin:
                    penalty = 0.0 if 0.0 <= u <= 1.0 else 0.05
                    ray_candidates.append((t + penalty, seg, "ray"))

            nx = -dy / seg.length
            ny = dx / seg.length
            if nx * vx + ny * vy < 0.0:
                nx, ny = -nx, -ny
            align = nx * vx + ny * vy
            if align >= min_align:
                rho = nx * bx0 + ny * by0
                dist = rho / max(align, 1e-6)
                if dist > 0.0:
                    normal_candidates.append((dist, seg, "normal"))

        candidates = ray_candidates if ray_candidates else normal_candidates
        if not candidates:
            return None
        dist, seg, method = min(candidates, key=lambda item: item[0])
        return DirectionObservation(direction=direction, distance=dist, method=method, segment_index=seg.index)

    def make_snapshot(self) -> dict | None:
        now = time.time()
        with self.lock:
            pose = self.pose
            raw = list(self.raw_segments)
            snapped = list(self.snapped_segments)
            corr = list(self.wall_field_correction)
            transform = list(self.wall_map_transform)
            motion_mode = self.motion_mode
            stationary = self.is_stationary
            mission = self.mission_state
            pose_age = now - self.pose_time if self.pose_time > 0.0 else math.inf
            raw_age = now - self.raw_time if self.raw_time > 0.0 else math.inf
        if pose is None:
            return None
        segments = self.segment_bases(pose, raw)
        observations = {
            name: self.observed_distance(segments, name)
            for name in DIRS
        }
        return {
            "time": now,
            "pose": pose,
            "pose_age": pose_age,
            "raw_age": raw_age,
            "raw_count": len(raw),
            "snapped_count": len(snapped),
            "segments": segments,
            "observations": observations,
            "corr": corr,
            "transform": transform,
            "motion_mode": motion_mode,
            "stationary": stationary,
            "mission": mission,
        }

    def collect_snapshot(self) -> dict | None:
        samples: list[dict] = []
        end = time.time() + max(0.1, float(self.args.sample_sec))
        interval = max(0.05, float(self.args.sample_interval))
        while time.time() < end:
            snap = self.make_snapshot()
            if snap is not None:
                samples.append(snap)
            time.sleep(interval)
        if not samples:
            return None

        latest = samples[-1]
        med_obs: dict[str, DirectionObservation | None] = {}
        for direction in DIRS:
            values = [
                s["observations"][direction].distance
                for s in samples
                if s["observations"].get(direction) is not None
            ]
            if not values:
                med_obs[direction] = None
                continue
            median_dist = statistics.median(values)
            nearest = min(
                (
                    s["observations"][direction]
                    for s in samples
                    if s["observations"].get(direction) is not None
                ),
                key=lambda obs: abs(obs.distance - median_dist),
            )
            med_obs[direction] = DirectionObservation(
                direction=direction,
                distance=median_dist,
                method=nearest.method,
                segment_index=nearest.segment_index,
            )
        latest["observations"] = med_obs
        latest["sample_count"] = len(samples)
        return latest

    def next_snapshot_id(self) -> int:
        with self.lock:
            self.snapshot_id += 1
            return self.snapshot_id

    def write_snapshot_csv(self, sid: int, snap: dict, measurements: dict[str, float]) -> None:
        pose = snap["pose"]
        corr = (snap["corr"] + [math.nan, math.nan, math.nan])[:3]
        transform = (snap["transform"] + [math.nan, math.nan, math.nan])[:3]
        base_common = [
            sid,
            f"{snap['time']:.3f}",
            f"{pose[0]:.6f}",
            f"{pose[1]:.6f}",
            f"{pose[2]:.6f}",
            snap["motion_mode"],
            "" if snap["stationary"] is None else str(bool(snap["stationary"])),
            snap["mission"],
            snap["raw_count"],
            snap["snapped_count"],
            f"{corr[0]:.6f}",
            f"{corr[1]:.6f}",
            f"{corr[2]:.6f}",
            f"{transform[0]:.6f}",
            f"{transform[1]:.6f}",
            f"{transform[2]:.6f}",
        ]
        self.csv.writerow(["snapshot", *base_common, "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""])
        for seg in snap["segments"][: int(self.args.max_segments)]:
            fx0, fy0, fx1, fy1 = seg.field
            bx0, by0, bx1, by1 = seg.base
            self.csv.writerow(
                [
                    "segment",
                    *base_common,
                    seg.index,
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    f"{fx0:.6f}",
                    f"{fy0:.6f}",
                    f"{fx1:.6f}",
                    f"{fy1:.6f}",
                    f"{bx0:.6f}",
                    f"{by0:.6f}",
                    f"{bx1:.6f}",
                    f"{by1:.6f}",
                    f"{seg.mid_x:.6f}",
                    f"{seg.mid_y:.6f}",
                    f"{seg.length:.6f}",
                    f"{seg.angle_deg:.3f}",
                ]
            )
        for direction, measured in measurements.items():
            obs = snap["observations"].get(direction)
            if obs is None:
                self.csv.writerow(
                    [
                        "compare",
                        *base_common,
                        "",
                        direction,
                        "none",
                        "",
                        f"{measured:.6f}",
                        "",
                        "NO_OBSERVATION",
                        *[""] * 12,
                    ]
                )
                continue
            error = obs.distance - measured
            status = "OK" if abs(error) <= float(self.args.error_threshold) else "BAD"
            self.csv.writerow(
                [
                    "compare",
                    *base_common,
                    obs.segment_index,
                    direction,
                    obs.method,
                    f"{obs.distance:.6f}",
                    f"{measured:.6f}",
                    f"{error:.6f}",
                    status,
                    *[""] * 12,
                ]
            )
        self.csv_file.flush()


def fmt_age(age: float) -> str:
    if not math.isfinite(age):
        return "inf"
    return f"{age:.2f}s"


def print_snapshot(sid: int, snap: dict, max_segments: int) -> None:
    pose = snap["pose"]
    corr = (snap["corr"] + [math.nan, math.nan, math.nan])[:3]
    transform = (snap["transform"] + [math.nan, math.nan, math.nan])[:3]
    print()
    print(f"SNAPSHOT #{sid} samples={snap.get('sample_count', 1)}")
    print(
        f"pose=({pose[0]:+.3f},{pose[1]:+.3f}) yaw={math.degrees(pose[2]):+.1f}deg "
        f"pose_age={fmt_age(snap['pose_age'])} raw_age={fmt_age(snap['raw_age'])}"
    )
    print(
        f"state={snap['mission'] or '?'} motion={snap['motion_mode'] or '?'} "
        f"stationary={snap['stationary']} raw={snap['raw_count']} snapped={snap['snapped_count']}"
    )
    print(
        f"wall_field_correction=({corr[0]:+.4f},{corr[1]:+.4f},{corr[2]:+.4f}) "
        f"wall_map_transform=({transform[0]:+.4f},{transform[1]:+.4f},{transform[2]:+.4f})"
    )
    print("observed distances from raw wall segments:")
    for direction in ("front", "left", "right", "back"):
        obs = snap["observations"].get(direction)
        if obs is None:
            print(f"  {direction:>5}: none")
        else:
            print(
                f"  {direction:>5}: {obs.distance:.3f} m "
                f"({obs.method}, seg#{obs.segment_index})"
            )
    if not snap["segments"]:
        print("segments: none")
        return
    print("segments in base_link:")
    for seg in snap["segments"][:max_segments]:
        print(
            f"  seg#{seg.index:02d} mid=({seg.mid_x:+.3f},{seg.mid_y:+.3f})m "
            f"len={seg.length:.3f}m angle={seg.angle_deg:+.1f}deg"
        )


def prompt_measurements(snap: dict) -> dict[str, float]:
    print()
    print("실측 거리를 m 단위로 입력해. 빈칸은 skip.")
    print("예: 앞벽까지 0.52m면 front에 0.52 입력")
    out: dict[str, float] = {}
    for direction in ("front", "left", "right", "back"):
        obs = snap["observations"].get(direction)
        hint = "none" if obs is None else f"observed {obs.distance:.3f}m"
        while True:
            text = input(f"{direction:>5} ({hint}): ").strip()
            if not text:
                break
            try:
                value = float(text)
            except ValueError:
                print("숫자만 입력하거나 빈칸으로 skip해줘.")
                continue
            if value <= 0.0:
                print("거리는 양수로 입력해줘.")
                continue
            out[direction] = value
            break
    return out


def print_comparison(snap: dict, measurements: dict[str, float], threshold: float) -> None:
    if not measurements:
        print("실측값 입력이 없어 비교는 생략했어.")
        return
    print()
    print("measurement comparison:")
    for direction, measured in measurements.items():
        obs = snap["observations"].get(direction)
        if obs is None:
            print(f"  {direction:>5}: measured={measured:.3f}m observed=NONE -> NO_OBSERVATION")
            continue
        error = obs.distance - measured
        status = "OK" if abs(error) <= threshold else "BAD"
        print(
            f"  {direction:>5}: measured={measured:.3f}m observed={obs.distance:.3f}m "
            f"error={error:+.3f}m {status} ({obs.method}, seg#{obs.segment_index})"
        )


def key_available() -> bool:
    readable, _, _ = select.select([sys.stdin], [], [], 0.05)
    return bool(readable)


def interactive_loop(node: WallDistancePauseDiagnoser, args: argparse.Namespace) -> None:
    old_term = termios.tcgetattr(sys.stdin)
    print()
    print("Wall distance pause diagnose")
    print("  SPACE : pause, snapshot, enter real wall distances")
    print("  q     : quit")
    print("  Ctrl+C: quit")
    print()
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if not key_available():
                continue
            ch = sys.stdin.read(1)
            if ch in ("q", "Q"):
                break
            if ch != " ":
                continue

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
            print()
            print("[PAUSE] 스페이스 입력. 로봇 정지 명령을 유지하고 스냅샷을 모을게.")
            if args.no_stop:
                print("주의: --no-stop 모드라 /base_command 정지 명령은 보내지 않아.")
            else:
                print("주의: /debug/pause_motion으로 base_controller 출력만 0으로 clamp해. /base_command 충돌은 없어.")
            node.set_paused(True)
            time.sleep(max(0.0, float(args.settle_sec)))
            snap = node.collect_snapshot()
            sid = node.next_snapshot_id()
            if snap is None:
                print("스냅샷 실패: /localization/pose가 아직 없어.")
                input("Enter를 누르면 다시 대기 모드로 돌아감...")
                node.set_paused(False)
                tty.setcbreak(sys.stdin.fileno())
                continue
            print_snapshot(sid, snap, int(args.max_segments))
            measurements = prompt_measurements(snap)
            print_comparison(snap, measurements, float(args.error_threshold))
            node.write_snapshot_csv(sid, snap, measurements)
            print(f"CSV 저장: {node.csv_path}")
            input("Enter를 누르면 정지 명령을 해제하고 main 파이프라인으로 복귀...")
            node.set_paused(False)
            print("[RESUME] 진단 정지 해제. 다시 SPACE를 누르면 다음 스냅샷.")
            tty.setcbreak(sys.stdin.fileno())
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        node.set_paused(False)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Pause live robot with SPACE and compare wall raw segments to measured distances."
    )
    parser.add_argument("--csv-dir", default="/home/seventt/seventt/workspace/data/wall_debug")
    parser.add_argument("--stop-rate-hz", type=float, default=30.0)
    parser.add_argument(
        "--publish-stop",
        action="store_true",
        help="Deprecated alias for --legacy-base-command-stop.",
    )
    parser.add_argument(
        "--legacy-base-command-stop",
        action="store_true",
        help="Also publish zero /base_command while measuring. Normally not needed and can fight other publishers.",
    )
    parser.add_argument(
        "--no-stop",
        action="store_true",
        help="Do not publish /debug/pause_motion on pause; snapshot only.",
    )
    parser.add_argument("--settle-sec", type=float, default=1.0, help="Wait after pause before sampling.")
    parser.add_argument("--sample-sec", type=float, default=2.0, help="Collect wall observations for this long.")
    parser.add_argument("--sample-interval", type=float, default=0.2)
    parser.add_argument("--error-threshold", type=float, default=0.15)
    parser.add_argument("--max-segments", type=int, default=10)
    parser.add_argument("--min-segment-length", type=float, default=0.08)
    parser.add_argument(
        "--normal-angle-deg",
        type=float,
        default=45.0,
        help="Fallback uses segments whose normal is within this angle of the measured direction.",
    )
    parser.add_argument(
        "--ray-segment-margin",
        type=float,
        default=0.30,
        help="Allow ray intersection slightly outside projected segment endpoints.",
    )
    args = parser.parse_args()
    args.legacy_base_command_stop = bool(args.legacy_base_command_stop or args.publish_stop)
    args.no_stop = bool(args.no_stop and not args.legacy_base_command_stop)
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = WallDistancePauseDiagnoser(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        interactive_loop(node, args)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_paused(False)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
