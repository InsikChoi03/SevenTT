#!/usr/bin/env python3
"""Debug wall mask line projection relative to base_link.

This subscribes to the wall-localizer debug topics and reports where the
orange raw map lines land in the robot frame. Use it with the robot stopped at
a known distance from a wall, for example wall roughly 0.50 m in front.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path
from typing import Iterable

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray


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


class WallBoundaryBaseDebug(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("wall_boundary_base_debug")
        self.expected_x = float(args.expected_x)
        self.tolerance = float(args.tolerance)
        self.target_field_x = args.target_field_x
        self.target_field_y = args.target_field_y
        self.print_period = max(0.1, float(args.print_period))
        self.max_lines = max(1, int(args.max_lines))

        self.pose: tuple[float, float, float] | None = None
        self.raw_segments: list[tuple[float, float, float, float]] = []
        self.mask_segments: list[tuple[float, float, float, float]] = []
        self.raw_time = 0.0
        self.mask_time = 0.0

        out_dir = Path(args.out_dir).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = out_dir / f"wall_boundary_base_debug_{stamp}.csv"
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(
            [
                "time_sec",
                "pose_x",
                "pose_y",
                "pose_yaw_rad",
                "segment_index",
                "img_x0",
                "img_y0",
                "img_x1",
                "img_y1",
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
                "front_error_m",
                "status",
            ]
        )

        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(
            Float32MultiArray,
            "/localization/wall_raw_segments",
            self.on_raw_segments,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32MultiArray,
            "/localization/wall_mask_segments_image",
            self.on_mask_segments,
            qos_profile_sensor_data,
        )
        self.create_timer(self.print_period, self.on_timer)

        self.get_logger().info(
            "wall boundary base debug ready: expect wall base_mid_x around "
            f"{self.expected_x:.2f}m, csv={self.csv_path}"
        )

    def destroy_node(self) -> bool:
        try:
            self.csv_file.flush()
            self.csv_file.close()
        finally:
            return super().destroy_node()

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            yaw_from_pose(msg),
        )

    def on_raw_segments(self, msg: Float32MultiArray) -> None:
        self.raw_segments = chunks4(msg.data)
        self.raw_time = time.time()

    def on_mask_segments(self, msg: Float32MultiArray) -> None:
        self.mask_segments = chunks4(msg.data)
        self.mask_time = time.time()

    def field_line_to_base(
        self,
        line: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float] | None:
        if self.pose is None:
            return None
        rx, ry, rth = self.pose
        ct, st = math.cos(rth), math.sin(rth)
        x0, y0, x1, y1 = line

        def inv(fx: float, fy: float) -> tuple[float, float]:
            dx = fx - rx
            dy = fy - ry
            return dx * ct + dy * st, -dx * st + dy * ct

        bx0, by0 = inv(x0, y0)
        bx1, by1 = inv(x1, y1)
        return bx0, by0, bx1, by1

    def classify(self, mid_x: float) -> tuple[float, str]:
        err = mid_x - self.expected_x
        if mid_x < -0.05:
            return err, "BEHIND_ROBOT"
        if abs(err) > self.tolerance:
            return err, "XY_OFFSET"
        return err, "OK"

    def on_timer(self) -> None:
        now = time.time()
        if self.pose is None:
            self.get_logger().warn("waiting for /localization/pose", throttle_duration_sec=2.0)
            return
        raw_age = now - self.raw_time if self.raw_time > 0.0 else math.inf
        mask_age = now - self.mask_time if self.mask_time > 0.0 else math.inf
        rx, ry, rth = self.pose
        target_txt = ""
        if self.target_field_x is not None and self.target_field_y is not None:
            d = math.hypot(float(self.target_field_x) - rx, float(self.target_field_y) - ry)
            target_txt = f" target_dist={d:.2f}m"

        header = (
            f"pose field=({rx:.3f},{ry:.3f}) yaw={math.degrees(rth):.1f}deg"
            f"{target_txt} raw={len(self.raw_segments)} age={raw_age:.2f}s"
            f" img_yellow={len(self.mask_segments)} age={mask_age:.2f}s"
        )
        self.get_logger().info(header)

        if not self.raw_segments:
            self.get_logger().warn("no /localization/wall_raw_segments yet", throttle_duration_sec=2.0)
            return

        rows = []
        for idx, line in enumerate(self.raw_segments[: self.max_lines]):
            base = self.field_line_to_base(line)
            if base is None:
                continue
            bx0, by0, bx1, by1 = base
            mid_x = 0.5 * (bx0 + bx1)
            mid_y = 0.5 * (by0 + by1)
            length = math.hypot(bx1 - bx0, by1 - by0)
            angle = math.degrees(math.atan2(by1 - by0, bx1 - bx0))
            err, status = self.classify(mid_x)
            img = self.mask_segments[idx] if idx < len(self.mask_segments) else ("", "", "", "")
            rows.append((idx, mid_x, mid_y, length, angle, err, status, base, line, img))

        rows.sort(key=lambda item: abs(item[5]))
        for idx, mid_x, mid_y, length, angle, err, status, base, line, img in rows:
            bx0, by0, bx1, by1 = base
            x0, y0, x1, y1 = line
            self.get_logger().info(
                f"seg#{idx:02d} base_mid=({mid_x:+.3f},{mid_y:+.3f})m "
                f"len={length:.3f}m angle={angle:+.1f}deg "
                f"front_err={err:+.3f}m {status}"
            )
            self.csv.writerow(
                [
                    f"{now:.3f}",
                    f"{rx:.6f}",
                    f"{ry:.6f}",
                    f"{rth:.6f}",
                    idx,
                    *img,
                    f"{x0:.6f}",
                    f"{y0:.6f}",
                    f"{x1:.6f}",
                    f"{y1:.6f}",
                    f"{bx0:.6f}",
                    f"{by0:.6f}",
                    f"{bx1:.6f}",
                    f"{by1:.6f}",
                    f"{mid_x:.6f}",
                    f"{mid_y:.6f}",
                    f"{length:.6f}",
                    f"{angle:.3f}",
                    f"{err:.6f}",
                    status,
                ]
            )
        self.csv_file.flush()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Print wall line positions in base_link coordinates from "
            "/localization/wall_raw_segments."
        )
    )
    parser.add_argument("--expected-x", type=float, default=0.50, help="Expected wall distance in front of robot.")
    parser.add_argument("--tolerance", type=float, default=0.15, help="Allowed base_mid_x error before warning.")
    parser.add_argument("--print-period", type=float, default=1.0, help="Print interval in seconds.")
    parser.add_argument("--max-lines", type=int, default=8, help="Max raw wall segments to print each cycle.")
    parser.add_argument(
        "--target-field-x",
        type=float,
        default=-1.3,
        help="Optional expected field x for the robot test pose; only printed as distance.",
    )
    parser.add_argument(
        "--target-field-y",
        type=float,
        default=1.3,
        help="Optional expected field y for the robot test pose; only printed as distance.",
    )
    parser.add_argument(
        "--out-dir",
        default="/home/seventt/seventt/workspace/data/wall_debug",
        help="Directory for CSV logs.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = WallBoundaryBaseDebug(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
