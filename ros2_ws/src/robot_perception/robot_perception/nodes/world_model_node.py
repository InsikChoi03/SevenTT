"""World model: tracked objects in `field` frame + robot pose.

Maintains the integrated world model for the AI Robot Challenge:
  • Subscribes top-cam pixel detections (`/camera_top/detections`) and projects each
    onto the ground plane using the latest robot pose (`/localization/pose`) plus the
    wide-angle camera extrinsics/intrinsics (node parameters).
  • Runs a lightweight nearest-neighbour tracker in field-frame metres: associate,
    EMA-smooth position & confidence, age out stale tracks, honour blacklist requests.
  • Publishes `/world_model` (robot_interfaces/WorldModel) at a fixed rate with all
    tracked objects (id, position, confidence, last_seen, blacklisted) and the robot
    pose (robot_x/y/theta) in the `field` frame.

This node uses no heavy ML libraries; it stays alive with no inputs (empty world),
and degrades gracefully (robot pose only) when the top-cam intrinsics are unset.
NOTE: intrinsics (top_fx/fy/cx/cy) and extrinsics (cam_height/pitch/offset) MUST be
calibrated for the actual rig — the defaults are placeholders.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt64
from robot_interfaces.msg import DetectionArray, Object, WorldModel

# cv2/numpy only needed for the fisheye ray (top cam is a ~150 deg fisheye). Guarded so the
# node still runs (pinhole path) if they are missing.
try:
    import cv2
    import numpy as np

    _CV2_AVAILABLE = True
except Exception:  # noqa: BLE001
    _CV2_AVAILABLE = False


# Optional label -> set_type hint. The top stream is generic, so most detections
# stay unknown (0); these are only used when YOLO-World happens to emit a class name.
_LABEL_TO_SET_TYPE: dict[str, int] = {
    "cube": 1,
    "icosahedron": 1,
    "apple": 2,
    "banana": 2,
    "orange": 2,
}


@dataclass
class Track:
    """A single tracked object in field-frame metres."""

    id: int
    x: float
    y: float
    confidence: float
    last_seen_sec: float
    class_label: str = ""
    set_type: int = 0
    blacklisted: bool = False


class WorldModelNode(Node):
    def __init__(self) -> None:
        super().__init__("world_model_node")

        # Publish / tracker timing.
        self.declare_parameter("publish_rate_hz", 10.0)

        # Top-cam intrinsics (pixels). Any zero -> cannot project (warn, pose only).
        self.declare_parameter("top_fx", 0.0)
        self.declare_parameter("top_fy", 0.0)
        self.declare_parameter("top_cx", 0.0)
        self.declare_parameter("top_cy", 0.0)
        # The wide (top) cam is a ~150 deg FISHEYE. fisheye_model=true -> the pixel->ground
        # ray is computed with cv2.fisheye.undistortPoints(K, D[4]) instead of the pinhole
        # ray ((u-cx)/fx,(v-cy)/fy,1). dist_coeffs is the 4 fisheye coeffs.
        self.declare_parameter("fisheye_model", False)
        self.declare_parameter("dist_coeffs", [0.0, 0.0, 0.0, 0.0])
        # Top image is published 180-deg rotated (camera flip_method=2, mounted inverted). The
        # rotated K handles the principal-point shift, but the optical x/y axes also flip, so the
        # undistorted ray must be rotated 180 about the optical axis (negate x,y) to stay
        # consistent with the (unrotated-mount) R_base_cam extrinsic. Without this, forward<->back
        # and left<->right are swapped in the ground projection.
        self.declare_parameter("image_rotated_180", False)

        # Top-cam extrinsics (base_link / floor).
        self.declare_parameter("cam_height_m", 0.8)        # camera height above floor
        self.declare_parameter("cam_pitch_deg", 60.0)      # downward tilt from horizontal
        self.declare_parameter("cam_offset_x", -0.15)      # rear lift offset (base_link x)
        self.declare_parameter("cam_offset_y", 0.0)        # base_link y offset
        # Camera mount yaw about base z. 0 = camera "forward" aligned with robot +x; 180 if the
        # rig is mounted reversed (verified by ground truth: 0.5m-front object lands at image top
        # only with yaw=180). Independent of image_rotated_180 (that is the optical-axis flip).
        self.declare_parameter("cam_yaw_deg", 0.0)
        self.declare_parameter("object_center_height_m", 0.04)  # ~8cm tall -> center 0.04

        # Tracker tuning.
        self.declare_parameter("assoc_radius_m", 0.18)     # nearest-neighbour gate
        self.declare_parameter("conf_ema", 0.5)            # EMA weight on new sample
        self.declare_parameter("forget_after_sec", 6.0)    # drop unseen non-blacklisted

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.fx = float(self.get_parameter("top_fx").value)
        self.fy = float(self.get_parameter("top_fy").value)
        self.cx = float(self.get_parameter("top_cx").value)
        self.cy = float(self.get_parameter("top_cy").value)
        self.cam_height = float(self.get_parameter("cam_height_m").value)
        self.cam_pitch_deg = float(self.get_parameter("cam_pitch_deg").value)
        self.cam_offset_x = float(self.get_parameter("cam_offset_x").value)
        self.cam_offset_y = float(self.get_parameter("cam_offset_y").value)
        self.cam_yaw_deg = float(self.get_parameter("cam_yaw_deg").value)
        self.object_center_height = float(self.get_parameter("object_center_height_m").value)
        self.assoc_radius = float(self.get_parameter("assoc_radius_m").value)
        self.conf_ema = float(self.get_parameter("conf_ema").value)
        self.forget_after = float(self.get_parameter("forget_after_sec").value)

        self.can_project = self.fx != 0.0 and self.fy != 0.0 and self.cx != 0.0 and self.cy != 0.0

        # Fisheye ray setup (top cam ~150 deg fisheye). Falls back to pinhole if cv2 is
        # missing or fisheye_model is false.
        self.use_fisheye = bool(self.get_parameter("fisheye_model").value)
        self.rotated_180 = bool(self.get_parameter("image_rotated_180").value)
        dist = [float(c) for c in self.get_parameter("dist_coeffs").value]
        self._fish_K = None
        self._fish_D = None
        if self.use_fisheye and self.can_project:
            if not _CV2_AVAILABLE:
                self.get_logger().error(
                    "fisheye_model set but cv2/numpy unavailable; falling back to pinhole ray"
                )
                self.use_fisheye = False
            elif len(dist) < 4:
                self.get_logger().error(
                    f"fisheye_model needs 4 dist_coeffs, got {len(dist)}; falling back to pinhole"
                )
                self.use_fisheye = False
            else:
                self._fish_K = np.array(
                    [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]], np.float64
                )
                self._fish_D = np.array(dist[:4], np.float64).reshape(4, 1)

        # Precompute the (constant) base->cam rotation from a downward pitch about base y.
        # REP-103: base x forward, y left, z up; optical axis = camera +z.
        # With zero pitch the optical axis points along base +x (forward); positive pitch
        # tips it down toward the floor.
        self._R_base_cam = self._matmul3(
            self._rz(math.radians(self.cam_yaw_deg)), self._build_R_base_cam(self.cam_pitch_deg)
        )

        # Latest robot pose in field (0,0,0 until first /localization/pose).
        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.robot_theta: float = 0.0
        self.have_pose: bool = False

        # Track store.
        self.tracks: dict[int, Track] = {}
        self._next_id: int = 1

        self.create_subscription(DetectionArray, "/camera_top/detections", self.on_detections, 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(UInt64, "/world_model/blacklist_add", self.on_blacklist_add, 10)

        self.pub = self.create_publisher(WorldModel, "/world_model", 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        if not self.can_project:
            self.get_logger().warn(
                "top-cam intrinsics unset (top_fx/fy/cx/cy); projection disabled — "
                "publishing robot pose + empty objects until calibrated"
            )
        self.get_logger().info(
            f"rate={rate}Hz project={self.can_project} fisheye={self.use_fisheye} "
            f"rot180={self.rotated_180} "
            f"cam_h={self.cam_height}m pitch={self.cam_pitch_deg}deg yaw={self.cam_yaw_deg}deg "
            f"offset=({self.cam_offset_x},{self.cam_offset_y}) "
            f"obj_h={self.object_center_height}m assoc={self.assoc_radius}m "
            f"ema={self.conf_ema} forget={self.forget_after}s"
        )

    # ------------------------------------------------------------------ helpers

    @staticmethod
    def _build_R_base_cam(pitch_deg: float) -> list[list[float]]:
        """Rotation base_link -> camera optical frame for a downward pitch.

        Camera optical axis is +z (REP-103 pinhole). We want it pointing forward and
        down in base_link. Start from optical-axis-along-base-+x and rotate down by the
        pitch about the base y-axis. Returned as a 3x3 row-major matrix mapping a
        vector expressed in the camera optical frame into base_link.
        NOTE: this is an idealised model; the real rig needs extrinsic calibration.
        """
        p = math.radians(pitch_deg)
        cp = math.cos(p)
        sp = math.sin(p)
        # Camera optical axes expressed in base_link:
        #   cam +z (optical/forward): forward & down -> (cos p, 0, -sin p)
        #   cam +x (image right)    : base -y (image x grows to robot's right) -> (0,-1,0)
        #   cam +y (image down)     : down & forward  -> (-sin p, 0, -cos p)
        # Columns are the camera basis vectors in base_link.
        return [
            [0.0, -sp, cp],
            [-1.0, 0.0, 0.0],
            [0.0, -cp, -sp],
        ]

    @staticmethod
    def _rz(theta: float) -> list[list[float]]:
        c = math.cos(theta)
        s = math.sin(theta)
        return [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _matmul3(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
        return [
            [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]

    @staticmethod
    def _matvec3(m: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
        return (
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ----------------------------------------------------------------- projection

    def _project_pixel(self, u: float, v: float) -> tuple[float, float] | None:
        """Project a top-cam pixel (u,v) onto the object-center ground plane -> field xy.

        Returns None when intrinsics are unset, no pose yet, or the ray does not point
        down into the plane (parallel / upward). Intrinsics & extrinsics need calibration.
        """
        if not self.can_project or not self.have_pose:
            return None

        # Ray in camera optical frame. Fisheye: undistort the pixel to a normalized pinhole
        # ray (x,y,1); pinhole fallback: d = ((u-cx)/fx,(v-cy)/fy,1). The top cam is a ~150 deg
        # fisheye, so the pinhole formula is badly wrong at the periphery.
        if self.use_fisheye:
            und = cv2.fisheye.undistortPoints(
                np.array([[[u, v]]], dtype=np.float64), self._fish_K, self._fish_D
            )
            xn, yn = und[0, 0]
            if self.rotated_180:
                xn, yn = -xn, -yn   # 180-deg image rotation -> ray rotated 180 about optical axis
            d_cam = (float(xn), float(yn), 1.0)
        else:
            d_cam = ((u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0)

        # R_field_cam = Rz(theta) @ R_base_cam.
        r_field_cam = self._matmul3(self._rz(self.robot_theta), self._R_base_cam)
        ray = self._matvec3(r_field_cam, d_cam)

        # Camera position in field:
        #   pcam = (rx,ry,0) + Rz(theta)*(off_x,off_y,0) + (0,0,cam_height)
        off = self._matvec3(self._rz(self.robot_theta), (self.cam_offset_x, self.cam_offset_y, 0.0))
        pcam = (self.robot_x + off[0], self.robot_y + off[1], self.cam_height + off[2])

        # Intersect pcam + t*ray with plane z = object_center_height.
        # Need ray.z < 0 (pointing down) and t > 0.
        if ray[2] >= -1e-6:
            return None  # ray parallel to / above the plane
        t = (self.object_center_height - pcam[2]) / ray[2]
        if t <= 0.0:
            return None
        return (pcam[0] + t * ray[0], pcam[1] + t * ray[1])

    # ----------------------------------------------------------------- callbacks

    def on_pose(self, msg: PoseStamped) -> None:
        self.robot_x = float(msg.pose.position.x)
        self.robot_y = float(msg.pose.position.y)
        # yaw from quaternion (z-up planar): theta = atan2(2*(wz+xy), 1-2*(y^2+z^2)).
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny_cosp, cosy_cosp)
        self.have_pose = True

    def on_blacklist_add(self, msg: UInt64) -> None:
        track = self.tracks.get(int(msg.data))
        if track is None:
            self.get_logger().warn(
                f"blacklist_add for unknown id={int(msg.data)} (ignored)",
                throttle_duration_sec=2.0,
            )
            return
        track.blacklisted = True  # keep so it isn't recreated/re-selected; never un-blacklist
        self.get_logger().info(f"blacklisted track id={track.id}")

    def on_detections(self, msg: DetectionArray) -> None:
        if not self.can_project:
            self.get_logger().warn(
                "top-cam detections received but projection disabled (intrinsics unset)",
                throttle_duration_sec=5.0,
            )
            return
        if not self.have_pose:
            self.get_logger().warn(
                "top-cam detections received before any robot pose; skipping",
                throttle_duration_sec=5.0,
            )
            return

        now = self._now_sec()
        for det in msg.detections:
            # Ground-contact pixel: box bottom-center, averaged with center for stability.
            u_center = float(det.x_center)
            v_center = float(det.y_center)
            v_bottom = v_center + float(det.height) * 0.5
            v = 0.5 * (v_center + v_bottom)
            xy = self._project_pixel(u_center, v)
            if xy is None:
                continue
            label = str(det.label)
            set_type = _LABEL_TO_SET_TYPE.get(label, 0)
            self._associate(xy[0], xy[1], float(det.confidence), label, set_type, now)

    # ------------------------------------------------------------------- tracker

    def _associate(
        self, x: float, y: float, conf: float, label: str, set_type: int, now: float
    ) -> None:
        """Associate a projected detection to the nearest track, else create one."""
        best_id: int | None = None
        best_d = self.assoc_radius
        for tid, tr in self.tracks.items():
            d = math.hypot(tr.x - x, tr.y - y)
            if d <= best_d:
                best_d = d
                best_id = tid

        if best_id is None:
            tid = self._next_id
            self._next_id += 1
            self.tracks[tid] = Track(
                id=tid,
                x=x,
                y=y,
                confidence=conf,
                last_seen_sec=now,
                class_label=label,
                set_type=set_type,
            )
            return

        # EMA update position & confidence on the matched track.
        tr = self.tracks[best_id]
        a = self.conf_ema
        tr.x = (1.0 - a) * tr.x + a * x
        tr.y = (1.0 - a) * tr.y + a * y
        tr.confidence = (1.0 - a) * tr.confidence + a * conf
        tr.last_seen_sec = now
        # Promote label/set_type only when the new detection carries a concrete class.
        if set_type != 0:
            tr.class_label = label
            tr.set_type = set_type
        elif label and not tr.class_label:
            tr.class_label = label
        # Never un-blacklist (blacklisted flag is left untouched).

    def _forget_stale(self, now: float) -> None:
        stale = [
            tid
            for tid, tr in self.tracks.items()
            if not tr.blacklisted and (now - tr.last_seen_sec) > self.forget_after
        ]
        for tid in stale:
            del self.tracks[tid]
        if stale:
            self.get_logger().info(f"forgot {len(stale)} stale track(s)")

    # ------------------------------------------------------------------- publish

    def tick(self) -> None:
        now = self._now_sec()
        self._forget_stale(now)

        msg = WorldModel()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"

        objects: list[Object] = []
        for tr in self.tracks.values():
            obj = Object()
            obj.id = int(tr.id)
            obj.class_label = tr.class_label
            obj.set_type = int(tr.set_type)
            obj.x = float(tr.x)
            obj.y = float(tr.y)
            obj.confidence = float(tr.confidence)
            obj.last_seen = self._sec_to_time_msg(tr.last_seen_sec)
            obj.blacklisted = bool(tr.blacklisted)
            objects.append(obj)
        msg.objects = objects

        msg.robot_x = float(self.robot_x)
        msg.robot_y = float(self.robot_y)
        msg.robot_theta = float(self.robot_theta)
        self.pub.publish(msg)

    @staticmethod
    def _sec_to_time_msg(sec: float):
        from builtin_interfaces.msg import Time

        t = Time()
        t.sec = int(sec)
        t.nanosec = int(round((sec - int(sec)) * 1e9))
        return t


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
