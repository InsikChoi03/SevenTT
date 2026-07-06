"""Mission FSM for the AI Robot Challenge.

States:
    SCAN              - top cam world model build/update
    SELECT_TARGET     - pick next object via /selected_target
    APPROACH          - drive base toward target
    ALIGN             - body cam visual servo
    CLASSIFY          - SigLIP gate (+ shape heuristic for set1)
    PICK              - arm pickup motion
    STORE_IN_TRAY     - place in body tray, increment counters
    DRIVE_TO_STORAGE  - move to storage zone
    ALIGN_OVER_BIN    - align over storage box
    DUMP_ALL          - tilt/release tray
    END               - terminal

Each periodic tick evaluates condition-driven transitions using the latest
world model / selected target / classification messages, and drives the base
(/base/goal_pose) and arm (/arm/pick_trigger). A manual /state_advance trigger
still force-advances one transition for debug/dry-run exercising.

Pick-gate (rulebook §6/§7, mispick on Set2 = -40, so be conservative):
  - Set2 fruit: siglip == today's fruit + image_face_visible + conf >= thresh -> PICK.
  - Set1 cube : shape == today's cube  + NOT image_face_visible + conf >= thresh -> PICK.
  - otherwise (or classify timeout) -> PASS (blacklist, reselect).
"""
from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_interfaces.msg import BaseCommand, Classification, DetectionArray, MissionState, Object, WorldModel
from std_msgs.msg import Bool, Empty, Int8, String, UInt64

# Body detection label -> set_type (mirror of world_model), for the ALIGN visual-servo filter.
_LABEL_ST = {"cube": 1, "octahedron": 1, "dodecahedron": 1, "icosahedron": 1, "fruit_photo_cube": 2}


STATES = [
    "SCAN", "SELECT_TARGET", "APPROACH", "ALIGN", "CLASSIFY", "PICK",
    "STORE_IN_TRAY", "DRIVE_TO_STORAGE", "ALIGN_OVER_BIN", "DUMP_ALL", "END",
]


class MissionFsmNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_fsm_node")

        # Today's announced targets (rulebook §6.1, §7.3).
        self.declare_parameter("set1_label", "")      # e.g. "icosahedron"
        self.declare_parameter("set2_label", "")      # e.g. "apple"
        self.declare_parameter("conf_threshold", 0.7)
        self.declare_parameter("pick_track_conf", 0.5)   # min world-model track confidence to pick the latched target
        self.declare_parameter("approach_dist_m", 0.25)
        self.declare_parameter("align_settle_sec", 1.0)
        self.declare_parameter("classify_timeout_sec", 2.0)
        # ALIGN visual servo: drive the mecanum base so the target lands on the arm's fixed grab
        # point (base_link m, measured 2026-07-04: 26.4cm fwd / 0.7cm left). SAFETY: never advance
        # the target past grab_min_x — the body cam blind-limit (~25cm) is only ~1cm nearer, so an
        # overshoot loses sight of the object right before the grab.
        self.declare_parameter("grab_x", 0.264)
        self.declare_parameter("grab_y", 0.007)
        self.declare_parameter("grab_min_x", 0.255)     # never push the target closer than this
        self.declare_parameter("align_tol_m", 0.04)     # aligned within this -> grab (gripper absorbs)
        self.declare_parameter("align_kp", 0.6)         # m/s per m of error
        self.declare_parameter("align_vmax", 0.16)      # cap
        # STICTION: the heavy base won't move below ~this speed (motor just buzzes), so any nonzero
        # servo command is boosted to at least this. Bigger tol above absorbs the coarser steps.
        self.declare_parameter("align_vmin", 0.13)
        self.declare_parameter("align_timeout_sec", 12.0)
        # Pulse+settle: the heavy base coasts after a command, so instead of a continuous servo we
        # nudge briefly, let it FULLY STOP (inertia dissipates), then measure the settled position
        # and grab if within tolerance (gripper opening absorbs the residual). Repeat otherwise.
        self.declare_parameter("align_pulse_sec", 0.15)     # burst length per nudge
        self.declare_parameter("align_settle_pulse_sec", 0.8)  # wait for the base to fully stop
        self.declare_parameter("pick_duration_sec", 3.0)
        self.declare_parameter("storage_x", 0.2)
        self.declare_parameter("storage_y", 0.2)
        self.declare_parameter("shape_target_total", 4)   # set1 shape * 4
        self.declare_parameter("fruit_target_total", 3)   # set2 fruit * 3
        self.declare_parameter("publish_rate_hz", 5.0)
        # --- mock-field-test knobs (all default to competition behaviour) ---
        # dry_pick: log the pick instead of firing /arm/pick_trigger (arm not driven in the test).
        # end_after_quota: END once both quotas are met (skip STORE/DRIVE/DUMP storage phase).
        # select_timeout_sec>0: if SELECT_TARGET finds nothing for this long, advance phase (1->2)
        #   or END (phase 2) — graceful termination when fewer objects are present than the quota.
        self.declare_parameter("dry_pick", False)
        self.declare_parameter("end_after_quota", False)
        self.declare_parameter("select_timeout_sec", 0.0)

        self.set1_label = str(self.get_parameter("set1_label").value)
        self.set2_label = str(self.get_parameter("set2_label").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.pick_track_conf = float(self.get_parameter("pick_track_conf").value)
        self.approach_dist_m = float(self.get_parameter("approach_dist_m").value)
        self.align_settle_sec = float(self.get_parameter("align_settle_sec").value)
        self.classify_timeout_sec = float(self.get_parameter("classify_timeout_sec").value)
        self.grab_x = float(self.get_parameter("grab_x").value)
        self.grab_y = float(self.get_parameter("grab_y").value)
        self.grab_min_x = float(self.get_parameter("grab_min_x").value)
        self.align_tol = float(self.get_parameter("align_tol_m").value)
        self.align_kp = float(self.get_parameter("align_kp").value)
        self.align_vmax = float(self.get_parameter("align_vmax").value)
        self.align_vmin = float(self.get_parameter("align_vmin").value)
        self.align_timeout_sec = float(self.get_parameter("align_timeout_sec").value)
        self.align_pulse_sec = float(self.get_parameter("align_pulse_sec").value)
        self.align_settle_pulse_sec = float(self.get_parameter("align_settle_pulse_sec").value)
        self._align_phase = "measure"      # measure -> pulse -> settle -> measure ...
        self._align_phase_start = 0.0
        self._pulse_vx = 0.0
        self._pulse_vy = 0.0
        # Body-cam ground homography for POSE-INDEPENDENT servo: project the object's body pixel
        # straight to base_link (no robot pose), so pose drift can't wander the servo target.
        self.declare_parameter("body_ground_homography_path",
                               "/home/seventt/seventt/workspace/data/calib/body_ground.npz")
        self.declare_parameter("body_cam_nadir_x", 0.065)
        self.declare_parameter("body_cam_height_m", 0.145)
        self.declare_parameter("object_center_height_m", 0.04)
        self._body_H = None
        try:
            self._body_H = np.load(str(self.get_parameter("body_ground_homography_path").value))["H"].astype(np.float64)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"body homography load failed ({exc}); ALIGN servo uses pose-based fallback")
        self._bnx = float(self.get_parameter("body_cam_nadir_x").value)
        self._bH = float(self.get_parameter("body_cam_height_m").value)
        self._boh = float(self.get_parameter("object_center_height_m").value)
        self._body_dets: list = []   # latest /camera_body/detections as (u, v_center, set_type)
        self.pick_duration_sec = float(self.get_parameter("pick_duration_sec").value)
        self.storage_x = float(self.get_parameter("storage_x").value)
        self.storage_y = float(self.get_parameter("storage_y").value)
        self.shape_target_total = int(self.get_parameter("shape_target_total").value)
        self.fruit_target_total = int(self.get_parameter("fruit_target_total").value)
        self.dry_pick = bool(self.get_parameter("dry_pick").value)
        self.end_after_quota = bool(self.get_parameter("end_after_quota").value)
        self.select_timeout_sec = float(self.get_parameter("select_timeout_sec").value)
        rate = float(self.get_parameter("publish_rate_hz").value)

        # --- runtime state ---
        self.state = "SCAN"
        self.phase = 1          # 1 = pursue Set1, 2 = pursue Set2 (pick ordering; mapping is continuous)
        self.tray_shape = 0
        self.tray_fruit = 0
        self.current_target: Object | None = None    # latched selected target (id, set_type, ...)
        self.set_type = 0                             # set_type confirmed by the pick gate

        # latest inbound messages
        self.world: WorldModel | None = None
        self.selected: Object | None = None
        self.siglip: Classification | None = None
        self.shape: Classification | None = None
        self.siglip_stamp_s: float | None = None      # arrival time (node clock) of last siglip
        self.shape_stamp_s: float | None = None        # arrival time (node clock) of last shape

        # time bookkeeping (seconds, node clock)
        self.state_enter_s = self._now_s()

        self.create_subscription(WorldModel, "/world_model", self.on_world, 10)
        self.create_subscription(Object, "/selected_target", self.on_target, 10)
        self.create_subscription(Classification, "/classification/siglip", self.on_siglip, 10)
        self.create_subscription(Classification, "/classification/shape", self.on_shape, 10)
        self.create_subscription(DetectionArray, "/camera_body/detections", self.on_body_dets, 10)
        self.create_subscription(Empty, "/state_advance", self.on_advance, 10)

        self.pub_state = self.create_publisher(MissionState, "/mission_state", 10)
        self.pub_blacklist = self.create_publisher(UInt64, "/world_model/blacklist_add", 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/base/goal_pose", 10)
        self.pub_pick = self.create_publisher(Bool, "/arm/pick_trigger", 10)
        self.pub_cmd = self.create_publisher(BaseCommand, "/base_command", 10)   # ALIGN visual servo
        # Current pick phase (1=Set1, 2=Set2) for the target selector's phase filter.
        self.pub_phase = self.create_publisher(Int8, "/planning/phase", 10)
        # Human-readable decision feed (PICK / PASS / SKIP / PHASE / END) for the visualiser.
        self.pub_decision = self.create_publisher(String, "/planning/decision", 10)

        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"FSM started at {self.state}  set1='{self.set1_label}' set2='{self.set2_label}' "
            f"conf>={self.conf_threshold} approach<{self.approach_dist_m}m rate={rate}Hz "
            f"dry_pick={self.dry_pick} end_after_quota={self.end_after_quota} "
            f"select_timeout={self.select_timeout_sec}s"
        )

    # ------------------------------------------------------------------ utils
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _time_in_state(self) -> float:
        return self._now_s() - self.state_enter_s

    def _enter(self, new_state: str) -> None:
        prev = self.state
        self.state = new_state
        self.state_enter_s = self._now_s()
        if new_state == "ALIGN":
            self._align_phase = "measure"       # start each ALIGN by measuring the settled position
        self.get_logger().info(
            f"transition {prev} -> {new_state}  "
            f"tray=(shape:{self.tray_shape}, fruit:{self.tray_fruit})  "
            f"target_id={self.current_target.id if self.current_target else 0}"
        )

    def _lookup_object(self, obj_id: int) -> Object | None:
        """Fresh world-model entry for a tracker id (None if not present)."""
        if self.world is None or obj_id == 0:
            return None
        for obj in self.world.objects:
            if obj.id == obj_id:
                return obj
        return None

    def _to_base(self, fx: float, fy: float) -> tuple[float, float] | None:
        """Field xy -> base_link xy (x fwd, y left) using the world model's robot pose."""
        if self.world is None:
            return None
        dx, dy = fx - self.world.robot_x, fy - self.world.robot_y
        ct, st = math.cos(self.world.robot_theta), math.sin(self.world.robot_theta)
        return (ct * dx + st * dy, -st * dx + ct * dy)

    def _front_target(self, set_type: int, max_r: float = 0.45) -> Object | None:
        """The non-blacklisted object of this set_type nearest the arm grab point (base_link).
        Used in ALIGN/CLASSIFY instead of a latched track id, which churns as tracks are dropped
        and recreated during the approach — the object physically in front IS the target."""
        if self.world is None:
            return None
        best = None
        bestd = max_r
        for o in self.world.objects:
            if o.blacklisted or o.set_type != set_type:
                continue
            base = self._to_base(o.x, o.y)
            if base is None:
                continue
            d = math.hypot(base[0] - self.grab_x, base[1] - self.grab_y)
            if d < bestd:
                bestd = d
                best = o
        return best

    def on_body_dets(self, msg: DetectionArray) -> None:
        self._body_dets = [(float(d.x_center), float(d.y_center), _LABEL_ST.get(str(d.label), 0))
                           for d in msg.detections]

    def _body_target_base(self, set_type: int) -> tuple[float, float] | None:
        """POSE-INDEPENDENT servo target: project each body detection of this set_type straight to
        base_link (body homography + 8cm height correction), return the one nearest the grab point.
        No robot pose used -> pose drift cannot wander the target (that was the back-and-forth)."""
        if self._body_H is None or not self._body_dets:
            return None
        best = None
        bestd = 0.5
        k = self._boh / self._bH
        for u, v, st in self._body_dets:
            if st != set_type:
                continue
            p = cv2.perspectiveTransform(np.array([[[u, v]]], np.float64), self._body_H)[0][0]
            bx, by = float(p[0]) - k * (float(p[0]) - self._bnx), float(p[1]) - k * float(p[1])
            d = math.hypot(bx - self.grab_x, by - self.grab_y)
            if d < bestd:
                bestd = d
                best = (bx, by)
        return best

    def _drive(self, vx: float, vy: float, omega: float = 0.0) -> None:
        c = BaseCommand()
        c.header.stamp = self.get_clock().now().to_msg()
        c.vx, c.vy, c.omega = float(vx), float(vy), float(omega)
        self.pub_cmd.publish(c)

    def _robot_xy(self) -> tuple[float, float] | None:
        if self.world is None:
            return None
        return (self.world.robot_x, self.world.robot_y)

    def _distance_to(self, x: float, y: float) -> float | None:
        rxy = self._robot_xy()
        if rxy is None:
            return None
        return math.hypot(x - rxy[0], y - rxy[1])

    def _publish_goal(self, x: float, y: float, theta: float = 0.0) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        # quaternion from yaw (REP-103): qz=sin(theta/2), qw=cos(theta/2), qx=qy=0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        self.pub_goal.publish(msg)

    def _publish_pick(self, trigger: bool) -> None:
        self.pub_pick.publish(Bool(data=trigger))

    def _blacklist(self, obj_id: int) -> None:
        if obj_id != 0:
            self.pub_blacklist.publish(UInt64(data=int(obj_id)))

    def _decide(self, text: str) -> None:
        """Publish a human-readable decision for the visualiser's decision feed."""
        self.pub_decision.publish(String(data=text))

    def _count_non_blacklisted(self) -> int:
        if self.world is None:
            return 0
        return sum(1 for o in self.world.objects if not o.blacklisted)

    # --------------------------------------------------------------- callbacks
    def on_world(self, msg: WorldModel) -> None:
        self.world = msg

    def on_target(self, msg: Object) -> None:
        self.selected = msg

    def on_siglip(self, msg: Classification) -> None:
        self.siglip = msg
        self.siglip_stamp_s = self._now_s()

    def on_shape(self, msg: Classification) -> None:
        self.shape = msg
        self.shape_stamp_s = self._now_s()

    def on_advance(self, _: Empty) -> None:
        """Manual debug/dry-run override: force one transition along the chain."""
        idx = STATES.index(self.state)
        if self.state == "STORE_IN_TRAY":
            self._do_store_in_tray()
        elif self.state == "CLASSIFY":
            # force a PICK so the downstream chain can be exercised
            self.set_type = self.current_target.set_type if self.current_target else 0
            if not self.dry_pick:
                self._publish_pick(True)
            self._enter("PICK")
        elif self.state == "DUMP_ALL":
            self._enter("END")
        elif idx + 1 < len(STATES):
            self._enter(STATES[idx + 1])

    # ------------------------------------------------------------ state helper
    def _do_store_in_tray(self) -> None:
        """Tray bookkeeping + blacklist picked object, then branch (phase-aware)."""
        if self.set_type == 1:
            self.tray_shape += 1
        elif self.set_type == 2:
            self.tray_fruit += 1
        if self.current_target is not None:
            self._blacklist(self.current_target.id)   # picked -> never reselect
        self.current_target = None
        self.set_type = 0

        # Advance to Set2 phase once the Set1 quota is met.
        if self.phase == 1 and self.tray_shape >= self.shape_target_total:
            self.phase = 2
            self.get_logger().info("Set1 quota met -> phase 2 (Set2)")
            self._decide("PHASE 1->2 (Set1 quota met)")

        both_met = (
            self.tray_shape >= self.shape_target_total
            and self.tray_fruit >= self.fruit_target_total
        )
        if both_met and self.end_after_quota:
            self.get_logger().info("both quotas met (test mode) -> END")
            self._decide("END (quotas met)")
            self._enter("END")
        elif both_met:
            self._enter("DRIVE_TO_STORAGE")
        else:
            self._enter("SELECT_TARGET")

    # -------------------------------------------------------------- transitions
    def _step(self) -> None:
        """Condition-driven transition for the current state (one tick)."""
        if self.state == "SCAN":
            if self._count_non_blacklisted() >= 1:
                self._enter("SELECT_TARGET")

        elif self.state == "SELECT_TARGET":
            sel = self.selected
            if sel is not None and sel.id != 0:
                self.current_target = sel
                self._publish_goal(sel.x, sel.y)
                self._enter("APPROACH")
            elif self.select_timeout_sec > 0.0 and self._time_in_state() > self.select_timeout_sec:
                # No phase-appropriate target for a while: advance Set1->Set2, or END on Set2.
                self._advance_phase_or_end()
            # else: keep waiting (selector may still surface a target as the map fills)

        elif self.state == "APPROACH":
            tgt = self._lookup_object(self.current_target.id) if self.current_target else None
            if tgt is None:
                # target vanished from world model; reselect
                self._enter("SELECT_TARGET")
                return
            self._publish_goal(tgt.x, tgt.y)
            d = self._distance_to(tgt.x, tgt.y)
            if d is not None and d < self.approach_dist_m:
                self._enter("ALIGN")

        elif self.state == "ALIGN":
            self._step_align()

        elif self.state == "CLASSIFY":
            self._step_classify()

        elif self.state == "PICK":
            if self._time_in_state() >= self.pick_duration_sec:
                self._enter("STORE_IN_TRAY")

        elif self.state == "STORE_IN_TRAY":
            self._do_store_in_tray()

        elif self.state == "DRIVE_TO_STORAGE":
            self._publish_goal(self.storage_x, self.storage_y)
            d = self._distance_to(self.storage_x, self.storage_y)
            if d is not None and d < self.approach_dist_m:
                self._enter("ALIGN_OVER_BIN")

        elif self.state == "ALIGN_OVER_BIN":
            if self._time_in_state() >= self.align_settle_sec:
                self._enter("DUMP_ALL")

        elif self.state == "DUMP_ALL":
            if not self.dry_pick:
                self._publish_pick(False)   # release / dump tray
            if self._time_in_state() >= self.pick_duration_sec:
                self.get_logger().info("DUMP_ALL complete -> END")
                self._enter("END")

        # END: terminal, no transition

    def _commit_pick(self, set_type: int, label: str, detail: str) -> None:
        """Enter PICK for a confirmed target. In dry_pick mode, log instead of firing the arm."""
        self.set_type = set_type
        obj_id = self.current_target.id if self.current_target else 0
        if self.dry_pick:
            self.get_logger().info(
                f"[DRY PICK] picked id={obj_id} class='{label}' set={set_type} ({detail})"
            )
        else:
            self._publish_pick(True)
            self.get_logger().info(f"GATE PICK set{set_type} '{label}' ({detail})")
        self._decide(f"{'DRY-' if self.dry_pick else ''}PICK set{set_type} {label} #{obj_id} ({detail})")
        self._enter("PICK")

    def _advance_phase_or_end(self) -> None:
        """No phase-appropriate target left: Set1 phase -> Set2 phase, or Set2 phase -> END."""
        if self.phase == 1:
            self.phase = 2
            self.get_logger().info("phase 1 (Set1) exhausted -> phase 2 (Set2)")
            self._decide("PHASE 1->2 (Set1 exhausted)")
            self._enter("SELECT_TARGET")   # reset the timer and re-select for Set2
        else:
            self.get_logger().info("phase 2 (Set2) exhausted -> END")
            self._decide("END (Set2 exhausted)")
            self._enter("END")

    def _skip_target(self, reason: str) -> None:
        """Abandon the current target WITHOUT blacklisting (wrong phase; keep for later)."""
        obj_id = self.current_target.id if self.current_target else 0
        self.get_logger().info(f"skip id={obj_id} ({reason}) -> reselect (not blacklisted)")
        self._decide(f"SKIP #{obj_id} ({reason})")
        self.current_target = None
        self.set_type = 0
        self._enter("SELECT_TARGET")

    def _step_align(self) -> None:
        """Pulse+settle visual align: nudge the base briefly, let it FULLY STOP (inertia dissipates),
        then measure the object's body-cam base_link position (pose-independent) and grab if within
        tolerance (the gripper opening absorbs the residual). Otherwise nudge again. The base is
        stationary at grab time, so the object won't drift while the arm descends."""
        now = self._now_s()
        if self._time_in_state() > self.align_timeout_sec:
            self._drive(0.0, 0.0)
            self.get_logger().warn("ALIGN: timeout -> reselect")
            self._enter("SELECT_TARGET")
            return

        if self._align_phase == "pulse":
            if now - self._align_phase_start < self.align_pulse_sec:
                self._drive(self._pulse_vx, self._pulse_vy)     # brief nudge
            else:
                self._drive(0.0, 0.0)
                self._align_phase = "settle"
                self._align_phase_start = now
            return
        if self._align_phase == "settle":
            self._drive(0.0, 0.0)                                # let the heavy base fully stop
            if now - self._align_phase_start >= self.align_settle_pulse_sec:
                self._align_phase = "measure"
            return

        # measure (base is settled/stationary)
        want = 1 if self.phase == 1 else 2
        base = self._body_target_base(want)
        if base is None:
            self._drive(0.0, 0.0)
            self.get_logger().info("ALIGN: target lost -> reselect")
            self._enter("SELECT_TARGET")
            return
        obj_x, obj_y = base
        ex, ey = obj_x - self.grab_x, obj_y - self.grab_y
        if math.hypot(ex, ey) < self.align_tol:                 # settled + within gripper tol -> grab
            self._drive(0.0, 0.0)
            self.get_logger().info(f"ALIGN ok: obj=({obj_x:.3f},{obj_y:.3f}) err={math.hypot(ex,ey)*100:.1f}cm -> grab")
            self.siglip_stamp_s = None
            self.shape_stamp_s = None
            self._enter("CLASSIFY")
            return
        # Latch a nudge toward the error (stiction-boosted), per axis, with the forward safety.
        vx = max(-self.align_vmax, min(self.align_vmax, self.align_kp * ex))
        vy = max(-self.align_vmax, min(self.align_vmax, self.align_kp * ey))
        vx = 0.0 if abs(ex) <= self.align_tol else math.copysign(max(abs(vx), self.align_vmin), ex)
        vy = 0.0 if abs(ey) <= self.align_tol else math.copysign(max(abs(vy), self.align_vmin), ey)
        if obj_x <= self.grab_min_x and vx > 0.0:               # never push nearer than the blind limit
            vx = 0.0
        self._pulse_vx, self._pulse_vy = vx, vy
        self._align_phase = "pulse"
        self._align_phase_start = now
        self._drive(vx, vy)

    def _step_classify(self) -> None:
        """Phase-aware pick gate from fresh siglip + shape classifications.

        Pick order is enforced here: in phase 1 only a confirmed Set1 shape is picked; a Set2
        object met here is skipped (NOT blacklisted) so it can be picked in phase 2, and vice
        versa. Recognition/mapping itself is phase-independent and runs continuously upstream.
        """
        # SPATIAL gate: confirm from the CURRENT TARGET's own world-model track (its fused wide+body
        # identity at THIS track's position), not a frame-global /classification/shape or /siglip
        # that may belong to a neighbouring distractor. Set1 = the track already carries the shape
        # (YOLO, wide+body); Set2 = the track carries the SigLIP fruit in fruit_label. Re-look it up
        # fresh so close-range body/siglip observations during APPROACH are included.
        # Target = object of this phase's set_type nearest the grab point (robust to id churn).
        want = 1 if self.phase == 1 else 2
        tgt = self._front_target(want)
        if tgt is None:
            # target vanished before a decision -> RE-ACQUIRE, never blacklist a lost target.
            self.get_logger().info("CLASSIFY: target lost -> reselect")
            self.current_target = None
            self._enter("SELECT_TARGET")
            return
        self.current_target = tgt   # keep the FSM latched to the object actually in front

        set1_ok = bool(
            self.set1_label
            and tgt is not None
            and tgt.set_type == 1
            and tgt.class_label == self.set1_label
            and tgt.confidence >= self.pick_track_conf
        )
        # Set2 fruit: the track's SigLIP-derived fruit_label (only set when a face was actually read).
        set2_ok = bool(
            self.set2_label
            and tgt is not None
            and tgt.set_type == 2
            and tgt.fruit_label == self.set2_label
            and tgt.confidence >= self.pick_track_conf
        )

        if self.phase == 1:
            if set1_ok:
                self._commit_pick(1, tgt.class_label, f"track conf={tgt.confidence:.2f}")
                return
            if set2_ok:
                self._skip_target(f"set2 '{tgt.fruit_label}' during Set1 phase")
                return
        else:  # phase 2
            if set2_ok:
                self._commit_pick(2, tgt.fruit_label, f"track conf={tgt.confidence:.2f} (siglip fruit)")
                return
            if set1_ok:
                self._skip_target(f"stray set1 '{tgt.class_label}' during Set2 phase")
                return

        # No confident phase-appropriate decision: pass (blacklist) once we time out.
        if self._time_in_state() > self.classify_timeout_sec:
            obj_id = self.current_target.id if self.current_target else 0
            self.get_logger().warn(f"GATE PASS id={obj_id} (classify timeout) -> blacklist")
            self._decide(f"PASS #{obj_id} (classify timeout)")
            self._blacklist(obj_id)
            self.current_target = None
            self.set_type = 0
            self._enter("SELECT_TARGET")

    # --------------------------------------------------------------------- tick
    def tick(self) -> None:
        # 1) drive condition-based transitions
        self._step()

        # 2) publish mission state every tick
        msg = MissionState()
        msg.state = self.state
        msg.tray_shape_count = self.tray_shape
        msg.tray_fruit_count = self.tray_fruit
        msg.current_target_id = self.current_target.id if self.current_target else 0
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_state.publish(msg)

        # 3) publish current pick phase (target selector filters candidates by it)
        self.pub_phase.publish(Int8(data=int(self.phase)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionFsmNode()
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
