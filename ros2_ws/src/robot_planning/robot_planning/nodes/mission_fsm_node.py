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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_interfaces.msg import Classification, MissionState, Object, WorldModel
from std_msgs.msg import Bool, Empty, Int8, String, UInt64


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
        self.declare_parameter("approach_dist_m", 0.25)
        self.declare_parameter("align_settle_sec", 1.0)
        self.declare_parameter("classify_timeout_sec", 2.0)
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
        self.approach_dist_m = float(self.get_parameter("approach_dist_m").value)
        self.align_settle_sec = float(self.get_parameter("align_settle_sec").value)
        self.classify_timeout_sec = float(self.get_parameter("classify_timeout_sec").value)
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
        self.create_subscription(Empty, "/state_advance", self.on_advance, 10)

        self.pub_state = self.create_publisher(MissionState, "/mission_state", 10)
        self.pub_blacklist = self.create_publisher(UInt64, "/world_model/blacklist_add", 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/base/goal_pose", 10)
        self.pub_pick = self.create_publisher(Bool, "/arm/pick_trigger", 10)
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
            if self._time_in_state() >= self.align_settle_sec:
                # clear stale classifications so CLASSIFY only trusts fresh ones
                self.siglip_stamp_s = None
                self.shape_stamp_s = None
                self._enter("CLASSIFY")

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

    def _step_classify(self) -> None:
        """Phase-aware pick gate from fresh siglip + shape classifications.

        Pick order is enforced here: in phase 1 only a confirmed Set1 shape is picked; a Set2
        object met here is skipped (NOT blacklisted) so it can be picked in phase 2, and vice
        versa. Recognition/mapping itself is phase-independent and runs continuously upstream.
        """
        enter = self.state_enter_s
        siglip = self.siglip if (self.siglip_stamp_s is not None and self.siglip_stamp_s >= enter) else None
        shape = self.shape if (self.shape_stamp_s is not None and self.shape_stamp_s >= enter) else None

        # Set1 shape confirmation. Anti-mispick cube rule kept as belt-and-suspenders even though
        # the 5-class YOLO now separates cube (class 0) from fruit_photo_cube (excluded from the
        # shape stream): if the target is the plain cube, still require siglip to see no fruit face.
        set1_ok = bool(
            self.set1_label
            and shape is not None
            and shape.label == self.set1_label
            and shape.confidence >= self.conf_threshold
            and (self.set1_label != "cube" or (siglip is not None and not siglip.image_face_visible))
        )
        # Set2 fruit confirmation: today's fruit, a picture face visible, confident.
        set2_ok = bool(
            self.set2_label
            and siglip is not None
            and siglip.label == self.set2_label
            and siglip.image_face_visible
            and siglip.confidence >= self.conf_threshold
        )

        if self.phase == 1:
            if set1_ok:
                self._commit_pick(1, shape.label, f"conf={shape.confidence:.2f}")
                return
            if set2_ok:
                self._skip_target(f"set2 '{siglip.label}' during Set1 phase")
                return
        else:  # phase 2
            if set2_ok:
                self._commit_pick(2, siglip.label, f"conf={siglip.confidence:.2f} face_visible")
                return
            if set1_ok:
                self._skip_target(f"stray set1 '{shape.label}' during Set2 phase")
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
