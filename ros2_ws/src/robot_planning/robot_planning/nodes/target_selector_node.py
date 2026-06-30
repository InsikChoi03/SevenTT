"""Target selector: from world model, pick the next object to pursue.

Strategy (rulebook §6, §7, §9):
  • Skip blacklisted objects (passed earlier or already picked) — always.
  • set1 target: class_label == set1_label  → base = 10
  • set2 target: class_label == set2_label  → base = 20
  • unknown (set_type==0, top-cam candidate): base = explore_base (default 5)
        → go investigate; the FSM (siglip/shape gates) will classify it and
          either pick it or add it to the blacklist.
  • storage_flag (set_type==3) and other non-target objects: skipped.
  • Score = base / (distance_m + 0.5) * max(0.01, confidence)
  • Publish best on /selected_target. Publish empty Object (id=0) if none.

Why explore_base < set1 base: a CONFIRMED today-target object (set1=10, set2=20)
always outranks an UNKNOWN candidate at the same distance/confidence, so the robot
commits to known targets first and only wanders off to investigate unknowns when no
confirmed target is currently visible.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Object, WorldModel


SET1_POINTS = 10.0
SET2_POINTS = 20.0


class TargetSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("target_selector_node")

        # Today's announced targets (rulebook §6.1, §7.3). Updated via param later.
        self.declare_parameter("set1_label", "")      # e.g. "icosahedron"
        self.declare_parameter("set2_label", "")      # e.g. "apple"
        self.declare_parameter("rate_hz", 2.0)
        # Base score for unknown candidates. Kept below set1 (10) so confirmed
        # today-targets always win over unexplored objects.
        self.declare_parameter("explore_base", 5.0)

        self.set1_label = str(self.get_parameter("set1_label").value)
        self.set2_label = str(self.get_parameter("set2_label").value)
        rate = float(self.get_parameter("rate_hz").value)
        self.explore_base = float(self.get_parameter("explore_base").value)

        self.world: WorldModel | None = None

        self.create_subscription(WorldModel, "/world_model", self.on_world, 10)
        self.pub = self.create_publisher(Object, "/selected_target", 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"set1='{self.set1_label}' set2='{self.set2_label}' "
            f"explore_base={self.explore_base} rate={rate}Hz"
        )

    def on_world(self, msg: WorldModel) -> None:
        self.world = msg

    def _score(self, obj: Object, rx: float, ry: float) -> float | None:
        # Blacklisted objects (passed or already picked) are always skipped.
        if obj.blacklisted:
            return None
        if obj.set_type == 1:
            if not self.set1_label or obj.class_label != self.set1_label:
                return None
            base = SET1_POINTS
        elif obj.set_type == 2:
            if not self.set2_label or obj.class_label != self.set2_label:
                return None
            base = SET2_POINTS
        elif obj.set_type == 0:
            # Unknown top-cam candidate: investigate it. Lower base than a
            # confirmed target so known targets outrank unexplored ones.
            base = self.explore_base
        else:
            # storage_flag (set_type==3) and anything else: not a pick target.
            return None
        d = math.hypot(obj.x - rx, obj.y - ry)
        return (base / (d + 0.5)) * max(0.01, obj.confidence)

    def tick(self) -> None:
        if self.world is None:
            return
        rx, ry = self.world.robot_x, self.world.robot_y
        best: tuple[float, Object] | None = None
        for obj in self.world.objects:
            s = self._score(obj, rx, ry)
            if s is None:
                continue
            if best is None or s > best[0]:
                best = (s, obj)

        out = best[1] if best is not None else Object()
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetSelectorNode()
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
