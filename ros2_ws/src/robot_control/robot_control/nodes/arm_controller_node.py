"""6-DOF arm controller: 좌표 → 분석 IK(arm_ik) → 서보각 → ArmCommand.

Inputs:
  /arm/target_pose       geometry_msgs/Pose     목표 끝점 (arm_base frame, meters)
  /arm/servo_correction  geometry_msgs/Vector3  본체 cam 비주얼서보 픽셀보정 (손목)
  /arm/gripper           std_msgs/Float32       그리퍼 서보각 0~180 (열림~닫힘, 동적 개폐)

Output:
  /arm_command           robot_interfaces/ArmCommand  서보각 0~180 (theta1..6 = ch0..ch5)

IK·영점/방향 보정(HOME_CMD/DIR)은 robot_control.arm_ik 에. 이 노드는 변환·발행만.
접근각/그리퍼/roll 은 파라미터 (추후 pose orientation / 별도 토픽으로 확장).
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose, Vector3
from rclpy.node import Node
from robot_interfaces.msg import ArmCommand
from std_msgs.msg import Float32

from robot_control import arm_ik


class ArmControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_controller_node")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("approach_deg", 0.0)     # 접근 피치(0=수평, 음수=아래로 집기)
        self.declare_parameter("wrist_roll", float(arm_ik.WRIST_ROLL_HOME))
        self.declare_parameter("gripper", float(arm_ik.GRIPPER_OPEN))

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.approach_deg = float(self.get_parameter("approach_deg").value)
        self.wrist_roll = float(self.get_parameter("wrist_roll").value)
        self.gripper = float(self.get_parameter("gripper").value)

        self.target_pose: Pose | None = None
        self.correction = Vector3()

        self.create_subscription(Pose, "/arm/target_pose", self.on_pose, 10)
        self.create_subscription(Vector3, "/arm/servo_correction", self.on_correction, 10)
        self.create_subscription(Float32, "/arm/gripper", self.on_gripper, 10)
        self.pub = self.create_publisher(ArmCommand, "/arm_command", 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"arm IK ready. HOME={arm_ik.HOME_CMD} approach={self.approach_deg}deg "
            f"grip(open={arm_ik.GRIPPER_OPEN}/close={arm_ik.GRIPPER_CLOSED})"
        )

    def on_pose(self, msg: Pose) -> None:
        self.target_pose = msg

    def on_correction(self, msg: Vector3) -> None:
        self.correction = msg

    def on_gripper(self, msg: Float32) -> None:
        """동적 그리퍼 개폐 (서보각 0~180). pick_sequencer_node 가 발행."""
        self.gripper = max(0.0, min(180.0, float(msg.data)))

    def _servo_angles(self) -> list[float]:
        """target_pose → 6 서보각(0~180). 없거나 도달불가면 HOME."""
        if self.target_pose is None:
            return list(arm_ik.HOME_CMD)
        x = self.target_pose.position.x * 100.0  # m → cm
        y = self.target_pose.position.y * 100.0
        z = self.target_pose.position.z * 100.0
        sol = arm_ik.ik_checked(x, y, z, self.approach_deg)
        if sol is None:
            self.get_logger().warn(
                f"target ({x:.1f},{y:.1f},{z:.1f})cm 도달불가 → HOME 유지",
                throttle_duration_sec=2.0,
            )
            return list(arm_ik.HOME_CMD)
        servo = [arm_ik.servo_cmd(j, sol[j]) for j in range(4)]
        return servo + [self.wrist_roll, self.gripper]

    def tick(self) -> None:
        a = self._servo_angles()
        # 비주얼 서보 보정: 손목 pitch(ch3)/roll(ch4)에 작은 픽셀 보정
        a[3] = max(0.0, min(180.0, a[3] + self.correction.x))
        a[4] = max(0.0, min(180.0, a[4] + self.correction.y))

        cmd = ArmCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "arm_base"
        cmd.theta1, cmd.theta2, cmd.theta3, cmd.theta4, cmd.theta5, cmd.theta6 = (
            int(round(max(0.0, min(180.0, v)))) for v in a
        )
        self.pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmControllerNode()
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
