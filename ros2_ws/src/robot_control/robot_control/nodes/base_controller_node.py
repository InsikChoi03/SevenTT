"""Mecanum base inverse kinematics: (vx, vy, omega) -> 4 wheel speeds.

Wheel convention (top view, X forward / Y left):
    FL --- FR
     |     |
    RL --- RR
Each wheel signed speed is the linear speed at wheel tread (m/s);
the bridge node converts to PWM via wheel radius + max RPM calibration.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import BaseCommand
from std_msgs.msg import Float32MultiArray


class BaseControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("base_controller_node")

        # Wheelbase half-width (lx) and half-track (ly), meters
        self.declare_parameter("lx", 0.10)
        self.declare_parameter("ly", 0.10)
        # Watchdog: zero wheels if no command in window
        self.declare_parameter("cmd_timeout_sec", 0.5)

        self.lx = float(self.get_parameter("lx").value)
        self.ly = float(self.get_parameter("ly").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)

        self.last_cmd_time = self.get_clock().now()
        self.last_cmd = (0.0, 0.0, 0.0)

        self.sub = self.create_subscription(BaseCommand, "/base_command", self.on_cmd, 10)
        self.pub = self.create_publisher(Float32MultiArray, "/base/wheel_speeds", 10)
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

        self.get_logger().info(f"base IK lx={self.lx} ly={self.ly} watchdog={self.cmd_timeout}s")

    def on_cmd(self, msg: BaseCommand) -> None:
        self.last_cmd = (msg.vx, msg.vy, msg.omega)
        self.last_cmd_time = self.get_clock().now()

    def tick(self) -> None:
        # Watchdog: zero output if no recent command
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            vx, vy, omega = 0.0, 0.0, 0.0
        else:
            vx, vy, omega = self.last_cmd

        # Mecanum inverse kinematics (wheel linear speeds, m/s)
        k = self.lx + self.ly
        v_fl = vx - vy - k * omega
        v_fr = vx + vy + k * omega
        v_rl = vx + vy - k * omega
        v_rr = vx - vy + k * omega

        # Clamp |v| <= 1 m/s for safety until calibration available
        v_fl, v_fr, v_rl, v_rr = (max(-1.0, min(1.0, w)) for w in (v_fl, v_fr, v_rl, v_rr))

        out = Float32MultiArray()
        out.data = [float(v_fl), float(v_fr), float(v_rl), float(v_rr)]
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseControllerNode()
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
