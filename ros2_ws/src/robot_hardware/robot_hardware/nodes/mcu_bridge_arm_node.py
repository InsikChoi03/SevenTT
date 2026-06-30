"""Serial bridge to Arduino_arm (PCA9685 → MG996R x6).

Outbound protocol:
    <ARM,t1,t2,t3,t4,t5,t6>\n
where each ti is integer degrees, 0~180.

Inbound (optional ack/error):
    <ARMACK,t1,t2,t3,t4,t5,t6>\n   echo current joint targets
    <ARMERR,code>\n
"""
from __future__ import annotations

import threading

import rclpy
import serial
from rclpy.node import Node
from robot_interfaces.msg import ArmCommand


def clamp_deg(v: int) -> int:
    return max(0, min(180, int(v)))


class McuBridgeArmNode(Node):
    def __init__(self) -> None:
        super().__init__("mcu_bridge_arm_node")

        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("reconnect_sec", 2.0)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.reconnect_sec = float(self.get_parameter("reconnect_sec").value)

        self.ser: serial.Serial | None = None
        self.ser_lock = threading.Lock()

        self.sub = self.create_subscription(ArmCommand, "/arm_command", self.on_arm_cmd, 10)

        self._open_serial()
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info(f"port={self.port} baud={self.baud}")

    def _open_serial(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"serial opened: {self.port}")
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().warn(f"serial open failed ({e}); will retry", throttle_duration_sec=5.0)

    def on_arm_cmd(self, msg: ArmCommand) -> None:
        t = [clamp_deg(msg.theta1), clamp_deg(msg.theta2), clamp_deg(msg.theta3),
             clamp_deg(msg.theta4), clamp_deg(msg.theta5), clamp_deg(msg.theta6)]
        line = f"<ARM,{t[0]},{t[1]},{t[2]},{t[3]},{t[4]},{t[5]}>\n"
        with self.ser_lock:
            if self.ser is None:
                self._open_serial()
            if self.ser is None:
                return
            try:
                self.ser.write(line.encode("ascii"))
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f"serial write failed ({e}); closing")
                try:
                    self.ser.close()
                finally:
                    self.ser = None

    def _read_loop(self) -> None:
        buf = b""
        while rclpy.ok():
            with self.ser_lock:
                ser = self.ser
            if ser is None:
                rclpy.spin_once(self, timeout_sec=self.reconnect_sec)
                continue
            try:
                data = ser.read(64)
            except (serial.SerialException, OSError):
                with self.ser_lock:
                    self.ser = None
                continue
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.strip().decode("ascii", errors="ignore")
                if text.startswith("<ARMERR,"):
                    self.get_logger().warn(f"arm MCU: {text}")
                elif text.startswith("<ARMACK,"):
                    self.get_logger().debug(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = McuBridgeArmNode()
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
