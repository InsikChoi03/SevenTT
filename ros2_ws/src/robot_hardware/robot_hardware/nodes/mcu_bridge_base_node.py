"""Serial bridge to Arduino_base.

Sends 4 wheel speed commands to base MCU, reads back odometry/telemetry.

Outbound protocol (one line per command, '\n' terminated):
    <BASE,fl,fr,rl,rr>
where each value is wheel linear speed in m/s, signed, 3 decimals.

Inbound (Arduino → host, line per packet):
    <ODOM,fl,fr,rl,rr,t_ms>   wheel speeds and timestamp ms
"""
from __future__ import annotations

import threading

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class McuBridgeBaseNode(Node):
    def __init__(self) -> None:
        super().__init__("mcu_bridge_base_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("reconnect_sec", 2.0)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.reconnect_sec = float(self.get_parameter("reconnect_sec").value)

        self.ser: serial.Serial | None = None
        self.ser_lock = threading.Lock()

        self.sub = self.create_subscription(
            Float32MultiArray, "/base/wheel_speeds", self.on_wheel_speeds, 10
        )
        self.pub_odom = self.create_publisher(Float32MultiArray, "/base/wheel_odom", 10)

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
            self.get_logger().warn(f"serial open failed ({e}); will retry on next command", throttle_duration_sec=5.0)

    def on_wheel_speeds(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 4:
            self.get_logger().warn(f"expected 4 wheel speeds, got {len(msg.data)}")
            return
        fl, fr, rl, rr = msg.data
        line = f"<BASE,{fl:.3f},{fr:.3f},{rl:.3f},{rr:.3f}>\n"
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
                self._handle_line(line.strip().decode("ascii", errors="ignore"))

    def _handle_line(self, line: str) -> None:
        if not line.startswith("<ODOM,") or not line.endswith(">"):
            return
        try:
            payload = line[6:-1]
            parts = payload.split(",")
            fl, fr, rl, rr = (float(p) for p in parts[:4])
        except (ValueError, IndexError):
            return
        out = Float32MultiArray()
        out.data = [fl, fr, rl, rr]
        self.pub_odom.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = McuBridgeBaseNode()
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
