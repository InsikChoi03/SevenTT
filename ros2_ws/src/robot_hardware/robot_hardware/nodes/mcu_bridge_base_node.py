"""Serial bridge to Arduino_base.

Sends 4 wheel speed commands to base MCU, reads back odometry/telemetry.

Outbound protocol (one line per command, '\n' terminated):
    <BASE,fl,fr,rl,rr>
where each value is wheel linear speed in m/s, signed, 3 decimals.

Inbound (Arduino → host, line per packet):
    <ODOM,fl,fr,rl,rr,t_ms>   encoder wheel speeds + timestamp ms (when encoders exist)
    <HB,fl,fr,rl,rr>          heartbeat = the COMMANDED wheel speeds echoed at 5 Hz. The current
                              base (PCA9685+MX1508) has NO encoders, so this is all we get.
Both are republished on /base/wheel_odom (m/s). With only <HB>, the localizer dead-reckons
OPEN-LOOP from commands (directions correct; magnitude approximate until an encoder or the
object-landmark correction tightens it). Without this the localizer never moves -> the robot
cannot track its own heading/position and drives the wrong way ("상하좌우 모름").
"""
from __future__ import annotations

import threading

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Range
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
        # 2R ARM passthrough on the SAME combined board (ttyUSB0): the firmware takes both
        # <BASE,...> and <ARM,shoulder,wrist,gripper> on one port, so the arm can't have its own
        # bridge here. /arm2r/target = [shoulder, wrist, gripper] servo degrees.
        self.sub_arm = self.create_subscription(
            Float32MultiArray, "/arm2r/target", self.on_arm_target, 10
        )
        self.pub_odom = self.create_publisher(Float32MultiArray, "/base/wheel_odom", 10)
        self.pub_range = self.create_publisher(Range, "/ultrasonic/range", 10)   # front HC-SR04

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

    def on_arm_target(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warn(f"arm target expected >=3 (shoulder,wrist,gripper), got {len(msg.data)}")
            return
        sh, wr, gr = (int(round(v)) for v in msg.data[:3])
        line = f"<ARM,{sh},{wr},{gr}>\n"
        with self.ser_lock:
            if self.ser is None:
                self._open_serial()
            if self.ser is None:
                return
            try:
                self.ser.write(line.encode("ascii"))
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f"arm serial write failed ({e}); closing")
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
        if not line.endswith(">"):
            return
        # <US,cm> front HC-SR04 (cm; -1 = no echo / beyond range) -> sensor_msgs/Range (metres).
        if line.startswith("<US,"):
            try:
                cm = int(line[4:-1])
            except ValueError:
                return
            r = Range()
            r.header.stamp = self.get_clock().now().to_msg()
            r.header.frame_id = "ultrasonic_front"
            r.radiation_type = Range.ULTRASOUND
            r.field_of_view = 0.26           # ~15 deg HC-SR04 beam
            r.min_range = 0.02
            r.max_range = 2.0
            r.range = float("inf") if cm < 0 else cm / 100.0
            self.pub_range.publish(r)
            return
        # <ODOM,fl,fr,rl,rr,t_ms> (encoders) OR <HB,fl,fr,rl,rr> (commanded-speed echo, no encoders).
        if line.startswith("<ODOM,"):
            payload = line[6:-1]
        elif line.startswith("<HB,"):
            payload = line[4:-1]
        else:
            return
        try:
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
