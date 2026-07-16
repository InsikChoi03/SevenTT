"""Serial bridge to Arduino_base.

Sends 4 wheel speed commands to base MCU, reads back odometry/telemetry.

Outbound protocol (one line per command, '\n' terminated):
    <BASE,fl,fr,rl,rr>
where each value is wheel linear speed in m/s, signed, 3 decimals.

Inbound (Arduino → host, line per packet):
    <ODOM,fl,fr,rl,rr,t_ms>   encoder wheel speeds + timestamp ms (when encoders exist)
    <HB,fl,fr,rl,rr>          heartbeat = the COMMANDED wheel speeds echoed at 5 Hz.

Only <ODOM> is published on /base/wheel_odom by default. <HB> is a command echo, not a
measurement, so using it as odometry can make the pose drift while the robot is physically still.
Set publish_heartbeat_as_odom=true only for deliberate open-loop fallback tests.
"""
from __future__ import annotations

import threading
import time

import rclpy
import serial
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class McuBridgeBaseNode(Node):
    def __init__(self) -> None:
        super().__init__("mcu_bridge_base_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("reconnect_sec", 2.0)
        self.declare_parameter("publish_encoder_odom", True)
        self.declare_parameter("publish_heartbeat_as_odom", False)
        self.declare_parameter("odom_deadband_mps", 0.005)
        self.declare_parameter("odom_scale", 1.0)
        self.declare_parameter("odom_wheel_scales", [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("startup_lift_ms", 0)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.reconnect_sec = float(self.get_parameter("reconnect_sec").value)
        self.publish_encoder_odom = bool(self.get_parameter("publish_encoder_odom").value)
        self.publish_heartbeat_as_odom = bool(self.get_parameter("publish_heartbeat_as_odom").value)
        self.odom_deadband = float(self.get_parameter("odom_deadband_mps").value)
        self.odom_scale = float(self.get_parameter("odom_scale").value)
        wheel_scales = [float(v) for v in self.get_parameter("odom_wheel_scales").value]
        self.odom_wheel_scales = wheel_scales if len(wheel_scales) == 4 else [1.0, 1.0, 1.0, 1.0]
        self.startup_lift_ms = max(0, min(10000, int(self.get_parameter("startup_lift_ms").value)))
        self._startup_lift_sent = False
        self._startup_lift_timer = None
        self._serial_opened_at = 0.0

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

        self._open_serial()
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        self.add_on_set_parameters_callback(self._on_params)
        if self.startup_lift_ms > 0:
            self._startup_lift_timer = self.create_timer(0.2, self._startup_lift_tick)

        self.get_logger().info(
            f"port={self.port} baud={self.baud} encoder_odom={self.publish_encoder_odom} "
            f"hb_as_odom={self.publish_heartbeat_as_odom} deadband={self.odom_deadband:.4f}m/s "
            f"odom_scale={self.odom_scale:.4f} wheel_scales={self.odom_wheel_scales} "
            f"startup_lift_ms={self.startup_lift_ms}"
        )

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "odom_scale":
                self.odom_scale = float(p.value)
            elif p.name == "odom_wheel_scales":
                vals = [float(v) for v in p.value]
                if len(vals) != 4:
                    return SetParametersResult(
                        successful=False,
                        reason="odom_wheel_scales must have 4 values",
                    )
                self.odom_wheel_scales = vals
            elif p.name == "odom_deadband_mps":
                self.odom_deadband = float(p.value)
            elif p.name == "publish_encoder_odom":
                self.publish_encoder_odom = bool(p.value)
            elif p.name == "publish_heartbeat_as_odom":
                self.publish_heartbeat_as_odom = bool(p.value)
        return SetParametersResult(successful=True)

    def _open_serial(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self._serial_opened_at = time.monotonic()
            self.get_logger().info(f"serial opened: {self.port}")
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().warn(f"serial open failed ({e}); will retry on next command", throttle_duration_sec=5.0)

    def _startup_lift_tick(self) -> None:
        if self._startup_lift_sent or self.startup_lift_ms <= 0:
            if self._startup_lift_timer is not None:
                self._startup_lift_timer.cancel()
            return
        with self.ser_lock:
            if self.ser is None:
                self._open_serial()
            self._send_startup_lift_locked()
        if self._startup_lift_sent and self._startup_lift_timer is not None:
            self._startup_lift_timer.cancel()

    def _send_startup_lift_locked(self) -> None:
        if self._startup_lift_sent or self.startup_lift_ms <= 0 or self.ser is None:
            return
        if time.monotonic() - self._serial_opened_at < 2.0:
            return
        line = f"<LIFT,{self.startup_lift_ms}>\n"
        try:
            self.ser.write(line.encode("ascii"))
            self._startup_lift_sent = True
            self.get_logger().info(f"startup lift command sent: {self.startup_lift_ms} ms")
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f"startup lift write failed ({e}); closing")
            try:
                self.ser.close()
            finally:
                self.ser = None

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
            self._send_startup_lift_locked()
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
            self._send_startup_lift_locked()
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
        # <ODOM,fl,fr,rl,rr,t_ms> is encoder measurement. <HB,...> is command echo.
        if line.startswith("<ODOM,"):
            if not self.publish_encoder_odom:
                return
            payload = line[6:-1]
        elif line.startswith("<HB,"):
            if not self.publish_heartbeat_as_odom:
                return
            payload = line[4:-1]
        else:
            return
        try:
            parts = payload.split(",")
            fl, fr, rl, rr = (float(p) for p in parts[:4])
        except (ValueError, IndexError):
            return
        vals = [
            fl * self.odom_scale * self.odom_wheel_scales[0],
            fr * self.odom_scale * self.odom_wheel_scales[1],
            rl * self.odom_scale * self.odom_wheel_scales[2],
            rr * self.odom_scale * self.odom_wheel_scales[3],
        ]
        if self.odom_deadband > 0.0:
            vals = [0.0 if abs(v) < self.odom_deadband else v for v in vals]
        out = Float32MultiArray()
        out.data = vals
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
