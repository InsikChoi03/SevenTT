"""Combined Arduino serial bridge with competition-start safety gating.

The base, 2R arm, start button, status LEDs, and opening profile output share one
Arduino UNO on ``/dev/ttyUSB0``.  This node is therefore the final actuator gate:
wheel commands are replaced with zero and arm commands are discarded until the
latched competition state reaches RUNNING.
"""
from __future__ import annotations

import threading
import time

import rclpy
import serial
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import MissionState
from std_msgs.msg import Float32MultiArray, String

from robot_hardware.competition_state import RUNNING, CompetitionStateMachine


COMPETITION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class McuBridgeBaseNode(Node):
    """Bridge the shared MCU and own the authoritative competition start state."""

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
        self.declare_parameter("profile_lift_ms", 2000)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.reconnect_sec = float(self.get_parameter("reconnect_sec").value)
        self.publish_encoder_odom = bool(self.get_parameter("publish_encoder_odom").value)
        self.publish_heartbeat_as_odom = bool(
            self.get_parameter("publish_heartbeat_as_odom").value
        )
        self.odom_deadband = float(self.get_parameter("odom_deadband_mps").value)
        self.odom_scale = float(self.get_parameter("odom_scale").value)
        wheel_scales = [float(v) for v in self.get_parameter("odom_wheel_scales").value]
        self.odom_wheel_scales = (
            wheel_scales if len(wheel_scales) == 4 else [1.0, 1.0, 1.0, 1.0]
        )
        self.profile_lift_ms = max(
            0, min(10000, int(self.get_parameter("profile_lift_ms").value))
        )

        self.ser: serial.Serial | None = None
        self.ser_lock = threading.Lock()
        self.competition = CompetitionStateMachine()
        self._mission_state = "STANDBY"
        self._profile_started = False
        self._profile_off_sent = False
        self._profile_off_deadline = 0.0

        self.pub_state = self.create_publisher(String, "/competition/state", COMPETITION_QOS)
        self.pub_odom = self.create_publisher(Float32MultiArray, "/base/wheel_odom", 10)
        self.create_subscription(
            Float32MultiArray, "/base/wheel_speeds", self.on_wheel_speeds, 10
        )
        self.create_subscription(Float32MultiArray, "/arm2r/target", self.on_arm_target, 10)
        self.create_subscription(MissionState, "/mission_state", self.on_mission_state, 10)

        self.add_on_set_parameters_callback(self._on_params)
        self.status_timer = self.create_timer(1.0, self._status_tick)
        self.profile_timer = self.create_timer(0.05, self._profile_tick)

        with self.ser_lock:
            self._open_serial_locked()
        self._publish_competition_state(send_status=True)

        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info(
            f"port={self.port} baud={self.baud} encoder_odom={self.publish_encoder_odom} "
            f"hb_as_odom={self.publish_heartbeat_as_odom} "
            f"deadband={self.odom_deadband:.4f}m/s odom_scale={self.odom_scale:.4f} "
            f"wheel_scales={self.odom_wheel_scales} competition=STANDBY "
            f"profile_lift_ms={self.profile_lift_ms}"
        )

    def _on_params(self, params) -> SetParametersResult:
        for param in params:
            if param.name == "odom_scale":
                self.odom_scale = float(param.value)
            elif param.name == "odom_wheel_scales":
                vals = [float(v) for v in param.value]
                if len(vals) != 4:
                    return SetParametersResult(
                        successful=False,
                        reason="odom_wheel_scales must have 4 values",
                    )
                self.odom_wheel_scales = vals
            elif param.name == "odom_deadband_mps":
                self.odom_deadband = float(param.value)
            elif param.name == "publish_encoder_odom":
                self.publish_encoder_odom = bool(param.value)
            elif param.name == "publish_heartbeat_as_odom":
                self.publish_heartbeat_as_odom = bool(param.value)
            elif param.name == "profile_lift_ms":
                value = int(param.value)
                if value < 0 or value > 10000:
                    return SetParametersResult(
                        successful=False,
                        reason="profile_lift_ms must be in [0, 10000]",
                    )
                self.profile_lift_ms = value
        return SetParametersResult(successful=True)

    # -------------------------------------------------------------- serial I/O
    def _open_serial_locked(self) -> bool:
        if self.ser is not None:
            return True
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"serial opened: {self.port}")
            return True
        except (serial.SerialException, OSError) as exc:
            self.ser = None
            self.get_logger().warn(
                f"serial open failed ({exc}); retrying every {self.reconnect_sec:.1f}s",
                throttle_duration_sec=5.0,
            )
            return False

    def _close_serial_locked(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
        self.ser = None

    def _write_line(self, line: str, context: str) -> bool:
        with self.ser_lock:
            if not self._open_serial_locked():
                return False
            try:
                self.ser.write((line + "\n").encode("ascii"))
                return True
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warn(f"{context} write failed ({exc}); closing")
                self._close_serial_locked()
                return False

    def _read_loop(self) -> None:
        buf = b""
        while rclpy.ok():
            with self.ser_lock:
                ser = self.ser
            if ser is None:
                time.sleep(max(0.1, self.reconnect_sec))
                continue
            try:
                data = ser.read(64)
            except (serial.SerialException, OSError):
                with self.ser_lock:
                    if self.ser is ser:
                        self._close_serial_locked()
                continue
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle_line(line.strip().decode("ascii", errors="ignore"))

    # ------------------------------------------------------- competition state
    def _publish_competition_state(self, *, send_status: bool) -> None:
        state = self.competition.state
        self.pub_state.publish(String(data=state))
        if send_status:
            self._write_line(f"<STATUS,{state}>", "competition STATUS")

    def _status_tick(self) -> None:
        # Periodic resend also reconnects the port if the Arduino was unplugged/rebooted.
        self._write_line(
            f"<STATUS,{self.competition.state}>",
            "periodic competition STATUS",
        )

    def _handle_start(self, line: str) -> None:
        try:
            sequence = int(line[7:-1])
        except (TypeError, ValueError):
            self.get_logger().warn(f"invalid START packet ignored: {line}")
            return
        old_state = self.competition.state
        new_state = self.competition.accept_start(sequence)
        if new_state is None:
            self.get_logger().info(
                f"START sequence ignored: n={sequence} state={old_state}"
            )
            return
        self.get_logger().info(
            f"competition transition {old_state} -> {new_state} (START,{sequence})"
        )
        self._publish_competition_state(send_status=True)
        self._maybe_start_profile()

    # ----------------------------------------------------------- safety gates
    def on_wheel_speeds(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 4:
            self.get_logger().warn(f"expected 4 wheel speeds, got {len(msg.data)}")
            return
        values = msg.data if self.competition.state == RUNNING else (0.0, 0.0, 0.0, 0.0)
        fl, fr, rl, rr = values
        self._write_line(
            f"<BASE,{fl:.3f},{fr:.3f},{rl:.3f},{rr:.3f}>",
            "base command",
        )

    def on_arm_target(self, msg: Float32MultiArray) -> None:
        if self.competition.state != RUNNING:
            return
        if len(msg.data) < 3:
            self.get_logger().warn(
                f"arm target expected >=3 (shoulder,wrist,gripper), got {len(msg.data)}"
            )
            return
        shoulder, wrist, gripper = (int(round(v)) for v in msg.data[:3])
        self._write_line(
            f"<ARM,{shoulder},{wrist},{gripper}>",
            "arm command",
        )

    # ------------------------------------------------------- one-shot profile
    def on_mission_state(self, msg: MissionState) -> None:
        self._mission_state = str(msg.state)
        self._maybe_start_profile()

    def _maybe_start_profile(self) -> None:
        if (
            self._profile_started
            or self.profile_lift_ms <= 0
            or self.competition.state != RUNNING
            or self._mission_state != "OPENING"
        ):
            return
        if self._write_line(f"<LIFT,{self.profile_lift_ms}>", "profile LIFT on"):
            self._profile_started = True
            self._profile_off_sent = False
            self._profile_off_deadline = time.monotonic() + self.profile_lift_ms / 1000.0
            self.get_logger().info(
                f"opening profile fired once: D13 HIGH for {self.profile_lift_ms} ms"
            )

    def _profile_tick(self) -> None:
        if (
            not self._profile_started
            or self._profile_off_sent
            or time.monotonic() < self._profile_off_deadline
        ):
            return
        if self._write_line("<LIFT,0>", "profile LIFT off"):
            self._profile_off_sent = True
            self.get_logger().info("opening profile OFF resent by Jetson")

    # -------------------------------------------------------------- telemetry
    def _handle_line(self, line: str) -> None:
        if not line.endswith(">"):
            return
        if line.startswith("<START,"):
            self._handle_start(line)
            return
        # <ODOM,...> is encoder measurement. <HB,...> is command echo.
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
            fl, fr, rl, rr = (float(part) for part in parts[:4])
        except (ValueError, IndexError):
            return
        vals = [
            fl * self.odom_scale * self.odom_wheel_scales[0],
            fr * self.odom_scale * self.odom_wheel_scales[1],
            rl * self.odom_scale * self.odom_wheel_scales[2],
            rr * self.odom_scale * self.odom_wheel_scales[3],
        ]
        if self.odom_deadband > 0.0:
            vals = [0.0 if abs(value) < self.odom_deadband else value for value in vals]
        self.pub_odom.publish(Float32MultiArray(data=vals))

    def destroy_node(self):
        # Best-effort hard stop on process shutdown. Arduino's own watchdog/10 s lift limit remain.
        self._write_line("<BASE,0.000,0.000,0.000,0.000>", "shutdown base stop")
        self._write_line("<LIFT,0>", "shutdown profile stop")
        with self.ser_lock:
            self._close_serial_locked()
        return super().destroy_node()


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
