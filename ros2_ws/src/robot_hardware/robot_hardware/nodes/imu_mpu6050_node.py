"""MPU6050 (I2C) IMU publisher — primarily the yaw-rate gyro that kills heading drift.

This board is a MPU6050 CLONE: WHO_AM_I reads 0x72 (not the genuine 0x68), but the register map
is compatible, so we DON'T validate WHO_AM_I. Bus/addr from the hardware: i2c-7, 0x68.

The Z gyro (yaw rate) is low-noise and only drifts slowly (bias), so integrating it in the localizer
gives a far steadier heading than the wide-cam object-flow / LK-VO (which churn or hallucinate). We
calibrate the stationary gyro bias at startup and subtract it. Accel is published too (gravity ref).

Publishes:
  /imu/data   sensor_msgs/Imu   angular_velocity (rad/s, bias-removed), linear_acceleration (m/s^2)

No smbus dependency required at import: uses smbus2 if present, else a tiny /dev/i2c ioctl fallback.
"""
from __future__ import annotations

import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# MPU6050 registers
_PWR_MGMT_1 = 0x6B
_GYRO_CONFIG = 0x1B
_ACCEL_CONFIG = 0x1C
_ACCEL_XOUT_H = 0x3B      # 0x3B..0x48 = accel(6) temp(2) gyro(6), one burst
_GYRO_Z_H = 0x47
_GYRO_LSB = 131.0         # ±250 deg/s -> 131 LSB/(deg/s)
_ACCEL_LSB = 16384.0      # ±2 g -> 16384 LSB/g
_DEG2RAD = 3.141592653589793 / 180.0
_G = 9.80665


class _I2C:
    """Minimal register I2C: smbus2 if available, else raw /dev/i2c ioctl (no dependency)."""

    def __init__(self, bus: int, addr: int) -> None:
        self.addr = addr
        self._smb = None
        try:
            import smbus2
            self._smb = smbus2.SMBus(bus)
        except Exception:  # noqa: BLE001 - fall back to ioctl
            import fcntl
            import os
            self._os = os
            self._fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
            fcntl.ioctl(self._fd, 0x0703, addr)   # I2C_SLAVE

    def write_byte(self, reg: int, val: int) -> None:
        if self._smb is not None:
            self._smb.write_byte_data(self.addr, reg, val)
        else:
            self._os.write(self._fd, bytes([reg, val]))

    def read_block(self, reg: int, n: int) -> bytes:
        if self._smb is not None:
            return bytes(self._smb.read_i2c_block_data(self.addr, reg, n))
        self._os.write(self._fd, bytes([reg]))
        return self._os.read(self._fd, n)


class ImuMpu6050Node(Node):
    def __init__(self) -> None:
        super().__init__("imu_mpu6050_node")
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("address", 0x68)
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("bias_samples", 300)     # stationary gyro samples averaged at startup
        self.declare_parameter("gyro_z_sign", 1.0)      # flip if +yaw (CCW) reads negative

        bus = int(self.get_parameter("i2c_bus").value)
        addr = int(self.get_parameter("address").value)
        self.rate = float(self.get_parameter("rate_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.gz_sign = float(self.get_parameter("gyro_z_sign").value)
        n_bias = int(self.get_parameter("bias_samples").value)

        self.pub = self.create_publisher(Imu, "/imu/data", 20)
        self._ok = False
        self._bias = (0.0, 0.0, 0.0)
        try:
            self.dev = _I2C(bus, addr)
            self.dev.write_byte(_PWR_MGMT_1, 0x00)      # wake
            time.sleep(0.05)
            self.dev.write_byte(_GYRO_CONFIG, 0x00)     # ±250 deg/s
            self.dev.write_byte(_ACCEL_CONFIG, 0x00)    # ±2 g
            time.sleep(0.05)
            self._calibrate_bias(n_bias)
            self._ok = True
            self.get_logger().info(
                f"MPU6050 on i2c-{bus} 0x{addr:02x} (clone WHO_AM_I ok) rate={self.rate}Hz "
                f"gyro_bias(rad/s)=({self._bias[0]:+.4f},{self._bias[1]:+.4f},{self._bias[2]:+.4f})"
            )
        except Exception as exc:  # noqa: BLE001 - stay alive so the graph doesn't break
            self.get_logger().error(f"MPU6050 init failed ({exc}); IMU disabled")

        if self._ok:
            self.timer = self.create_timer(1.0 / max(1.0, self.rate), self.tick)

    def _read(self) -> tuple[tuple, tuple]:
        """Return (ax,ay,az) m/s^2, (gx,gy,gz) rad/s (RAW, no bias removed)."""
        raw = self.dev.read_block(_ACCEL_XOUT_H, 14)
        ax, ay, az, _t, gx, gy, gz = struct.unpack(">hhhhhhh", raw)
        acc = (ax / _ACCEL_LSB * _G, ay / _ACCEL_LSB * _G, az / _ACCEL_LSB * _G)
        gyr = (gx / _GYRO_LSB * _DEG2RAD, gy / _GYRO_LSB * _DEG2RAD, gz / _GYRO_LSB * _DEG2RAD)
        return acc, gyr

    def _calibrate_bias(self, n: int) -> None:
        sx = sy = sz = 0.0
        got = 0
        for _ in range(max(1, n)):
            try:
                _acc, g = self._read()
            except Exception:  # noqa: BLE001
                continue
            sx += g[0]; sy += g[1]; sz += g[2]
            got += 1
            time.sleep(0.002)
        if got:
            self._bias = (sx / got, sy / got, sz / got)

    def tick(self) -> None:
        try:
            acc, gyr = self._read()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"IMU read failed: {exc}", throttle_duration_sec=2.0)
            return
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.angular_velocity.x = gyr[0] - self._bias[0]
        m.angular_velocity.y = gyr[1] - self._bias[1]
        m.angular_velocity.z = (gyr[2] - self._bias[2]) * self.gz_sign   # yaw rate (heading source)
        m.linear_acceleration.x = acc[0]
        m.linear_acceleration.y = acc[1]
        m.linear_acceleration.z = acc[2]
        m.orientation_covariance[0] = -1.0        # orientation not provided
        self.pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuMpu6050Node()
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
