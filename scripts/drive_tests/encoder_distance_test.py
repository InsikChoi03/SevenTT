#!/usr/bin/env python3
"""Serial-only encoder distance / rotation test scaffold for the mecanum base.

This script is intentionally low-level and ROS-free. It is the right place to validate:

- whether the MCU emits real <ODOM,...> packets
- whether commanded forward motion roughly matches measured distance
- whether the robot stops near a requested distance
- whether the robot rotates near a requested angle
- whether a short diagonal rolling start helps lateral motion break static friction
- whether a forward distance segment helps lateral motion start more reliably
- whether the base can drive a standalone 45-degree forward diagonal path
- whether a short high-power boost can break static friction before steady lateral motion
- whether repeated high-power pulses can ratchet the base sideways

Current firmware note:
- `base_arm_combined.ino` emits `<ODOM,...>` after the encoder-enabled sketch is flashed
- `<ENC,...>` raw tick lines are printed so wheel/encoder mapping can be checked
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import statistics
import sys
import time

import serial


def send_base(ser: serial.Serial, fl: float, fr: float, rl: float, rr: float) -> None:
    ser.write(f"<BASE,{fl:.3f},{fr:.3f},{rl:.3f},{rr:.3f}>\n".encode("ascii"))


def stop_base(ser: serial.Serial) -> None:
    send_base(ser, 0.0, 0.0, 0.0, 0.0)


def reset_encoders(ser: serial.Serial) -> None:
    ser.write(b"<ENCZERO>\n")


def brake_base(ser: serial.Serial, wheel_speeds: list[float], ms: int, hz: float) -> None:
    if ms <= 0:
        return
    period = 1.0 / max(hz, 1e-6)
    end = time.monotonic() + (ms / 1000.0)
    while time.monotonic() < end:
        send_base(ser, *wheel_speeds)
        time.sleep(period)
    stop_base(ser)


def parse_packet(line: str) -> tuple[str, list[float]] | None:
    if not (line.startswith("<") and line.endswith(">")):
        return None
    body = line[1:-1]
    parts = body.split(",")
    if not parts:
        return None
    tag = parts[0]
    vals: list[float] = []
    for p in parts[1:]:
        try:
            vals.append(float(p))
        except ValueError:
            return None
    return tag, vals


def parse_index_list(text: str) -> list[int]:
    out: list[int] = []
    for part in text.split(","):
        p = part.strip()
        if not p:
            continue
        idx = int(p)
        if idx < 0 or idx > 3:
            raise ValueError(f"wheel index out of range: {idx}")
        out.append(idx)
    if not out:
        raise ValueError("need at least one wheel index")
    return out


def selected_motion_speed(vals: list[float], wheels: list[int], signs: list[float]) -> float:
    if len(vals) < 4:
        return 0.0
    samples = [float(signs[i]) * float(vals[i]) for i in wheels if abs(float(signs[i])) > 1e-9]
    if not samples:
        return 0.0
    if len(samples) == 1:
        return samples[0]
    if len(samples) == 2:
        return 0.5 * (samples[0] + samples[1])
    return float(statistics.median(samples))


def motion_profile(direction: str, speed: float) -> tuple[list[float], list[float], list[float]]:
    mag = abs(float(speed))
    if direction == "forward":
        cmd = [mag, mag, mag, mag]
        odom_signs = [1.0, 1.0, 1.0, 1.0]
    elif direction == "backward":
        cmd = [-mag, -mag, -mag, -mag]
        odom_signs = [-1.0, -1.0, -1.0, -1.0]
    elif direction == "right":
        cmd = [mag, -mag, -mag, mag]
        odom_signs = [1.0, -1.0, -1.0, 1.0]
    elif direction == "left":
        cmd = [-mag, mag, mag, -mag]
        odom_signs = [-1.0, 1.0, 1.0, -1.0]
    elif direction == "forward_right":
        cmd = [mag, 0.0, 0.0, mag]
        odom_signs = [1.0, 0.0, 0.0, 1.0]
    elif direction == "forward_left":
        cmd = [0.0, mag, mag, 0.0]
        odom_signs = [0.0, 1.0, 1.0, 0.0]
    elif direction == "ccw":
        cmd = [-mag, mag, -mag, mag]
        odom_signs = [-1.0, 1.0, -1.0, 1.0]
    elif direction == "cw":
        cmd = [mag, -mag, mag, -mag]
        odom_signs = [1.0, -1.0, 1.0, -1.0]
    else:
        raise ValueError(f"unsupported direction: {direction}")
    brake = [-0.5 * v for v in cmd]
    return cmd, brake, odom_signs


def diagonal_kick_profile(direction: str, speed: float) -> list[float]:
    mag = abs(float(speed))
    if direction == "right":
        return [mag, 0.0, 0.0, mag]
    if direction == "left":
        return [0.0, mag, mag, 0.0]
    raise ValueError(f"lateral kick only supports left/right, got: {direction}")


def run_open_loop(ser: serial.Serial, wheel_speeds: list[float], ms: int, hz: float) -> None:
    if ms <= 0:
        return
    period = 1.0 / max(hz, 1e-6)
    end = time.monotonic() + (ms / 1000.0)
    while time.monotonic() < end:
        send_base(ser, *wheel_speeds)
        time.sleep(period)


def append_csv(path: Path, row: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def run_open_loop_calibration(
    ser: serial.Serial,
    *,
    direction: str,
    speed: float,
    duration_ms: int,
    hz: float,
    k: float,
    trusted_wheels: list[int],
    brake_ms: int,
    brake_wheels: list[float],
) -> tuple[float, float, bool]:
    """Run a fixed-duration move and integrate encoder odom without stopping from odom."""
    cmd_wheels, _, odom_signs = motion_profile(direction, speed)
    rotational = direction in {"cw", "ccw"}
    cmd_period = 1.0 / max(hz, 1e-6)
    duration_s = max(0.001, duration_ms / 1000.0)
    start = time.monotonic()
    end_at = start + duration_s
    last_cmd = 0.0
    last_odom_t = None
    progress = 0.0
    saw_hb = False
    saw_odom = False
    rx_buf = b""

    print(
        f"open-loop-calib: direction={direction} wheel_cmd={cmd_wheels} "
        f"duration={duration_ms} ms odom_wheels={trusted_wheels}"
    )
    while time.monotonic() < end_at:
        now = time.monotonic()
        if now - last_cmd >= cmd_period:
            send_base(ser, *cmd_wheels)
            last_cmd = now

        data = ser.read(128)
        if data:
            rx_buf += data
        while b"\n" in rx_buf:
            raw, rx_buf = rx_buf.split(b"\n", 1)
            line = raw.strip().decode("ascii", errors="ignore")
            pkt = parse_packet(line)
            if pkt is None:
                continue
            tag, vals = pkt
            if tag == "HB":
                saw_hb = True
                continue
            if tag == "ENC" and len(vals) >= 4:
                print(
                    f"raw enc e1={int(vals[0])} e2={int(vals[1])} "
                    f"e3={int(vals[2])} e4={int(vals[3])}"
                )
                continue
            if tag != "ODOM":
                continue
            saw_odom = True
            t_now = time.monotonic()
            signed_speed = selected_motion_speed(vals, trusted_wheels, odom_signs)
            if last_odom_t is not None:
                dt = max(0.0, min(0.5, t_now - last_odom_t))
                if rotational:
                    progress += (signed_speed / max(float(k), 1e-6)) * dt
                else:
                    progress += signed_speed * dt
            last_odom_t = t_now
            if rotational:
                print(f"odom spin={signed_speed:+.3f} angle={math.degrees(progress):+.1f} deg")
            else:
                print(f"odom speed={signed_speed:+.3f} distance={progress:+.3f} m")

    stop_base(ser)
    if brake_ms > 0:
        print(f"applying reverse brake pulse: {brake_wheels} for {brake_ms} ms")
        brake_base(ser, brake_wheels, brake_ms, hz)
    stop_base(ser)

    if saw_hb and not saw_odom:
        print("MCU is only sending <HB,...> command echo.")
        print("Add real encoder output <ODOM,fl,fr,rl,rr,t_ms> in firmware first.")
    return progress, duration_s, saw_odom


def run_closed_loop_segment(
    ser: serial.Serial,
    *,
    direction: str,
    speed: float,
    target_m: float,
    target_deg: float,
    timeout: float,
    hz: float,
    k: float,
    trusted_wheels: list[int],
    label: str,
) -> bool:
    cmd_wheels, _, odom_signs = motion_profile(direction, speed)
    rotational = direction in {"cw", "ccw"}
    target = math.radians(abs(float(target_deg))) if rotational else abs(float(target_m))
    target_label = f"{math.degrees(target):.1f}deg" if rotational else f"{target:.3f}m"
    cmd_period = 1.0 / max(hz, 1e-6)
    start = time.monotonic()
    last_cmd = 0.0
    last_odom_t = None
    progress = 0.0
    saw_hb = False
    saw_odom = False
    rx_buf = b""

    print(f"{label}: direction={direction} wheel_cmd={cmd_wheels} target={target_label}")

    while True:
        now = time.monotonic()
        if now - start > timeout:
            if rotational:
                print(f"{label} timeout: angle={math.degrees(progress):.1f} deg")
            else:
                print(f"{label} timeout: distance={progress:.3f} m")
            return False

        if now - last_cmd >= cmd_period:
            send_base(ser, *cmd_wheels)
            last_cmd = now

        data = ser.read(128)
        if data:
            rx_buf += data
        while b"\n" in rx_buf:
            raw, rx_buf = rx_buf.split(b"\n", 1)
            line = raw.strip().decode("ascii", errors="ignore")
            pkt = parse_packet(line)
            if pkt is None:
                continue
            tag, vals = pkt

            if tag == "HB":
                saw_hb = True
            elif tag == "ENC":
                if len(vals) >= 4:
                    print(
                        f"raw enc e1={int(vals[0])} e2={int(vals[1])} "
                        f"e3={int(vals[2])} e4={int(vals[3])}"
                    )
            elif tag == "ODOM":
                saw_odom = True
                t_now = time.monotonic()
                signed_speed = selected_motion_speed(vals, trusted_wheels, odom_signs)
                if last_odom_t is not None:
                    dt = max(0.0, min(0.5, t_now - last_odom_t))
                    if rotational:
                        progress += (signed_speed / max(float(k), 1e-6)) * dt
                    else:
                        progress += signed_speed * dt
                last_odom_t = t_now
                if rotational:
                    print(
                        f"{label} odom spin={signed_speed:+.3f} "
                        f"angle={math.degrees(progress):+.1f} / {math.degrees(target):.1f} deg"
                    )
                else:
                    print(f"{label} odom speed={signed_speed:+.3f} distance={progress:+.3f} / {target:.3f} m")
                if abs(progress) >= target:
                    print(f"{label} target reached")
                    return True

        if saw_hb and not saw_odom and (time.monotonic() - start) > 1.5:
            print("MCU is only sending <HB,...> command echo.")
            print("Add real encoder output <ODOM,fl,fr,rl,rr,t_ms> in firmware first.")
            return False


def main() -> None:
    ap = argparse.ArgumentParser(description="Encoder distance / open-loop calibration test using MCU packets")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument(
        "--direction",
        choices=["forward", "backward", "left", "right", "forward_right", "forward_left", "cw", "ccw"],
        default="forward",
    )
    ap.add_argument(
        "--mode",
        choices=[
            "single",
            "lateral-kick",
            "forward-then-lateral",
            "start-boost",
            "pulse",
            "open-loop-calib",
        ],
        default="single",
        help=(
            "single keeps the original behavior; lateral-kick adds a timed diagonal start; "
            "forward-then-lateral drives forward by --pre-m before left/right; "
            "start-boost briefly uses --boost-speed before --speed; pulse alternates strong lateral bursts and stops; "
            "open-loop-calib sends the normal wheel command for --duration-ms and compares encoder vs measured travel"
        ),
    )
    ap.add_argument("--speed", type=float, default=0.10, help="wheel speed command magnitude in m/s")
    ap.add_argument("--target-m", type=float, default=0.50, help="requested travel distance in meters")
    ap.add_argument("--target-deg", type=float, default=90.0, help="requested rotation in degrees for cw/ccw")
    ap.add_argument("--timeout", type=float, default=10.0, help="max test duration in seconds")
    ap.add_argument("--hz", type=float, default=20.0, help="command resend rate")
    ap.add_argument("--k", type=float, default=0.20, help="mecanum k = lx + ly in meters for rotation integration")
    ap.add_argument(
        "--brake-ms",
        type=int,
        default=0,
        help="reverse brake pulse duration in ms after target reached (0 disables)",
    )
    ap.add_argument(
        "--brake-speed-scale",
        type=float,
        default=0.5,
        help="reverse brake pulse speed as a fraction of commanded speed magnitude",
    )
    ap.add_argument(
        "--odom-wheels",
        default="1,2,3",
        help="comma-separated ODOM wheel indexes to trust; default ignores weak FL encoder",
    )
    ap.add_argument(
        "--kick-ms",
        type=int,
        default=250,
        help="lateral-kick diagonal rolling-start duration in ms",
    )
    ap.add_argument(
        "--kick-speed-scale",
        type=float,
        default=1.0,
        help="lateral-kick speed as a fraction of --speed",
    )
    ap.add_argument(
        "--pre-m",
        type=float,
        default=0.10,
        help="forward-then-lateral forward segment target in meters",
    )
    ap.add_argument(
        "--pre-timeout",
        type=float,
        default=5.0,
        help="forward-then-lateral forward segment timeout in seconds",
    )
    ap.add_argument(
        "--boost-speed",
        type=float,
        default=0.50,
        help="start-boost initial wheel speed magnitude in m/s",
    )
    ap.add_argument(
        "--boost-ms",
        type=int,
        default=200,
        help="start-boost duration in ms",
    )
    ap.add_argument(
        "--pulse-speed",
        type=float,
        default=0.50,
        help="pulse mode wheel speed magnitude during each burst in m/s",
    )
    ap.add_argument(
        "--pulse-on-ms",
        type=int,
        default=150,
        help="pulse mode burst duration in ms",
    )
    ap.add_argument(
        "--pulse-off-ms",
        type=int,
        default=50,
        help="pulse mode stop duration between bursts in ms",
    )
    ap.add_argument(
        "--duration-ms",
        type=int,
        default=1000,
        help="open-loop-calib fixed drive duration in ms",
    )
    ap.add_argument(
        "--actual-m",
        type=float,
        default=None,
        help="measured linear travel in meters for open-loop-calib; prints encoder scale actual/odom",
    )
    ap.add_argument(
        "--actual-deg",
        type=float,
        default=None,
        help="measured rotation in degrees for open-loop-calib; prints encoder/yaw scale actual/odom",
    )
    ap.add_argument(
        "--log-csv",
        default="",
        help="optional CSV path for open-loop-calib results",
    )
    args = ap.parse_args()
    trusted_wheels = parse_index_list(args.odom_wheels)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except (serial.SerialException, OSError) as e:
        print(f"port open failed: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"opened {args.port} @ {args.baud}")
    print("waiting for MCU reboot...")
    time.sleep(2.0)
    ser.reset_input_buffer()
    stop_base(ser)
    reset_encoders(ser)

    lateral_modes = {"lateral-kick", "forward-then-lateral", "start-boost", "pulse"}
    if args.mode in lateral_modes and args.direction not in {"left", "right"}:
        print(f"--mode {args.mode} only supports --direction left/right", file=sys.stderr)
        sys.exit(2)

    cmd_wheels, brake_wheels, odom_signs = motion_profile(args.direction, args.speed)
    boost_wheels = motion_profile(args.direction, args.boost_speed)[0]
    pulse_wheels = motion_profile(args.direction, args.pulse_speed)[0]
    brake_scale = max(0.0, float(args.brake_speed_scale))
    brake_wheels = [v * brake_scale for v in brake_wheels]
    rotational = args.direction in {"cw", "ccw"}
    target = math.radians(abs(float(args.target_deg))) if rotational else abs(float(args.target_m))
    cmd_period = 1.0 / max(args.hz, 1e-6)
    start = time.monotonic()
    last_cmd = 0.0
    last_odom_t = None
    progress = 0.0
    saw_hb = False
    saw_odom = False
    rx_buf = b""
    target_label = f"{math.degrees(target):.1f}deg" if rotational else f"{target:.3f}m"
    print(
        f"test: mode={args.mode} direction={args.direction} wheel_cmd={cmd_wheels} "
        f"target={target_label} odom_wheels={trusted_wheels} "
        f"brake_ms={args.brake_ms} brake_wheels={brake_wheels}"
    )
    if args.mode == "start-boost":
        print(f"start boost: wheel_cmd={boost_wheels} for {args.boost_ms} ms, then {cmd_wheels}")
    elif args.mode == "pulse":
        print(
            f"pulse: wheel_cmd={pulse_wheels} on={args.pulse_on_ms} ms "
            f"off={args.pulse_off_ms} ms"
        )

    try:
        if args.mode == "open-loop-calib":
            progress, ran_s, saw_odom = run_open_loop_calibration(
                ser,
                direction=args.direction,
                speed=args.speed,
                duration_ms=args.duration_ms,
                hz=args.hz,
                k=args.k,
                trusted_wheels=trusted_wheels,
                brake_ms=args.brake_ms,
                brake_wheels=brake_wheels,
            )
            if not saw_odom:
                return
            if rotational:
                odom_value = math.degrees(progress)
                actual_value = args.actual_deg
                unit = "deg"
            else:
                odom_value = progress
                actual_value = args.actual_m
                unit = "m"
            print(f"RESULT odom_{unit}={odom_value:+.4f} duration_s={ran_s:.3f}")
            scale = None
            if actual_value is None:
                prompt = f"Measured actual travel ({unit}); blank to skip"
                text = input(f"{prompt}: ").strip()
                if text:
                    actual_value = float(text)
            if actual_value is not None:
                if abs(odom_value) <= 1e-9:
                    print("Cannot compute scale: odom value is ~0")
                else:
                    scale = float(actual_value) / float(odom_value)
                    print(f"RESULT actual_{unit}={actual_value:+.4f}")
                    print(f"RESULT encoder_scale_actual_over_odom={scale:.6f}")
                    print(
                        "Apply concept: calibrated_distance = raw_encoder_distance "
                        f"* {scale:.6f}"
                    )
            if args.log_csv:
                row = {
                    "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "mode": args.mode,
                    "direction": args.direction,
                    "speed": args.speed,
                    "duration_ms": args.duration_ms,
                    "odom_wheels": args.odom_wheels,
                    "odom_value": odom_value,
                    "actual_value": "" if actual_value is None else actual_value,
                    "scale_actual_over_odom": "" if scale is None else scale,
                    "unit": unit,
                }
                append_csv(Path(args.log_csv), row)
                print(f"logged: {args.log_csv}")
            return

        if args.mode == "lateral-kick":
            kick_wheels = diagonal_kick_profile(
                args.direction,
                args.speed * max(0.0, float(args.kick_speed_scale)),
            )
            print(f"diagonal kick: wheel_cmd={kick_wheels} for {args.kick_ms} ms")
            run_open_loop(ser, kick_wheels, args.kick_ms, args.hz)
        elif args.mode == "forward-then-lateral":
            ok = run_closed_loop_segment(
                ser,
                direction="forward",
                speed=args.speed,
                target_m=args.pre_m,
                target_deg=args.target_deg,
                timeout=args.pre_timeout,
                hz=args.hz,
                k=args.k,
                trusted_wheels=trusted_wheels,
                label="pre-forward",
            )
            if not ok:
                stop_base(ser)
                return
            start = time.monotonic()
            last_cmd = 0.0
            last_odom_t = None
            progress = 0.0
            saw_hb = False
            saw_odom = False
            rx_buf = b""

        while True:
            now = time.monotonic()
            if now - start > args.timeout:
                if rotational:
                    print(f"timeout: angle={math.degrees(progress):.1f} deg")
                else:
                    print(f"timeout: distance={progress:.3f} m")
                break

            if now - last_cmd >= cmd_period:
                elapsed_ms = (now - start) * 1000.0
                if args.mode == "start-boost" and elapsed_ms < max(0, args.boost_ms):
                    send_base(ser, *boost_wheels)
                elif args.mode == "pulse":
                    on_ms = max(1, args.pulse_on_ms)
                    off_ms = max(0, args.pulse_off_ms)
                    cycle_ms = on_ms + off_ms
                    if cycle_ms <= 0 or (elapsed_ms % cycle_ms) < on_ms:
                        send_base(ser, *pulse_wheels)
                    else:
                        stop_base(ser)
                else:
                    send_base(ser, *cmd_wheels)
                last_cmd = now

            data = ser.read(128)
            if data:
                rx_buf += data
            while b"\n" in rx_buf:
                raw, rx_buf = rx_buf.split(b"\n", 1)
                line = raw.strip().decode("ascii", errors="ignore")
                pkt = parse_packet(line)
                if pkt is None:
                    continue
                tag, vals = pkt

                if tag == "HB":
                    saw_hb = True
                elif tag == "ENC":
                    if len(vals) >= 4:
                        print(
                            f"raw enc e1={int(vals[0])} e2={int(vals[1])} "
                            f"e3={int(vals[2])} e4={int(vals[3])}"
                        )
                elif tag == "ODOM":
                    saw_odom = True
                    t_now = time.monotonic()
                    signed_speed = selected_motion_speed(vals, trusted_wheels, odom_signs)
                    if last_odom_t is not None:
                        dt = max(0.0, min(0.5, t_now - last_odom_t))
                        if rotational:
                            progress += (signed_speed / max(float(args.k), 1e-6)) * dt
                        else:
                            progress += signed_speed * dt
                    last_odom_t = t_now
                    if rotational:
                        print(
                            f"odom spin={signed_speed:+.3f} angle={math.degrees(progress):+.1f} / "
                            f"{math.degrees(target):.1f} deg"
                        )
                    else:
                        print(f"odom speed={signed_speed:+.3f} distance={progress:+.3f} / {target:.3f} m")
                    if abs(progress) >= target:
                        print("target reached")
                        if args.brake_ms > 0:
                            print(f"applying reverse brake pulse: {brake_wheels} for {args.brake_ms} ms")
                            brake_base(ser, brake_wheels, args.brake_ms, args.hz)
                        stop_base(ser)
                        return

            if saw_hb and not saw_odom and (time.monotonic() - start) > 1.5:
                print("MCU is only sending <HB,...> command echo.")
                print("Add real encoder output <ODOM,fl,fr,rl,rr,t_ms> in firmware first.")
                break
    finally:
        for _ in range(5):
            stop_base(ser)
            time.sleep(0.02)
        ser.flush()
        ser.close()
        print("stopped and closed")


if __name__ == "__main__":
    main()
