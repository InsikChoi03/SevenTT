#!/usr/bin/env python3
"""Prompt for five 2R arm poses without printing the MCU telemetry stream."""
from __future__ import annotations

import argparse
import time


class StopTest(Exception):
    """Raised when the operator requests an early, safe exit."""


def read_for(ser, seconds: float) -> list[str]:
    deadline = time.monotonic() + seconds
    chunks: list[bytes] = []
    while time.monotonic() < deadline:
        data = ser.read(512)
        if data:
            chunks.append(data)
    return b"".join(chunks).decode("ascii", "ignore").splitlines()


def send_line(ser, line: str) -> None:
    ser.write((line.rstrip() + "\n").encode("ascii"))
    ser.flush()


def ask_angle(label: str, minimum: float, maximum: float) -> int:
    while True:
        raw = input(f"{label} 각도 입력 ({minimum:.0f}~{maximum:.0f}, q=종료): ").strip()
        if raw.lower() == "q":
            raise StopTest
        try:
            value = float(raw)
        except ValueError:
            print("  숫자로 입력하세요.")
            continue
        if not minimum <= value <= maximum:
            print(f"  안전 범위 {minimum:.0f}~{maximum:.0f}도 안에서 입력하세요.")
            continue
        return int(round(value))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--tests", type=int, default=5)
    parser.add_argument("--min-angle", type=float, default=0.0)
    parser.add_argument("--max-angle", type=float, default=170.0)
    args = parser.parse_args()

    if args.tests < 1:
        parser.error("--tests must be at least 1")
    if args.min_angle >= args.max_angle:
        parser.error("--min-angle must be lower than --max-angle")

    try:
        import serial
    except ImportError:
        print("pyserial이 필요합니다: pip install pyserial")
        return 1

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=1.0)
    except Exception as exc:  # noqa: BLE001 - print the serial driver error for diagnosis
        print(f"포트 열기 실패: {args.port}: {exc}")
        return 1

    try:
        print("Arduino 재시작 대기 중...")
        boot_lines = read_for(ser, 2.5)
        banner = next((line for line in boot_lines if line.startswith("<BANNER")), None)
        if banner:
            print(banner)
        ser.reset_input_buffer()
        send_line(ser, "<STATUS,RUNNING>")
        read_for(ser, 0.25)

        print("\n각도 순서: 어깨 / 손목 / 그리퍼")
        print("MCU의 HB·ODOM 로그는 화면에 표시하지 않습니다.")
        for index in range(1, args.tests + 1):
            print(f"\n========== TEST {index}/{args.tests} ==========")
            shoulder = ask_angle("어깨", args.min_angle, args.max_angle)
            wrist = ask_angle("손목", args.min_angle, args.max_angle)
            gripper = ask_angle("그리퍼", args.min_angle, args.max_angle)

            ser.reset_input_buffer()
            command = f"<ARM,{shoulder},{wrist},{gripper}>"
            send_line(ser, command)
            feedback = read_for(ser, 0.6)
            ack = next((line for line in feedback if line.startswith("<ARMACK")), None)
            print(f"전송 완료: 어깨={shoulder}, 손목={wrist}, 그리퍼={gripper}")
            print(ack if ack else "ARMACK 없음 - 실제 움직임과 전원 상태를 확인하세요.")

            if index < args.tests:
                answer = input("자세 확인 후 Enter (q=종료): ").strip().lower()
                if answer == "q":
                    raise StopTest

        print(f"\n{args.tests}회 테스트 완료.")
    except (KeyboardInterrupt, EOFError, StopTest):
        print("\n테스트를 중단합니다.")
    finally:
        try:
            send_line(ser, "<STATUS,STANDBY>")
            read_for(ser, 0.2)
            print("STANDBY 전송 완료.")
        finally:
            ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
