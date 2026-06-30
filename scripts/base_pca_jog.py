#!/usr/bin/env python3
"""PCA9685 채널 단독 구동 — MX1508 채널↔모터 매핑 실측 (대화형).

base_pca_scan 펌웨어와 함께 사용. 채널 하나만 켜서 어느 바퀴가 어느 방향으로
도는지 직접 보면서 매핑을 만든다.

대화형 명령(프롬프트 ch> 에서):
    0~15     그 채널만 펄스 (기본 1.5초) → 어느 바퀴/방향인지 관찰
    d<n>     duty 변경 (예: d3000)  기본 2500
    t<sec>   펄스 길이 변경 (예: t2.5)
    scan     0→15 순차 펄스
    off      전 채널 정지
    q        종료

사용:
    python3 base_pca_jog.py [/dev/ttyUSB0]
    python3 base_pca_jog.py /dev/ttyUSB0 --scan      # 비대화 순차 스윕
"""
import argparse
import sys
import time

import serial


def make_pulse(ser):
    def pulse(ch, duty, dur):
        end = time.time() + dur
        # 워치독(2s) 이기게 0.3s마다 재전송하며 dur 동안 유지
        while time.time() < end:
            ser.write(f"<ONLY,{ch},{duty}>\n".encode("ascii"))
            time.sleep(0.3)
        ser.write(b"<OFF>\n")
        ser.flush()
    return pulse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("--scan", action="store_true", help="대화 없이 0~15 순차 스윕")
    ap.add_argument("--duty", type=int, default=2500)
    ap.add_argument("--dur", type=float, default=1.5)
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(2.2)
    ser.reset_input_buffer()
    pulse = make_pulse(ser)

    duty = args.duty
    dur = args.dur

    try:
        if args.scan:
            for ch in range(16):
                print(f"  ch {ch:>2} 펄스 ({dur}s, duty {duty})  ← 지금 도는 바퀴/방향 메모")
                sys.stdout.flush()
                pulse(ch, duty, dur)
                time.sleep(0.7)
            print("스윕 끝")
            return

        print("채널 0-15 입력→펄스 / d<n>=duty / t<sec>=길이 / scan / off / q")
        while True:
            try:
                cmd = input("ch> ").strip()
            except EOFError:
                break
            if cmd == "q":
                break
            elif cmd == "off":
                ser.write(b"<OFF>\n")
            elif cmd == "scan":
                for ch in range(16):
                    print(f"  ch {ch:>2} ..."); sys.stdout.flush()
                    pulse(ch, duty, dur)
                    time.sleep(0.6)
            elif cmd.startswith("d") and cmd[1:].isdigit():
                duty = int(cmd[1:]); print("  duty =", duty)
            elif cmd.startswith("t"):
                try:
                    dur = float(cmd[1:]); print("  dur =", dur)
                except ValueError:
                    print("  형식: t2.5")
            elif cmd.isdigit():
                ch = int(cmd)
                print(f"  ch {ch} 펄스 ({dur}s, duty {duty})")
                pulse(ch, duty, dur)
            elif cmd:
                print("  ? 채널 숫자(0-15), d<n>, t<sec>, scan, off, q")
    finally:
        ser.write(b"<OFF>\n")
        ser.flush()
        ser.close()
        print("OFF, 종료")


if __name__ == "__main__":
    main()
