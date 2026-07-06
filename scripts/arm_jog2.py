#!/usr/bin/env python3
"""새 2R 팔 수동 각도 테스트 조그 — PCA9685 ch0=어깨, ch1=손목, ch2=그리퍼.

통합 아두이노(베이스+팔)가 기존 arm_servo 프로토콜 `<ARM,t1..t6>`(서보각 0~180 정수)을
그대로 받는다고 가정하고, t1/t2/t3 = ch0/1/2 = 어깨/손목/그리퍼로 보낸다(t4~6=90 filler, 미사용).
펌웨어가 smooth 보간이라 급점프 없음.

대화형 명령:
  s 45      어깨(ch0) → 45°       w 90   손목(ch1) → 90°     g 30   그리퍼(ch2) → 30°
  s +5      어깨 +5° 상대조그       w -10  손목 -10°
  0 45      채널 직접 (0/1/2)
  a         현재 각도 보기          sweep s   어깨 천천히 최소↔최대 왕복(테스트, 아무키로 중단)
  q         종료(서보 위치 유지)

  python3 scripts/arm_jog2.py                 # /dev/ttyACM0, 클램프 10~170, 시작 90
  python3 scripts/arm_jog2.py --port /dev/ttyUSB0 --start 90
안전: 각도 [--min,--max]로 클램프(기본 10~170, 하드스톱 회피 [[feedback-avoid-servo-extremes]]).
  ⚠️ 첫 명령/시작 시 ch0/1/2 모두 시작각으로 이동. 그리퍼 안전각 미확인이니 전원 차단 준비.
"""
from __future__ import annotations

import argparse
import glob
import sys
import time

CH = {"s": 0, "w": 1, "g": 2, "0": 0, "1": 1, "2": 2}
NAME = {0: "shoulder(어깨)", 1: "wrist(손목)", 2: "gripper(그리퍼)"}


def send(ser, ang):
    # <ARM,t1..t6> — t1/t2/t3 = ch0/1/2 = 어깨/손목/그리퍼, t4~6=90(미사용 채널 filler)
    t = [int(round(ang[0])), int(round(ang[1])), int(round(ang[2])), 90, 90, 90]
    ser.write(("<ARM," + ",".join(str(x) for x in t) + ">\n").encode())


def drain(ser, s=0.15):
    end = time.time() + s
    while time.time() < end:
        ser.read(256)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--min", type=float, default=10.0)
    ap.add_argument("--max", type=float, default=170.0)
    ap.add_argument("--start", type=float, default=90.0, help="시작 각도(3채널 공통)")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial 필요: pip install pyserial"); return 1
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"[err] 포트 {args.port} 열기 실패: {e}")
        print("가능 포트:", glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        return 1
    time.sleep(2.2)                  # MCU 리셋 대기
    drain(ser, 0.5)

    def clamp(v):
        return max(args.min, min(args.max, v))

    ang = [clamp(args.start)] * 3    # ch0,1,2 현재 목표(스크립트 추적값)
    print(f"[jog] {args.port} 연결. ch0=어깨 ch1=손목 ch2=그리퍼. 클램프 [{args.min:.0f},{args.max:.0f}]")
    print("⚠️ 지금 3채널 모두 시작각으로 이동합니다. 그리퍼 안전각 미확인 — 이상하면 Ctrl-C/전원차단.")
    print("명령: 's 45' | 's +5' | '0 90' | 'a' | 'sweep s' | 'q'")
    send(ser, ang); drain(ser)      # 시작각 적용

    while True:
        try:
            line = input("jog> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        if line == "q":
            break
        if line == "a":
            print("  현재:", {NAME[i]: round(ang[i], 1) for i in range(3)})
            continue
        parts = line.split()
        # sweep <ch>: 최소↔최대 천천히 왕복 (각도제어 눈으로 확인)
        if len(parts) == 2 and parts[0] == "sweep":
            ch = CH.get(parts[1])
            if ch is None:
                print("  sweep s/w/g"); continue
            print(f"  {NAME[ch]} sweep {args.min:.0f}↔{args.max:.0f} (Ctrl-C로 중단)")
            try:
                for target in (args.max, args.min, args.start):
                    step = 2 if target >= ang[ch] else -2
                    for a in range(int(ang[ch]), int(target), step):
                        ang[ch] = a; send(ser, ang); drain(ser, 0.05)
                    ang[ch] = target; send(ser, ang); drain(ser)
                    time.sleep(0.3)
            except KeyboardInterrupt:
                print("\n  sweep 중단")
            continue
        if len(parts) != 2:
            print("  형식: '<s|w|g|0|1|2> <각도 또는 +/-델타>'"); continue
        ch = CH.get(parts[0])
        if ch is None:
            print("  채널: s/w/g 또는 0/1/2"); continue
        val = parts[1]
        try:
            new = ang[ch] + float(val) if val[0] in "+-" else float(val)
        except (ValueError, IndexError):
            print("  각도는 숫자 (예: 45, +5, -10)"); continue
        c = clamp(new)
        if abs(c - new) > 1e-6:
            print(f"  ⚠️ 클램프 {new:.0f}→{c:.0f} (하드스톱 회피)")
        ang[ch] = c
        send(ser, ang); drain(ser)
        print(f"  {NAME[ch]} → {ang[ch]:.0f}°  (전송 ch0/1/2 = {ang[0]:.0f},{ang[1]:.0f},{ang[2]:.0f})")

    ser.close()
    print("[jog] 종료 — 서보 현재 위치 유지")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
