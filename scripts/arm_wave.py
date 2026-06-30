#!/usr/bin/env python3
"""로봇팔 인사(웨이브) — 베이스 옆으로 + 어깨·팔꿈치·손목 함께 앞뒤로. 느리고 부드럽게.

스크립트가 자세 사이를 잘게 보간(기본 20°/s)해서 펌웨어 슬루보다 더 느리게/매끄럽게.
arm_servo v2 권장. ⚠️ 서보 전원 ON + 팔 주변 공간 + 받칠 준비.

  python3 arm_wave.py                       # 왼쪽 4번, 20°/s
  python3 arm_wave.py --speed 12            # 더 느리게
  python3 arm_wave.py --side right --waves 5
  python3 arm_wave.py --dry
"""
import argparse
import sys
import time

import serial

# 서보각 [base, shoulder, elbow, wristPitch, wristRoll, gripper]
HOME = [70, 96, 55, 55, 90, 90]


def lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(6)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--waves", type=int, default=4, help="흔드는 횟수")
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--speed", type=float, default=20.0, help="관절 속도 deg/s (작을수록 느림)")
    ap.add_argument("--pause", type=float, default=0.3, help="앞/뒤 끝에서 잠깐 멈춤 s")
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args()

    base_side = 120 if args.side == "left" else 30   # ch0: +서보 = 왼쪽

    RAISE = [HOME[0], 120, 95, 60, 90, 90]
    TURN = [base_side, 120, 95, 60, 90, 90]
    # 웨이브: 어깨·팔꿈치·손목 함께 앞뒤로
    WAVE_FWD = [base_side, 105, 115, 95, 90, 90]
    WAVE_BACK = [base_side, 135, 75, 30, 90, 90]
    CENTER = [base_side, 120, 95, 60, 90, 90]

    wps = [("들어올리기", RAISE, 0.4), ("옆으로 돌리기", TURN, 0.4)]
    for i in range(args.waves):
        wps.append((f"흔들기{i+1} 앞", WAVE_FWD, args.pause))
        wps.append((f"흔들기{i+1} 뒤", WAVE_BACK, args.pause))
    wps += [("가운데", CENTER, 0.3), ("정면 복귀", RAISE, 0.3), ("HOME", HOME, 0.4)]

    if args.dry:
        print(f"속도 {args.speed}°/s, 흔들기 {args.waves}회 (앞↔뒤 1회 ≈ "
              f"{max(abs(WAVE_FWD[i]-WAVE_BACK[i]) for i in range(6))/args.speed:.1f}s)")
        for n, p, _ in wps:
            print(f"  {n:12s} {p}")
        return

    ser = serial.Serial(args.port, 115200, timeout=0.2, write_timeout=2.0)
    time.sleep(2.3)  # 최초 연결 1회 리셋→home
    cur = list(HOME)
    dt = 0.04

    def move(target):
        nonlocal cur
        dmax = max(abs(target[i] - cur[i]) for i in range(6))
        steps = max(int((dmax / args.speed) / dt), 1)
        for k in range(1, steps + 1):
            p = lerp(cur, target, k / steps)
            ser.write(("<ARM," + ",".join(str(int(round(v))) for v in p) + ">\n").encode("ascii"))
            if ser.in_waiting:
                ser.read(ser.in_waiting)   # 펌웨어 ACK 비우기 (안 비우면 버퍼 차서 멈춤)
            time.sleep(dt)
        cur = list(target)

    ok = True
    try:
        for name, pose, pause in wps:
            print(" ", name)
            sys.stdout.flush()
            move(pose)
            if pause:
                time.sleep(pause)
    except serial.SerialTimeoutException:
        ok = False
        print("\n⚠️ 시리얼 쓰기 멈춤(write timeout) — 팔 보드가 응답을 안 함.")
        print("   원인 후보: 서보 전원 브라운아웃으로 보드 리셋 / PCA9685 I2C 멈춤.")
    except (serial.SerialException, OSError) as e:
        ok = False
        print(f"\n⚠️ 시리얼 끊김 ({e}) — 보드 리셋/USB 분리 가능성 (서보 전원 부족?).")
    except KeyboardInterrupt:
        ok = False
        print("\n중단됨")
    finally:
        try:
            ser.close()
        except Exception:
            pass
    if ok:
        print("인사 완료 👋")


if __name__ == "__main__":
    main()
