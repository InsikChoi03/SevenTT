#!/usr/bin/env python3
"""집게(채널6/gripper) 개폐 서보값 튜닝 — 각도 하나씩 입력해 보내고 눈으로 확인.

팔은 HOME 자세 유지, **그리퍼만** 움직임. 열림/닫힘 값 찾으면 arm_ik.py 의
GRIPPER_OPEN / GRIPPER_CLOSED 에 반영하면 됨.

⚠️ 서보 전원 ON. 실행하면 팔이 먼저 HOME(직상)으로 감 — 주변 공간 확보.

  python3 gripper_jog.py                 # /dev/ttyACM0
  python3 gripper_jog.py /dev/ttyACM1    # 포트 지정

프롬프트에 0~180 입력(Enter) → 그 각도로 집게 이동 (작을수록 열림).
  o = 현재 OPEN값 / c = 현재 CLOSED값 / +N, -N = 상대조정 / q = 종료
"""
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "ros2_ws", "src", "robot_control", "robot_control"))
import arm_ik  # noqa: E402


def send(ser, six):
    ser.write(("<ARM," + ",".join(str(int(round(v))) for v in six) + ">\n").encode("ascii"))


def drain(ser):
    """펌웨어 ACK 비우기 (안 읽으면 버퍼 차서 교착 — reference-mcu-ack-deadlock)."""
    try:
        n = ser.in_waiting
        if n:
            ser.read(n)
    except Exception:
        pass


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    ser = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(2.3)            # 아두이노 리셋 대기
    drain(ser)

    base = list(arm_ik.HOME_CMD)          # [70,96,55,55,90,90]
    grip = int(base[5])
    OPEN, CLOSED = int(arm_ik.GRIPPER_OPEN), int(arm_ik.GRIPPER_CLOSED)

    send(ser, base); time.sleep(1.5); drain(ser)   # 안전 HOME
    print(f"HOME 전송 완료. 집게 채널6 현재={grip}.  (작을수록 열림)")
    print(f"참고 현재 arm_ik 값: OPEN={OPEN} / CLOSED={CLOSED}")
    print("입력: 0~180 절대각 / +N -N 상대 / o=OPEN값 / c=CLOSED값 / q=종료\n")

    try:
        while True:
            s = input(f"집게각 [{grip}] > ").strip().lower()
            if s == "q":
                break
            if not s:
                continue
            if s == "o":
                grip = OPEN
            elif s == "c":
                grip = CLOSED
            elif s and s[0] in "+-" and s[1:].replace(".", "").isdigit():
                grip = int(round(grip + float(s)))
            else:
                try:
                    grip = int(round(float(s)))
                except ValueError:
                    print("  숫자 / +N / -N / o / c / q"); continue
            grip = max(0, min(180, grip))
            send(ser, base[:5] + [grip]); drain(ser)
            print(f"  → 집게 {grip} 전송")
    finally:
        send(ser, base[:5] + [grip]); time.sleep(0.3); drain(ser)
        ser.close()

    print(f"\n마지막 집게각 = {grip}")
    print("열림/닫힘 값 정하면 arm_ik.py 의 GRIPPER_OPEN / GRIPPER_CLOSED 갱신 → 알려주시면 반영해드림")


if __name__ == "__main__":
    main()
