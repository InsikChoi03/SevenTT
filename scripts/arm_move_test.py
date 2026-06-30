#!/usr/bin/env python3
"""팔 직접 테스트 — arm_ik로 좌표→서보명령 만들어 <ARM,...> 전송 (arm_servo 펌웨어 필요).

ROS 없이 한 자세씩 검증. ⚠️ 서보 전원 ON + 주변 공간 + 받칠 준비. 느리게.

예:
  python3 scripts/arm_move_test.py --home
  python3 scripts/arm_move_test.py --xyz 15 0 12 --approach 0
  python3 scripts/arm_move_test.py --xyz 14 6 8 --approach -30 --grip 130
"""
import argparse
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "ros2_ws", "src", "robot_control", "robot_control"))
import arm_ik  # noqa: E402


def send(ser, cmd):
    ser.write(("<ARM," + ",".join(str(int(round(v))) for v in cmd) + ">\n").encode("ascii"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--home", action="store_true", help="HOME 자세로")
    ap.add_argument("--xyz", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="목표 끝점 cm (arm_base 기준: x앞 y왼 z위)")
    ap.add_argument("--approach", type=float, default=0.0, help="접근피치 deg (0=수평, 음수=아래)")
    ap.add_argument("--roll", type=float, default=arm_ik.WRIST_ROLL_HOME)
    ap.add_argument("--grip", type=float, default=arm_ik.GRIPPER_OPEN,
                    help=f"그리퍼 서보각 (열림 {arm_ik.GRIPPER_OPEN} / 닫힘 {arm_ik.GRIPPER_CLOSED})")
    args = ap.parse_args()

    if args.home:
        cmd = list(arm_ik.HOME_CMD)
        print("HOME →", cmd)
    elif args.xyz:
        x, y, z = args.xyz
        sol = arm_ik.ik_checked(x, y, z, args.approach)
        if sol is None:
            print(f"({x},{y},{z}) 접근{args.approach}° → 도달불가/가동범위밖")
            return
        cmd = [arm_ik.servo_cmd(j, sol[j]) for j in range(4)] + [args.roll, args.grip]
        print(f"θ(기구학)={tuple(round(s, 1) for s in sol)}  →  서보 {[round(c, 1) for c in cmd]}")
    else:
        print("--home 또는 --xyz X Y Z 필요")
        return

    ser = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(2.2)
    send(ser, cmd)
    time.sleep(0.4)
    print("ACK:", ser.read(200).decode("ascii", "ignore").strip())
    ser.close()


if __name__ == "__main__":
    main()
