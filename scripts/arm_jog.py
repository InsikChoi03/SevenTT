#!/usr/bin/env python3
"""팔 대화형 조그 — 연결을 '유지'해서 home detour 없이 연속 이동. arm_servo v2(smooth) 권장.

I/O는 단순: 명령을 보낸 뒤에만 응답을 읽어 출력(백그라운드 스레드 없음 → 프롬프트 안 깨짐).

명령(arm>):
  home               HOME 자세로
  x y z [approach]   좌표(cm, arm_base). approach 기본 0 (음수=아래)
  g <val>            그리퍼 (열림 35 / 닫힘 130)
  r <val>            손목 roll
  q                  종료
"""
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "ros2_ws", "src", "robot_control", "robot_control"))
import arm_ik  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"


def main():
    ser = serial.Serial(PORT, 115200, timeout=0.1)
    time.sleep(2.3)  # 최초 1회 리셋→home (이후 연결 유지, detour 없음)

    def drain(secs=0.35):
        end = time.time() + secs
        buf = b""
        while time.time() < end:
            d = ser.read(256)
            if d:
                buf += d
        for ln in buf.decode("ascii", "ignore").splitlines():
            ln = ln.strip()
            if ln:
                print("   ", ln)

    def send(cmd):
        ser.write(("<ARM," + ",".join(str(int(round(v))) for v in cmd) + ">\n").encode("ascii"))

    drain(0.5)  # 부팅 배너 표시
    last = list(arm_ik.HOME_CMD)
    print("명령: home / 'x y z [approach]' / g<val> / r<val> / q")

    try:
        while True:
            try:
                s = input("arm> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not s:
                continue
            if s == "q":
                break
            try:
                if s == "home":
                    last = list(arm_ik.HOME_CMD)
                    send(last)
                elif s[0] in "gG":
                    last[5] = float(s[1:].strip())
                    send(last)
                    print("    grip", last[5])
                elif s[0] in "rR":
                    last[4] = float(s[1:].strip())
                    send(last)
                    print("    roll", last[4])
                else:
                    p = s.split()
                    if len(p) < 3:
                        print("    형식: x y z [approach]  (예: 20 0 10 -30)")
                        continue
                    x, y, z = float(p[0]), float(p[1]), float(p[2])
                    ap = float(p[3]) if len(p) > 3 else 0.0
                    sol = arm_ik.ik_checked(x, y, z, ap)
                    if sol is None:
                        print("    도달불가/가동범위밖")
                        continue
                    cmd = [arm_ik.servo_cmd(j, sol[j]) for j in range(4)] + [last[4], last[5]]
                    last = cmd
                    print("    서보", [round(c) for c in cmd])
                    send(cmd)
            except ValueError:
                print("    입력 형식 오류 — 다시")
                continue
            drain(0.35)
    finally:
        ser.close()
        print("종료(연결 닫음)")


if __name__ == "__main__":
    main()
