#!/usr/bin/env python3
"""하드코딩 pick-and-place — 앞 물체 잡아 → 올려 → 팔꿈치 접어 뒤로 → 본체 보관함에 넣기. 카메라 없음.

= "잡아서 본체 보관함(트레이)에 떨어뜨리는" 동선 (place 자세: 어깨 덜 재끼고 팔꿈치/손목 접음, place-wrist 30).

베이스 yaw 고정(안 돌림). 수직평면 안에서 앞(reach+)/뒤(reach−)만 — 뒤는 어깨를 꺾어 도달.
뒤는 바닥이 아니라 '적당한 높이(~20cm)'에 놓음(짧은 팔 기하 한계). arm_servo v2 전제.
⚠️ 서보 전원 ON + 공간 + 받칠 준비. 먼저 --dry 로 서보값/도달 확인.

  python3 pick_place.py --dry
  python3 pick_place.py --pick-reach 18 --place-reach -9 --place-z 20 --speed 25
"""
import argparse
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "ros2_ws", "src", "robot_control", "robot_control"))
import arm_ik  # noqa: E402


def lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(6)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    # 앞 픽업
    ap.add_argument("--pick-reach", type=float, default=18.0, help="앞 물체 거리 cm")
    ap.add_argument("--pick-approach", type=float, default=-45.0, help="픽업 접근각(음수=아래)")
    ap.add_argument("--table-z", type=float, default=0.0)
    ap.add_argument("--grab-z", type=float, default=5.0, help="잡는 높이(마진상 너무 낮으면 안 됨)")
    ap.add_argument("--clearance", type=float, default=8.0, help="호버 높이(테이블 위)")
    ap.add_argument("--lift", type=float, default=14.0, help="들어올린 높이")
    # 뒤 놓기 (허리 꺾어)
    ap.add_argument("--place-reach", type=float, default=-8.0, help="뒤 거리 cm(음수=뒤)")
    ap.add_argument("--place-z", type=float, default=28.0, help="뒤 놓는 높이(본체 위) cm")
    ap.add_argument("--place-approach", type=float, default=60.0, help="뒤 접근각(팔꿈치 접기 자세)")
    ap.add_argument("--place-wrist", type=float, default=30.0,
                    help="뒤 놓기 손목(ch3) servo. 기본 30 = 보관함 넣는 동선(IK값과 반대로 꺾음). IK값 쓰려면 152")
    ap.add_argument("--speed", type=float, default=25.0, help="관절 속도 deg/s")
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args()

    OPEN, CLOSE = arm_ik.GRIPPER_OPEN, arm_ik.GRIPPER_CLOSED

    def fpose(reach, z, approach, grip, eu=None, wrist=None):
        if eu is None:
            sol = arm_ik.ik_checked_planar(reach, z, approach)
        else:                                    # elbow 분기 강제 (뒤 놓기 = 어깨 덜/팔꿈치 접는 자세)
            sol = arm_ik.ik_planar(reach, z, approach, elbow_up=eu)
            if sol is not None and not arm_ik.in_limits(sol):
                sol = None
        if sol is None:
            return None
        s = [arm_ik.servo_cmd(j, sol[j]) for j in range(4)]
        if wrist is not None:                    # 손목(ch3) 직접 지정 (그리퍼 방향만 바꿈)
            s[3] = max(12, min(168, wrist))
        return s + [arm_ik.WRIST_ROLL_HOME, grip]

    pr, pap = args.pick_reach, args.pick_approach
    z_hover = args.table_z + args.clearance
    z_grab = args.grab_z
    z_lift = args.table_z + args.lift
    qr, qz, qap = args.place_reach, args.place_z, args.place_approach

    steps = [
        ("픽 호버(열림)", fpose(pr, z_hover, pap, OPEN), 0.2),
        ("픽 하강",       fpose(pr, z_grab, pap, OPEN), 0.2),
        ("집기(닫기)",    fpose(pr, z_grab, pap, CLOSE), 0.7),
        ("올리기",        fpose(pr, z_lift, pap, CLOSE), 0.3),
        ("팔꿈치 접어 뒤로", fpose(qr, qz, qap, CLOSE, eu=True, wrist=args.place_wrist), 0.4),
        ("놓기(열기)",     fpose(qr, qz, qap, OPEN, eu=True, wrist=args.place_wrist), 0.7),
        ("HOME",         list(arm_ik.HOME_CMD), 0.3),
    ]

    for name, p, _ in steps:
        if p is None:
            print(f"  ✗ '{name}' 도달불가 — reach/z/approach 조정 필요")
            return
    if args.dry:
        print(f"pick: 앞 {pr}cm (z {z_grab}, ap {pap})  →  place: 뒤 {qr}cm z {qz} ap {qap}  "
              f"(베이스 고정, 속도 {args.speed}°/s)")
        for name, p, _ in steps:
            print(f"  {name:16s} 서보 {[round(v) for v in p]}")
        return

    ser = serial.Serial(args.port, 115200, timeout=0.2, write_timeout=2.0)
    time.sleep(2.3)
    cur = list(arm_ik.HOME_CMD)
    dt = 0.04

    def move(target):
        nonlocal cur
        dmax = max(abs(target[i] - cur[i]) for i in range(6))
        n = max(int((dmax / args.speed) / dt), 1)
        for k in range(1, n + 1):
            p = lerp(cur, target, k / n)
            ser.write(("<ARM," + ",".join(str(int(round(v))) for v in p) + ">\n").encode("ascii"))
            if ser.in_waiting:
                ser.read(ser.in_waiting)   # 펌웨어 ACK 비우기 (안 비우면 버퍼 차서 멈춤)
            time.sleep(dt)
        cur = list(target)

    ok = True
    try:
        for name, p, pause in steps:
            print(" ", name)
            sys.stdout.flush()
            move(p)
            if pause:
                time.sleep(pause)
    except serial.SerialTimeoutException:
        ok = False
        print("\n⚠️ 쓰기 멈춤 — 펌웨어 멈춤(I2C/리셋?). arm_servo 최신(setWireTimeout)인지 확인")
    except (serial.SerialException, OSError) as e:
        ok = False
        print(f"\n⚠️ 시리얼 끊김 ({e}) — 보드 리셋/USB 분리(서보 전원?)")
    except KeyboardInterrupt:
        ok = False
        print("\n중단됨")
    finally:
        try:
            ser.close()
        except Exception:
            pass
    if ok:
        print("pick-and-place 완료 ✅")


if __name__ == "__main__":
    main()
