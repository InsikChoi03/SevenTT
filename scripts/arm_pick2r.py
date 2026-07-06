#!/usr/bin/env python3
"""2R 팔(어깨/손목/그리퍼) 집기·놓기 — 재사용 모듈 + CLI 테스트.

arm_jog2.py로 실측·검증한 서보각(사용자 확인: "잘 잡는다")을 그대로 재생한다.
arm_ik.py는 아직 HOME_CMD/GRIPPER 캘리브 전(TODO)이라, 검증된 raw 서보각을 쓴다.

포즈 (ch0=어깨, ch1=손목, ch2=그리퍼 / t4~6=90 필러 미사용):
  INIT  어깨 88, 손목 10, 그리퍼 140   ← 연결 직후 스냅하는 초기/대기 자세
  집기  어깨 10, 손목 170 이동 → 딜레이 → 그리퍼 150(닫힘)
  놓기  어깨 110, 손목 30 이동(집은 채) → 딜레이 → 그리퍼 80(열림) → 초기복귀 전 대기(--init-delay)

펌웨어가 자체 smooth 보간(MAX_STEP 2°/30ms ≈ 67°/s)이라 목표각만 보내면 됨(호스트 lerp 불필요).
각 이동은 **거리/속도로 도달시간을 계산해 팔이 다 도착할 때까지 기다린 뒤** move/grip 딜레이만큼
더 대기하고 다음 동작(그리퍼)으로 넘어간다 → 도착 전에 그리퍼가 급히 닫히는 문제 방지.
⚠️ 첫 <ARM> 명령은 boot-limp 해제와 함께 즉시 스냅 → 연결 직후 INIT(88,10,140)으로 흡수.

── 모듈로 쓰기 (추후 pick_run / mission_fsm 등에서) ──
    from arm_pick2r import Arm2R
    with Arm2R("/dev/ttyUSB0") as arm:   # 열고 INIT로 스냅
        arm.grip()                        # 집기: pick 자세 → 닫기
        arm.release()                     # 놓기: place 자세 → 열기
        arm.go_init()                     # 대기 자세 복귀

── CLI 테스트 ──
    python3 scripts/arm_pick2r.py               # 집기 → 놓기 1회
    python3 scripts/arm_pick2r.py --only grip
    python3 scripts/arm_pick2r.py --only release
    python3 scripts/arm_pick2r.py --loop 3
    python3 scripts/arm_pick2r.py --move-delay 1.0 --grip-delay 0.5
"""
from __future__ import annotations

import argparse
import glob
import time

# ── 검증된 포즈 (2026-07-03 arm_jog2.py 실측, 사용자 "잘 잡는다" 확인) ──
GRIP_OPEN = 80       # 그리퍼 열림(놓기)
GRIP_CLOSED = 150    # 그리퍼 닫힘(잡기)

INIT = {"shoulder": 88, "wrist": 10, "gripper": 140}         # 연결 직후/대기 자세
PICK = {"shoulder": 10, "wrist": 170}                         # 집기 도달 자세(어깨/손목)
PLACE = {"shoulder": 110, "wrist": 30}                        # 놓기 도달 자세(어깨/손목)

# 펌웨어 smooth 보간 속도(MAX_STEP 2°/30ms ≈ 67°/s). 안전 위해 보수적으로 잡아 도달을 확실히 기다림.
SERVO_SPEED_DPS = 60.0


class Arm2R:
    """2R 팔 시리얼 제어. with 문 또는 open()/close()로 사용. 그리퍼 상태를 추적해
    놓기 이동 중에는 잡은 채(닫힘) 유지한 뒤 마지막에만 연다."""

    def __init__(self, port="/dev/ttyUSB0", baud=115200,
                 move_delay=0.8, grip_delay=0.6, reset_wait=2.2):
        self.port = port
        self.baud = baud
        self.move_delay = move_delay      # 팔 도달 후 그리퍼 움직이기 전 추가 대기(초)
        self.grip_delay = grip_delay      # 그리퍼 개폐 후 다음 동작 전 추가 대기(초)
        self.reset_wait = reset_wait
        self.ser = None
        # 마지막으로 보낸 목표 자세 [어깨, 손목, 그리퍼] — 이동시간 계산용. open()에서 INIT로 스냅.
        self._pose = [INIT["shoulder"], INIT["wrist"], INIT["gripper"]]

    # ── 연결 ──
    def open(self):
        import serial
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(self.reset_wait)      # MCU 리셋 대기 (통합보드=베이스/리프트도 함께 리셋)
        self._drain(0.5)
        self.go_init()                   # 첫 <ARM> → boot-limp 해제, INIT로 스냅 흡수
        return self

    def close(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # ── 저수준 ──
    def _send(self, shoulder, wrist, gripper):
        t = [int(round(shoulder)), int(round(wrist)), int(round(gripper)), 90, 90, 90]
        self.ser.write(("<ARM," + ",".join(str(x) for x in t) + ">\n").encode())

    def _drain(self, s):
        end = time.time() + s
        while time.time() < end:
            self.ser.read(256)

    def _move(self, shoulder, wrist, gripper, dwell):
        """목표로 보내고 → 실제 도달할 때까지(이동거리/속도) 기다린 후 → dwell(딜레이) 추가 대기.
        그래야 팔이 다 도착한 뒤에 다음 동작(그리퍼)이 시작됨."""
        travel = max(abs(shoulder - self._pose[0]),
                     abs(wrist - self._pose[1]),
                     abs(gripper - self._pose[2])) / SERVO_SPEED_DPS
        self._send(shoulder, wrist, gripper)
        self._drain(travel + dwell)
        self._pose = [shoulder, wrist, gripper]

    # ── 동작 ──
    def go_init(self):
        """대기/초기 자세로. 그리퍼는 INIT 값."""
        self._move(INIT["shoulder"], INIT["wrist"], INIT["gripper"], self.move_delay)

    def grip(self):
        """집기: pick 자세로 이동(그리퍼 열고) → 도달+딜레이 → 그리퍼 닫기 → 딜레이."""
        self._move(PICK["shoulder"], PICK["wrist"], GRIP_OPEN, self.move_delay)    # 집기 위치로
        self._move(PICK["shoulder"], PICK["wrist"], GRIP_CLOSED, self.grip_delay)  # 잡기

    def release(self):
        """놓기: 잡은 채 place 자세로 이동 → 도달+딜레이 → 그리퍼 열기 → 딜레이."""
        self._move(PLACE["shoulder"], PLACE["wrist"], self._pose[2], self.move_delay)  # 놓기 위치로(잡은 채)
        self._move(PLACE["shoulder"], PLACE["wrist"], GRIP_OPEN, self.grip_delay)      # 놓기


# ── CLI 테스트 ──
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--only", choices=["grip", "release"], help="지정 시 해당 동작만")
    ap.add_argument("--loop", type=int, default=1, help="집기→놓기 반복 (--only 시 무시)")
    ap.add_argument("--move-delay", type=float, default=0.8, help="팔 도달 후 그리퍼 전 추가 대기(초)")
    ap.add_argument("--grip-delay", type=float, default=0.6, help="그리퍼 개폐 후 추가 대기(초)")
    ap.add_argument("--cycle-pause", type=float, default=0.5, help="집기↔놓기 사이 대기(초)")
    ap.add_argument("--init-delay", type=float, default=3.0, help="놓기 후 초기자세 복귀 전 대기(초)")
    args = ap.parse_args()

    try:
        import serial  # noqa: F401
    except ImportError:
        print("pyserial 필요: pip install pyserial")
        return 1

    try:
        arm = Arm2R(args.port, args.baud, args.move_delay, args.grip_delay)
        print("⚠️ 첫 <ARM>에 INIT(88,10,140)으로 스냅합니다. 그리퍼 밑에 손/물건 없는지 확인.")
        arm.open()
    except Exception as e:
        print(f"[err] 포트 {args.port} 열기 실패: {e}")
        print("가능 포트:", glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        return 1

    try:
        if args.only == "grip":
            print("  [집기]")
            arm.grip()
        elif args.only == "release":
            print("  [놓기]")
            arm.release()
        else:
            for i in range(args.loop):
                print(f"=== cycle {i + 1}/{args.loop} ===")
                print("  [집기] 어깨10/손목170 → 그리퍼150")
                arm.grip()
                time.sleep(args.cycle_pause)
                print("  [놓기] 어깨110/손목30 → 그리퍼80")
                arm.release()
                if i < args.loop - 1:
                    time.sleep(args.cycle_pause)
            print(f"  [{args.init_delay:.0f}초 대기 후 초기자세 복귀]")
            time.sleep(args.init_delay)
            arm.go_init()
    except KeyboardInterrupt:
        print("\n중단됨 — 서보 현재 위치 유지")
    finally:
        arm.close()
    print("[done]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
