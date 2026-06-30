#!/usr/bin/env python3
"""메카넘 합성 동작 테스트 — IK 부호 패턴으로 4휠 속도 만들어 <BASE,...> 전송.

휠 순서 fl,fr,rl,rr. 부호 패턴은 base_controller_node 의 역기구학과 일치:
    v_fl = vx - vy - k*w   v_fr = vx + vy + k*w
    v_rl = vx + vy - k*w   v_rr = vx - vy + k*w

⚠️ 전진/후진/회전은 공중 띄움 상태에서 '바퀴 방향 패턴'으로 확인 가능하지만,
   횡이동(strafe)의 실제 방향은 메카넘 롤러 방향에 달려서 **바닥 위에서만** 검증된다.
   바닥 테스트는 공간 확보 + 저속 + 즉시 정지 준비.

사용:
    python3 base_motion_test.py --motion forward --speed 0.25 --dur 2
    python3 base_motion_test.py --motion seq            # 전 동작 순차
"""
import argparse
import time

import serial

# (fl, fr, rl, rr) 부호 패턴
MOTIONS = {
    "forward": (+1, +1, +1, +1),   # 전진
    "back":    (-1, -1, -1, -1),   # 후진
    "left":    (-1, +1, +1, -1),   # 좌 횡이동
    "right":   (+1, -1, -1, +1),   # 우 횡이동
    "ccw":     (-1, +1, -1, +1),   # 제자리 좌회전(반시계)
    "cw":      (+1, -1, +1, -1),   # 제자리 우회전(시계)
}
SEQ = ["forward", "back", "left", "right", "ccw", "cw"]


def send(ser, sp):
    fl, fr, rl, rr = sp
    ser.write(f"<BASE,{fl:.3f},{fr:.3f},{rl:.3f},{rr:.3f}>\n".encode("ascii"))


def run_motion(ser, name, speed, dur, hz=20.0):
    sp = tuple(p * speed for p in MOTIONS[name])
    print(f"[{name}] fl,fr,rl,rr = {sp}  ({dur}s)")
    end = time.time() + dur
    period = 1.0 / hz
    while time.time() < end:
        send(ser, sp)
        time.sleep(period)
    send(ser, (0, 0, 0, 0))
    ser.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--speed", type=float, default=0.25)
    ap.add_argument("--dur", type=float, default=2.0)
    ap.add_argument("--motion", choices=list(MOTIONS) + ["seq"], default="seq")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(2.2)
    ser.reset_input_buffer()
    try:
        if args.motion == "seq":
            for m in SEQ:
                run_motion(ser, m, args.speed, args.dur)
                time.sleep(1.0)
        else:
            run_motion(ser, args.motion, args.speed, args.dur)
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        for _ in range(5):
            send(ser, (0, 0, 0, 0))
            time.sleep(0.02)
        ser.flush()
        ser.close()
        print("정지 + 닫음")


if __name__ == "__main__":
    main()
