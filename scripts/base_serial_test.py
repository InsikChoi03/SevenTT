#!/usr/bin/env python3
"""메카넘 베이스 UNO 시리얼 브링업 테스트 (ROS 없이 순수 pyserial).

포트를 열고 (1) 펌웨어 텔레메트리(<ODOM,...>)를 잠깐 들은 뒤,
(2) 바퀴를 하나씩 저속으로 짧게 jog 한다. 명령은 연속 재전송하고
종료/예외 시 반드시 0속도를 보낸 뒤 닫는다 → 폭주 방지.

프로토콜 (memory: project-serial-protocols):
    out: <BASE,fl,fr,rl,rr>\\n   휠 선속도 m/s, 부호 포함, 소수 3자리
    in : <ODOM,fl,fr,rl,rr,t_ms>\\n
휠 순서: FL(앞왼) FR(앞오) RL(뒤왼) RR(뒤오)

⚠️ 반드시 바퀴를 공중에 띄운 상태 + 베이스 12V 전원 ON 에서 실행할 것.

예:
    python3 base_serial_test.py --port /dev/ttyACM0          # 4바퀴 순차 jog
    python3 base_serial_test.py --port /dev/ttyACM0 --wheel 0 # FL만
    python3 base_serial_test.py --port /dev/ttyACM0 --listen 5 --no-jog  # 텔레메트리만 관찰
"""
from __future__ import annotations

import argparse
import sys
import time

import serial

WHEELS = ["FL", "FR", "RL", "RR"]


def send(ser: serial.Serial, speeds) -> None:
    fl, fr, rl, rr = speeds
    ser.write(f"<BASE,{fl:.3f},{fr:.3f},{rl:.3f},{rr:.3f}>\n".encode("ascii"))


def stop(ser: serial.Serial) -> None:
    send(ser, (0.0, 0.0, 0.0, 0.0))


def drain(ser: serial.Serial, secs: float) -> int:
    """secs 동안 들어오는 라인을 읽어 출력. 받은 라인 수 반환."""
    end = time.monotonic() + secs
    buf = b""
    n = 0
    while time.monotonic() < end:
        data = ser.read(64)
        if not data:
            continue
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            s = line.strip().decode("ascii", errors="ignore")
            if s:
                print("  RX:", s)
                n += 1
    return n


def jog(ser: serial.Serial, idx: int, speed: float, dur: float, hz: float) -> None:
    speeds = [0.0, 0.0, 0.0, 0.0]
    speeds[idx] = speed
    print(f"[jog] {WHEELS[idx]} @ {speed:+.3f} m/s for {dur:.1f}s  (관찰: 어느 바퀴가 어느 방향으로 도는지)")
    end = time.monotonic() + dur
    period = 1.0 / hz
    while time.monotonic() < end:
        send(ser, speeds)
        time.sleep(period)
    stop(ser)
    ser.flush()


def main() -> None:
    ap = argparse.ArgumentParser(description="Mecanum base serial bring-up jog test")
    ap.add_argument("--port", default="/dev/ttyACM0", help="시리얼 포트 (정품 UNO=/dev/ttyACM0, CH340=/dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=float, default=0.12, help="jog 휠 속도 m/s (저속부터)")
    ap.add_argument("--dur", type=float, default=1.5, help="바퀴당 jog 시간 s")
    ap.add_argument("--hz", type=float, default=20.0, help="명령 재전송 주기")
    ap.add_argument("--listen", type=float, default=2.0, help="시작 시 텔레메트리 청취 시간 s")
    ap.add_argument("--wheel", type=int, default=-1, help="이 인덱스(0..3)만 jog. -1=4바퀴 순차")
    ap.add_argument("--no-jog", action="store_true", help="jog 없이 텔레메트리만 관찰")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except (serial.SerialException, OSError) as e:
        print(f"포트 열기 실패 ({args.port}): {e}", file=sys.stderr)
        print("  → 포트 이름 확인(ls /dev/ttyACM* /dev/ttyUSB*) 또는 권한(dialout/sudo) 확인", file=sys.stderr)
        sys.exit(1)

    # UNO는 시리얼 open 시 DTR로 리셋됨 → 부트로더 대기
    print(f"포트 열림: {args.port} @ {args.baud}  (UNO 부트로더 대기 2s)")
    time.sleep(2.0)
    ser.reset_input_buffer()
    stop(ser)

    print(f"== 텔레메트리 {args.listen:.1f}s 청취 ==")
    got = drain(ser, args.listen)
    if got == 0:
        print("  (수신 라인 없음 — 펌웨어가 <ODOM>을 안 보내거나 엔코더 없을 수 있음. jog는 계속 진행)")

    try:
        if not args.no_jog:
            idxs = range(4) if args.wheel < 0 else [args.wheel]
            for i in idxs:
                jog(ser, i, args.speed, args.dur, args.hz)
                drain(ser, 0.5)
                time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n중단됨 — 정지 명령 전송")
    finally:
        for _ in range(5):
            stop(ser)
            time.sleep(0.02)
        ser.flush()
        ser.close()
        print("정지 + 포트 닫음.")


if __name__ == "__main__":
    main()
