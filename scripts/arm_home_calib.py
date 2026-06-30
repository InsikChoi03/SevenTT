#!/usr/bin/env python3
"""팔 home 보정 도우미 (home_calibration 스케치, 9600).

문제: miniterm은 키를 '한 글자씩' 보내서, 스케치의 parseInt(타임아웃 100ms)가
입력을 놓침 → "채널값 입력이 안 됨". 이 스크립트는 명령을 '한 줄 통째로' 보내 해결.

프롬프트(cal>) 명령:
  <ch> <val>     채널 ch 를 val(0~180)도로   예)  0 70
  preset         시작 추정값 일괄 적용 (--preset 로 변경 가능)
  p              현재 HOME_CMD 출력
  q              종료

⚠️ 연결 시 팔이 리셋되며 home 자세로 움직임 → 서보 전원 ON + 주변 공간 확보 + 받칠 준비.
"""
import argparse
import sys
import threading
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("--preset", default="70,96,55,55,90,90",
                    help="시작 home 값 6개 (ch0..ch5) — 최신 보정값")
    ap.add_argument("--apply-preset", action="store_true",
                    help="preset 적용만 하고 종료 (비대화)")
    ap.add_argument("--no-apply", action="store_true",
                    help="시작 시 자동 적용 안 함")
    args = ap.parse_args()
    preset = [int(x) for x in args.preset.split(",")]

    ser = serial.Serial(args.port, 9600, timeout=0.2)
    time.sleep(2.3)  # 보드 리셋(DTR) 후 부트 대기

    stop = {"v": False}

    def reader():
        while not stop["v"]:
            d = ser.read(256)
            if d:
                sys.stdout.write(d.decode("utf-8", "replace"))
                sys.stdout.flush()
    threading.Thread(target=reader, daemon=True).start()

    def send(line):
        ser.write((line.strip() + "\n").encode("ascii"))
        ser.flush()
        time.sleep(0.15)

    def apply_preset():
        print(f"\n[preset 적용] {preset}")
        for ch, val in enumerate(preset):
            send(f"{ch} {val}")
            time.sleep(0.3)
        send("p")

    # 시작 시 최신 home 자동 적용 (--no-apply 로 끄기)
    if not args.no_apply:
        apply_preset()
        time.sleep(0.6)

    if args.apply_preset:        # 비대화: 적용만 하고 종료
        stop["v"] = True
        time.sleep(0.2)
        ser.close()
        return

    print("\ncal> 명령: '<ch> <val>'  /  preset  /  p  /  q")
    try:
        while True:
            try:
                cmd = input("cal> ").strip()
            except EOFError:
                break
            if cmd == "q":
                break
            elif cmd == "preset":
                apply_preset()
            elif cmd == "p":
                send("p")
            elif cmd:
                send(cmd)   # "0 70" 같은 줄을 통째로 전송 (이게 핵심 수정)
    finally:
        stop["v"] = True
        time.sleep(0.2)
        ser.close()
        print("\n종료")


if __name__ == "__main__":
    main()
