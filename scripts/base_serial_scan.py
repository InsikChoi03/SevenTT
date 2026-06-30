#!/usr/bin/env python3
"""베이스 UNO 시리얼 진단: baud 스캔 + 부트 배너 + 명령 응답 sniff.

각 baud 후보로 포트를 열고:
  1) DTR 리셋 → 스케치 부트 직후 출력(배너)을 raw로 캡처
  2) <BASE,...> 명령을 잠깐 퍼붓고 응답을 캡처
출력의 printable 비율로 "이 baud가 맞다 / 깨진다 / 아무 말 없다"를 판별.

해석:
  - 어떤 baud에서 <...> 프레임이나 깨끗한 ASCII가 보임  → 그 baud가 펌웨어 속도
  - 한 baud만 printable 높고 나머진 깨짐                → 그 baud가 정답
  - 전 baud에서 0바이트                                → 펌웨어가 시리얼로 아무것도 안 보냄
                                                          (순정 리모컨 펌웨어 의심 or 송신 안 하는 펌웨어)

사용: python3 base_serial_scan.py [/dev/ttyUSB0]
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUDS = [115200, 57600, 38400, 19200, 9600]


def printable_ratio(data: bytes) -> str:
    if not data:
        return "0/0"
    p = sum(1 for b in data if b in (9, 10, 13) or 32 <= b <= 126)
    return f"{p}/{len(data)}"


def sniff(baud: int) -> None:
    try:
        ser = serial.Serial(PORT, baud, timeout=0.2)
    except (serial.SerialException, OSError) as e:
        print(f"[{baud:>6}] 포트 열기 실패: {e}")
        return

    # DTR 토글로 UNO 리셋 → 부트 배너 유도
    try:
        ser.setDTR(False)
        time.sleep(0.1)
        ser.setDTR(True)
    except OSError:
        pass
    time.sleep(2.0)  # 부트로더 + 스케치 시작 대기

    boot = ser.read(4096)

    # 명령 퍼붓고 응답 보기
    ser.reset_input_buffer()
    for _ in range(20):
        ser.write(b"<BASE,0.300,0.300,0.300,0.300>\n")
        time.sleep(0.05)
    resp = ser.read(4096)
    # 안전: 정지
    ser.write(b"<BASE,0.000,0.000,0.000,0.000>\n")
    ser.flush()
    ser.close()

    print(f"[{baud:>6}] boot={len(boot):>4}B (printable {printable_ratio(boot)})  "
          f"resp={len(resp):>4}B (printable {printable_ratio(resp)})")
    if boot:
        print(f"         boot raw: {boot[:160]!r}")
    if resp:
        print(f"         resp raw: {resp[:160]!r}")


def main() -> None:
    print(f"== {PORT} baud 스캔 (배너+명령응답 sniff) ==")
    print("   ⚠️ 모터를 돌려보려면 12V ON + 바퀴 공중. (이 스캔은 0.3 m/s 명령을 잠깐 보냄)")
    for b in BAUDS:
        sniff(b)
        time.sleep(0.3)
    print("끝. printable 비율이 높고 <...> 프레임 보이는 baud가 펌웨어 속도.")


if __name__ == "__main__":
    main()
