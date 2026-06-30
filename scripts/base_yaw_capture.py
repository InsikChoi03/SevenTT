#!/usr/bin/env python3
"""팔은 HOME 유지, 베이스 yaw(joint0)만 좌우 스윕하며 본체캠 캡처.

카메라가 팔 base-yaw판에 달려 있어 yaw 회전 = 카메라 팬. 어깨/팔꿈치/손목은 HOME 고정
이라 reach 안 함 → 충돌·시야가림 위험 거의 없음(안전).
한 번 실행 = 한 배치(기본 10장). 다음 배치는 큐브 재배치 후 다시 실행.

  python3 scripts/base_yaw_capture.py                      # ±45°, 10장, 본체캠
  python3 scripts/base_yaw_capture.py --yaw-range 45 --shots 10
  python3 scripts/base_yaw_capture.py --dry-run            # 팔 안 움직이고 yaw 목록만
"""
from __future__ import annotations
import argparse, os, sys, time
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "ros2_ws", "src", "robot_control", "robot_control"))
import arm_ik  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402

OUT = "data/dataset/images"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--sensor-id", type=int, default=camera_config.BODY)
    ap.add_argument("--yaw-range", type=float, default=45.0, help="±도 (기본 45)")
    ap.add_argument("--shots", type=int, default=10)
    ap.add_argument("--settle", type=float, default=1.0, help="회전 후 정지 대기(s)")
    ap.add_argument("--roll", type=int, default=90)
    ap.add_argument("--grip", type=int, default=90)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    lo, hi = arm_ik.JOINT_LIMITS[0]                  # 베이스 가동범위 (안전 마진 포함)
    n = max(1, args.shots)
    yaws = [(-args.yaw_range + 2 * args.yaw_range * i / (n - 1)) for i in range(n)] if n > 1 else [0.0]
    clipped = [y for y in yaws if lo - 1e-6 <= y <= hi + 1e-6]
    if len(clipped) < len(yaws):
        print(f"[warn] yaw {lo}~{hi} 범위 밖 {len(yaws)-len(clipped)}개 제외")
    yaws = clipped
    print(f"[plan] 팔 HOME 고정, 베이스 yaw {yaws[0]:+.0f}..{yaws[-1]:+.0f}deg, {len(yaws)}장")
    if args.dry_run:
        for y in yaws:
            print(f"   yaw={y:+.1f}  servo0={round(arm_ik.servo_cmd(0, y))}")
        return 0

    import serial
    os.makedirs(OUT, exist_ok=True)
    ser = serial.Serial(args.port, 115200, timeout=0.1)
    time.sleep(2.3)                                  # MCU 리셋→home

    def drain(s=0.3):
        end = time.time() + s
        while time.time() < end:
            ser.read(256)                            # ACK 비워 펌웨어 교착 방지

    def send(cmd6):
        ser.write(("<ARM," + ",".join(str(int(round(v))) for v in cmd6) + ">\n").encode())
        drain(0.3)

    drain(0.5)
    send(list(arm_ik.HOME_CMD)); time.sleep(1.0)     # 어깨/팔꿈치/손목 HOME 확실히

    from csi_capture import CsiCamera
    cam = CsiCamera(args.sensor_id)
    for _ in range(8):
        cam.read(1.0)

    base = int(time.time()); saved = 0
    try:
        for i, y in enumerate(yaws):
            cmd = [arm_ik.servo_cmd(0, y), arm_ik.HOME_CMD[1], arm_ik.HOME_CMD[2],
                   arm_ik.HOME_CMD[3], args.roll, args.grip]   # joint0만 변경, 나머지 HOME
            send(cmd); time.sleep(args.settle)
            f = None
            for _ in range(3):
                g = cam.read(1.0)
                if g is not None:
                    f = g
            if f is None:
                print(f"  {i+1}/{len(yaws)} 캡처실패"); continue
            p = f"{OUT}/baseyaw_{base}_{i:02d}.jpg"
            cv2.imwrite(p, f); saved += 1
            print(f"  {i+1}/{len(yaws)} yaw={y:+.0f} -> {os.path.basename(p)}", flush=True)
        send(list(arm_ik.HOME_CMD))                  # 끝나면 정면 복귀
        time.sleep(0.5)
    finally:
        cam.release(); ser.close()
    print(f"[done] {saved}장 -> {OUT}  (확인 후 다음 배치는 큐브 바꾸고 다시 실행)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
