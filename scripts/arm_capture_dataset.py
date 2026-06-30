#!/usr/bin/env python3
"""로봇팔로 여러 시점에서 본체캠 자동 촬영 → 학습 데이터 수집.

eye-in-hand(본체캠=sensor-id 1)라 팔을 움직이면 같은 물체를 다양한 각도/거리에서 촬영.
'물체를 잘 보는' 기준 포즈(x,y,z,approach)를 중심으로 yaw/높이/리치/접근각을 조금씩
흔들어 스윕하고, 각 포즈에서 팔이 멈춘 뒤 한 장씩 캡처해 data/dataset/images 에 저장.

준비: ① 본체캠 스트리밍 정상(필요시 reboot)  ② arm_jog로 물체 잘 보이는 포즈 찾아 그 x y z approach 확인
사용:
  python3 scripts/arm_capture_dataset.py --x 18 --y 0 --z 6 --approach -45 \
      --dyaw 12 --dz 3 --dreach 3 --dapproach 15 --settle 1.2
주의: 작업영역 비우고, 팔이 물체/카메라/주변에 안 부딪히는지 먼저 arm_jog로 확인할 것.
"""
from __future__ import annotations
import argparse, os, sys, time, math
import cv2
import serial

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "ros2_ws", "src", "robot_control", "robot_control"))
import arm_ik  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402

OUT = "data/dataset/images"

# 보수적 안전 한계 (기구학각 deg) — arm_ik.JOINT_LIMITS보다 좁게.
# 목적: 어깨 과도 전방경사 / 집게의 카메라 가림·충돌 회피.
# ★ 실제 셋업(카메라 마운트)에 맞춰 조정하고, 반드시 arm_jog로 스윕 코너를 먼저
#   손으로 돌려 (집게가 시야 안 가리고 카메라에 안 부딪히는지) 확인한 뒤 사용 ★
SAFE_LIMITS = {
    0: (-45.0, 45.0),    # base yaw: 정면 ±45 (옆으로 과도 회전 제한)
    1: (55.0, 120.0),    # shoulder: 하한55=너무 앞으로 안 숙임 / 상한120=뒤로 과도X
    2: (-100.0, 10.0),   # elbow
    3: (-95.0, 20.0),    # wrist pitch: 집게가 카메라쪽으로 과도하게 꺾이지 않게
}


def within_safe(sol, safe):
    """기구학각 튜플이 보수적 안전 한계 안인가."""
    for j, a in enumerate(sol):
        lo, hi = safe[j]
        if a < lo or a > hi:
            return False
    return True


def gen_poses(cx, cy, cz, cap, dyaw, dz, dreach, dapproach, n, safe=None):
    """기준 (cx,cy,cz,cap) 둘레로 yaw/높이/리치/접근각 스윕. ik 가능 + 안전한계 통과만, 중복 제거."""
    safe = safe or SAFE_LIMITS
    R0 = math.hypot(cx, cy)
    Y0 = math.degrees(math.atan2(cy, cx))
    def lin(d, k):
        return [0.0] if (d == 0 or k <= 1) else [(-d + 2*d*i/(k-1)) for i in range(k)]
    poses, seen = [], set()
    for yaw in lin(dyaw, n):
        for dr in lin(dreach, n):
            for dzz in lin(dz, n):
                for da in lin(dapproach, n):
                    R = R0 + dr; Y = math.radians(Y0 + yaw)
                    x, y, z, ap = R*math.cos(Y), R*math.sin(Y), cz + dzz, cap + da
                    sol = arm_ik.ik_checked(x, y, z, ap)
                    if sol is None or not within_safe(sol, safe):
                        continue                       # 도달불가 or 보수적 안전한계 밖
                    cmd = tuple(round(arm_ik.servo_cmd(j, sol[j])) for j in range(4))
                    if cmd in seen:
                        continue
                    seen.add(cmd)
                    poses.append((cmd, (x, y, z, ap)))
    return poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--sensor-id", type=int, default=camera_config.BODY)
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, required=True)
    ap.add_argument("--approach", type=float, default=-45.0)
    ap.add_argument("--dyaw", type=float, default=12.0)
    ap.add_argument("--dz", type=float, default=3.0)
    ap.add_argument("--dreach", type=float, default=3.0)
    ap.add_argument("--dapproach", type=float, default=15.0)
    ap.add_argument("--steps", type=int, default=3, help="각 축 스텝 수(홀수 권장)")
    ap.add_argument("--shoulder-min", type=float, default=SAFE_LIMITS[1][0],
                    help="어깨 최소각(deg). 클수록 앞으로 덜 숙임=안전(기본 55)")
    ap.add_argument("--yaw-max", type=float, default=SAFE_LIMITS[0][1],
                    help="베이스 좌우 회전 최대(deg, ±, 기본 45)")
    ap.add_argument("--settle", type=float, default=1.2, help="이동 후 정지 대기(s)")
    ap.add_argument("--roll", type=int, default=90)
    ap.add_argument("--grip", type=int, default=90)
    ap.add_argument("--dry-run", action="store_true", help="포즈 수만 출력, 팔 안 움직임")
    args = ap.parse_args()

    safe = dict(SAFE_LIMITS)
    safe[1] = (args.shoulder_min, SAFE_LIMITS[1][1])      # 어깨 전방경사 하한 override
    safe[0] = (-abs(args.yaw_max), abs(args.yaw_max))     # 좌우 회전 범위 override
    poses = gen_poses(args.x, args.y, args.z, args.approach,
                      args.dyaw, args.dz, args.dreach, args.dapproach, args.steps, safe)
    print(f"[plan] 안전·도달가능 포즈 {len(poses)}개 "
          f"(기준 x={args.x} y={args.y} z={args.z} ap={args.approach}; "
          f"shoulder>={args.shoulder_min}, yaw±{abs(args.yaw_max)})")
    if not poses:
        print("도달 가능한 포즈 없음 — 기준 좌표/범위 조정"); return 1
    if args.dry_run:
        for c, p in poses[:8]:
            print("   servo", list(c), " <-", tuple(round(v, 1) for v in p))
        print("   ... (--dry-run)"); return 0

    os.makedirs(OUT, exist_ok=True)
    from csi_capture import CsiCamera
    ser = serial.Serial(args.port, 115200, timeout=0.1)
    time.sleep(2.3)  # MCU 리셋→home
    def drain(s=0.3):
        end = time.time()+s
        while time.time() < end: ser.read(256)   # ACK 비워 펌웨어 교착 방지
    def send(cmd6):
        ser.write(("<ARM," + ",".join(str(int(round(v))) for v in cmd6) + ">\n").encode()); drain(0.3)
    drain(0.5)

    cam = CsiCamera(args.sensor_id)
    for _ in range(8): cam.read(1.0)

    base = int(time.time()); saved = 0
    try:
        for i, (cmd, p) in enumerate(poses):
            send(list(cmd) + [args.roll, args.grip])
            time.sleep(args.settle)                 # 정지 + 모션블러 방지
            f = None
            for _ in range(3):
                g = cam.read(1.0)
                if g is not None: f = g
            if f is None:
                print(f"  {i+1}/{len(poses)} 캡처실패"); continue
            path = f"{OUT}/arm_{base}_{i:03d}.jpg"
            cv2.imwrite(path, f); saved += 1
            print(f"  {i+1}/{len(poses)} servo{list(cmd)} -> {os.path.basename(path)}", flush=True)
        # 끝나면 home 복귀
        send(list(arm_ik.HOME_CMD))
    finally:
        cam.release(); ser.close()
    print(f"[done] {saved}장 저장 -> {OUT}  (다음: python3 scripts/autolabel.py --process)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
