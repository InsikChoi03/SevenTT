#!/usr/bin/env python3
"""개루프 look-then-grab 픽업: (스캔자세) 캡처 → 객체 검출 → H → IK → 하강 → grab → 들어올림.

검출기 2종:
  - 기본(HSV 색):   --color red|green|blue|yellow
  - YOLO 큐브:       --yolo   (models/cube.pt, cls 0=cube 만 잡음 → "큐브만")

⚠️ 서보 전원 ON + 팔 주변 공간 + 받칠 준비. 처음엔 --dry 로 좌표만 확인 후 실제 실행.

  python3 pick_run.py --yolo --dry                     # 안 움직이고 큐브검출+좌표만
  python3 pick_run.py --yolo --table-z 0 --approach -45 # 실제 픽업
  python3 pick_run.py --color red --dry                 # 색검출 모드(구버전)

전제: pick_calib.py 로 data/pick/homography.npz 만들어 둠 (픽셀→테이블 x,y cm).
  본체캠은 베이스 고정이라 H는 한 번 캘리브하면 영구 유효(팔 자세 무관).
  (캠이 팔에 달렸을 때만 --scan-pose 를 캘리브 때와 동일하게 맞춰야 함.)
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from csi_capture import CsiCamera  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "ros2_ws", "src", "robot_control", "robot_control"))
import arm_ik  # noqa: E402

# HSV 색 범위 (필요시 조정)
HSV = {
    "red":   [(0, 120, 80), (10, 255, 255)],
    "red2":  [(170, 120, 80), (180, 255, 255)],
    "green": [(40, 80, 60), (85, 255, 255)],
    "blue":  [(95, 120, 60), (130, 255, 255)],
    "yellow": [(20, 120, 120), (35, 255, 255)],
}


def detect_color(frame, color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lo, hi = HSV[color]
    mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
    if color == "red":
        lo2, hi2 = HSV["red2"]
        mask |= cv2.inRange(hsv, np.array(lo2), np.array(hi2))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 200:
        return None, None
    m = cv2.moments(c)
    px = (m["m10"] / m["m00"], m["m01"] / m["m00"])
    x, y, w, h = cv2.boundingRect(c)
    return px, (x, y, x + w, y + h, 1.0)


def detect_cube_yolo(frame, model, conf, cube_cls, anchor):
    """cube.pt 로 큐브(cls 0)만 검출. 가장 높은 conf 1개 반환.

    anchor='bottom' → bbox 바닥중앙(테이블 접점, 큐브 높이 시차 줄임), 'center' → 중심.
    반환: (픽셀(u,v), (x1,y1,x2,y2,conf))  또는 (None, None).
    """
    res = model.predict(frame, conf=conf, imgsz=640, verbose=False)
    if not res:
        return None, None
    boxes = res[0].boxes
    if boxes is None or len(boxes) == 0:
        return None, None
    xywh = boxes.xywh.cpu().numpy()
    confs = boxes.conf.cpu().numpy()
    clses = boxes.cls.cpu().numpy().astype(int)
    idxs = [i for i in range(len(clses)) if clses[i] == cube_cls]
    if not idxs:
        return None, None
    i = max(idxs, key=lambda k: float(confs[k]))   # 큐브 여러 개면 가장 확실한 것
    cx, cy, w, h = (float(v) for v in xywh[i])
    v = cy + h / 2.0 if anchor == "bottom" else cy
    box = (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2, float(confs[i]))
    return (cx, v), box


def save_detect(frame, px, box, path):
    disp = frame.copy()
    if box is not None:
        x1, y1, x2, y2, cf = box
        cv2.rectangle(disp, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(disp, f"cube {cf:.2f}", (int(x1), int(y1) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    if px is not None:
        cv2.circle(disp, (int(px[0]), int(px[1])), 8, (0, 0, 255), -1)   # 잡을 픽셀(테이블 접점)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, disp)


def send(ser, c):
    ser.write(("<ARM," + ",".join(str(int(round(v))) for v in c) + ">\n").encode("ascii"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--yolo", action="store_true", help="YOLO 큐브검출(cube.pt) 사용")
    ap.add_argument("--model", default="models/cube.pt")
    ap.add_argument("--conf", type=float, default=0.5, help="YOLO conf 임계")
    ap.add_argument("--cube-class", type=int, default=0, help="cube.pt 의 cube 클래스 id")
    ap.add_argument("--anchor", choices=["bottom", "center"], default="bottom",
                    help="잡을 픽셀: bottom=bbox 바닥중앙(테이블 접점, 권장) / center=중심")
    ap.add_argument("--color", default="red", choices=[k for k in HSV if k != "red2"],
                    help="(--yolo 아닐 때) HSV 색")
    ap.add_argument("--scan-pose", default=",".join(str(v) for v in arm_ik.HOME_CMD),
                    help="스캔자세 서보 6값. 베이스고정캠은 팔을 화면 밖에 두는 용도(기본 HOME)")
    ap.add_argument("--table-z", type=float, default=0.0, help="테이블 높이 (arm_base cm)")
    ap.add_argument("--grab-z", type=float, default=None, help="잡는 높이 (기본=table-z)")
    ap.add_argument("--approach", type=float, default=-85.0,
                    help="접근 피치 deg (음수=아래). 집게가 top-down(인형뽑기)형이라 수직에 가깝게. "
                         "순수 -90은 호버까지 되는 영역이 매우 좁음(앞 8~12cm) → -85~-80 권장(앞 ~17cm까지)")
    ap.add_argument("--clearance", type=float, default=3.0,
                    help="하강 전 호버 높이 cm. 팔이 길어 잡기지점 위 여유가 작음 → 6은 도달불가 흔함, 2~3 권장")
    ap.add_argument("--save-detect", default="data/pick/cube_detect.png",
                    help="검출 결과 주석 이미지 저장 경로(검증용)")
    ap.add_argument("--dry", action="store_true", help="팔 안 움직이고 검출+좌표만")
    args = ap.parse_args()

    scan = [float(x) for x in args.scan_pose.split(",")]
    grab_z = args.table_z if args.grab_z is None else args.grab_z

    # 호모그래피: 없으면 --dry 검출까진 허용(좌표/IK만 생략), 실제 픽업은 차단.
    hom_path = "data/pick/homography.npz"
    H = None
    if os.path.exists(hom_path):
        H = np.load(hom_path)["H"]
    elif not args.dry:
        print(f"✗ {hom_path} 없음 — 먼저 pick_calib.py 로 호모그래피 캘리브 필요"); return
    else:
        print(f"⚠ {hom_path} 없음 — 검출만 수행(테이블좌표/IK 생략). 캘리브 후 좌표까지 나옴.")

    # YOLO 모델은 --yolo 일 때만 로드(느린 import 회피)
    model = None
    if args.yolo:
        from ultralytics import YOLO  # noqa: E402
        model = YOLO(args.model)

    ser = None
    if not args.dry:
        ser = serial.Serial(args.port, 115200, timeout=0.2)
        time.sleep(2.3)
        send(ser, scan)
        time.sleep(2.0)  # 스캔자세 안착(팔을 화면 밖으로)

    cam = CsiCamera(0)  # 본체 cam = sensor-id 0 (camera_config.BODY). 광각(1)은 천장 향함
    try:
        for _ in range(5):
            cam.read(1.0)
        frame = cam.read(1.0)
    finally:
        cam.release()
    if frame is None:
        print("캡처 실패")
        if ser is not None:
            ser.close()
        return

    if args.yolo:
        px, box = detect_cube_yolo(frame, model, args.conf, args.cube_class, args.anchor)
        miss = f"큐브 못 찾음 (conf={args.conf}/조명/거리 확인)"
    else:
        px, box = detect_color(frame, args.color)
        miss = f"'{args.color}' 객체 못 찾음 (색 범위/조명 확인)"

    if args.save_detect:
        save_detect(frame, px, box, args.save_detect)
        print(f"검출 이미지 저장 → {args.save_detect}")

    if px is None:
        print(miss)
        if ser is not None:
            ser.close()
        return

    cf = box[4] if box is not None else 0.0
    print(f"큐브 검출 conf={cf:.2f}  픽셀 {tuple(round(p) for p in px)} (anchor={args.anchor})")

    if H is None:   # 캘리브 전 --dry: 픽셀까지만
        print("(호모그래피 없음 → 테이블좌표/IK 생략)")
        return

    xy = cv2.perspectiveTransform(np.array([[list(px)]], np.float32), H)[0][0]
    x, y = float(xy[0]), float(xy[1])
    print(f"  → 테이블 ({x:.1f}, {y:.1f}) cm")

    hover = arm_ik.ik_checked(x, y, grab_z + args.clearance, args.approach)
    grab = arm_ik.ik_checked(x, y, grab_z, args.approach)
    if hover is None or grab is None:
        print(f"IK 도달불가 (x={x:.1f} y={y:.1f}). approach/위치 조정 필요")
        if ser is not None:
            ser.close()
        return
    hov = [arm_ik.servo_cmd(j, hover[j]) for j in range(4)] + [arm_ik.WRIST_ROLL_HOME, arm_ik.GRIPPER_OPEN]
    grb = [arm_ik.servo_cmd(j, grab[j]) for j in range(4)] + [arm_ik.WRIST_ROLL_HOME, arm_ik.GRIPPER_OPEN]
    print("  hover 서보:", [round(v) for v in hov])
    print("  grab  서보:", [round(v) for v in grb])
    if args.dry:
        print("(--dry: 팔 안 움직임)")
        return

    # 시퀀스: 호버(열림) → 하강(열림) → 닫기 → 들어올림(닫은채) → 스캔자세
    print("호버..."); send(ser, hov); time.sleep(1.8)
    print("하강..."); send(ser, grb); time.sleep(1.8)
    print("집게 닫기..."); grbC = grb[:5] + [arm_ik.GRIPPER_CLOSED]; send(ser, grbC); time.sleep(1.2)
    print("들어올림..."); send(ser, hov[:5] + [arm_ik.GRIPPER_CLOSED]); time.sleep(1.8)
    print("스캔 복귀..."); send(ser, scan); time.sleep(1.5)
    ser.close()
    print("픽업 시퀀스 완료")


if __name__ == "__main__":
    main()
