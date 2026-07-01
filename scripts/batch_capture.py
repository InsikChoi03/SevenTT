#!/usr/bin/env python3
"""데이터 수집 루프 (대화형):
  [Enter→촬영] → 탐지/라벨 → 몽타주 확인 → 거를 프레임번호 입력(prune) → 다음 배치 Enter → 반복

팔은 HOME 고정, base yaw만 ±회전(안전). 카메라가 팔 base-yaw판 장착이라 yaw로 팬.
각 배치 = base yaw 스윕 N장 → --class로 라벨(FastSAM 박스 + SigLIP 배경거름, 클래스는 고정).

  python3 scripts/batch_capture.py --loop --class cube     # ★대화형 반복 수집 (권장)
  python3 scripts/batch_capture.py --class cube            # 1배치만 (비대화)
  python3 scripts/batch_capture.py --prune <id> --remove 0,7
"""
from __future__ import annotations
import argparse, os, sys, time, glob
import cv2, numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "ros2_ws", "src", "robot_control", "robot_control"))
import arm_ik  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402
import box_utils  # noqa: E402
import autolabel  # noqa: E402

MONT = "data/cam_test/batch_montage.png"
_COL = [(0, 255, 0), (0, 200, 255), (255, 150, 0), (255, 0, 200)]
_yolo = None


def find_latest_best():
    g = glob.glob("runs/**/weights/best.pt", recursive=True)
    return max(g, key=lambda p: os.path.getmtime(p)) if g else None


def yolo_label(path, model_path, conf):
    """학습한 YOLO로 한 장 라벨 → (yolo_lines, review, flags). 박스+클래스 동시(혼합장면 OK)."""
    global _yolo
    if _yolo is None:
        from ultralytics import YOLO
        _yolo = YOLO(model_path)
    frame = cv2.imread(path)
    if frame is None:
        return None
    H, W = frame.shape[:2]
    r = _yolo.predict(frame, imgsz=640, conf=conf, verbose=False, agnostic_nms=True, iou=0.6)[0]
    names = _yolo.names
    name2id = {v: k for k, v in names.items()}
    # 부분-전체 쌍: 과일패치는 큐브 안에 nested → dedupe에서 서로 억제 금지(둘 다 유지)
    exempt = ({frozenset((name2id["cube"], name2id["fruit_photo_cube"]))}
              if "cube" in name2id and "fruit_photo_cube" in name2id else None)
    lines = []; review = frame.copy(); flags = []
    dets = []
    if r.boxes is not None:
        for b in r.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            dets.append((x1, y1, x2, y2, int(b.cls[0]), float(b.conf[0])))
    for (x1, y1, x2, y2, cls, cf) in box_utils.dedupe(dets, nested_exempt=exempt):   # 중복/박스속박스 제거(단 cube⊃patch는 유지)
        cx = ((x1 + x2) / 2) / W; cy = ((y1 + y2) / 2) / H
        w = (x2 - x1) / W; h = (y2 - y1) / H
        lines.append(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
        col = _COL[cls % len(_COL)]
        cv2.rectangle(review, (int(x1), int(y1)), (int(x2), int(y2)), col, 3)
        weak = cf < 0.5
        tag = f"{names[cls]} {cf:.2f}" + (" ?" if weak else "")
        if weak:
            flags.append(f"{names[cls]}({cf:.2f})")
        cv2.putText(review, tag, (int(x1), max(30, int(y1) - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 0), 6)
        cv2.putText(review, tag, (int(x1), max(30, int(y1) - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.6, col, 2)
    return lines, review, flags


# ----------------------------------------------------------------- capture
def yaw_list(yaw_range, shots):
    n = max(1, shots)
    lo, hi = arm_ik.JOINT_LIMITS[0]
    ys = [(-yaw_range + 2 * yaw_range * i / (n - 1)) for i in range(n)] if n > 1 else [0.0]
    return [y for y in ys if lo - 1e-6 <= y <= hi + 1e-6]


def capture_batch(port, sensor_id, yaw_range, shots, settle, roll, grip):
    """팔 HOME 고정, base yaw 스윕하며 N장 캡처. (base_id, [(idx,path)...]) 반환."""
    import serial
    os.makedirs(autolabel.IMG_DIR, exist_ok=True)
    ys = yaw_list(yaw_range, shots)
    print(f"[capture] 팔 HOME, base yaw {ys[0]:+.0f}..{ys[-1]:+.0f}deg, {len(ys)}장", flush=True)
    ser = serial.Serial(port, 115200, timeout=0.1); time.sleep(2.3)
    def drain(s=0.3):
        e = time.time() + s
        while time.time() < e: ser.read(256)
    def send(c):
        ser.write(("<ARM," + ",".join(str(int(round(v))) for v in c) + ">\n").encode()); drain(0.3)
    drain(0.5); send(list(arm_ik.HOME_CMD)); time.sleep(1.0)

    from csi_capture import CsiCamera
    cam = CsiCamera(sensor_id)
    for _ in range(8): cam.read(1.0)
    base = int(time.time()); paths = []
    try:
        for i, y in enumerate(ys):
            send([arm_ik.servo_cmd(0, y), arm_ik.HOME_CMD[1], arm_ik.HOME_CMD[2],
                  arm_ik.HOME_CMD[3], roll, grip])
            time.sleep(settle)
            f = None
            for _ in range(3):
                g = cam.read(1.0)
                if g is not None: f = g
            if f is None: print(f"  {i}: 캡처실패"); continue
            p = f"{autolabel.IMG_DIR}/baseyaw_{base}_{i:02d}.jpg"; cv2.imwrite(p, f); paths.append((i, p))
        send(list(arm_ik.HOME_CMD)); time.sleep(0.5)
    finally:
        cam.release(); ser.close()
    return base, paths


# ----------------------------------------------------------------- label + montage
def make_montage(items, mont_path):
    tw = 500; tiles = []
    for idx, im in items:
        h, w = im.shape[:2]; t = cv2.resize(im, (tw, int(h * tw / w)))
        cv2.rectangle(t, (0, 0), (56, 42), (0, 0, 0), -1)
        cv2.putText(t, str(idx), (6, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
        tiles.append(t)
    if not tiles: return
    cols = 4; rows = (len(tiles) + cols - 1) // cols; th = tiles[0].shape[0]
    grid = np.full((rows * th, cols * tw, 3), 40, np.uint8)
    for i, t in enumerate(tiles):
        r, c = divmod(i, cols); grid[r*th:r*th+t.shape[0], c*tw:c*tw+tw] = t
    os.makedirs(os.path.dirname(mont_path), exist_ok=True)
    cv2.imwrite(mont_path, grid)


def label_and_montage(paths, fc, model=None, conf=0.3):
    if model is None:
        autolabel.load_models()          # FastSAM+SigLIP 백엔드만 로드
    items = []; tot = 0
    for idx, p in paths:
        out = yolo_label(p, model, conf) if model else autolabel.label_image(p, force_class=fc)
        if out is None: continue
        lines, review, _ = out
        name = os.path.splitext(os.path.basename(p))[0]
        open(f"{autolabel.LBL_DIR}/{name}.txt", "w").write("\n".join(lines) + ("\n" if lines else ""))
        cv2.imwrite(f"{autolabel.REV_DIR}/{name}.png", review)
        items.append((idx, review)); tot += len(lines)
        print(f"  {idx}: 박스 {len(lines)}개")
    autolabel.write_data_yaml()
    make_montage(items, MONT)
    return tot


def prune_batch(batch_id, idxs):
    for idx in idxs:
        stem = f"baseyaw_{batch_id}_{idx:02d}"
        for d, ext in ((autolabel.IMG_DIR, ".jpg"), (autolabel.LBL_DIR, ".txt"), (autolabel.REV_DIR, ".png")):
            fp = f"{d}/{stem}{ext}"
            if os.path.exists(fp): os.remove(fp)
        print(f"  제외 {stem}")
    remain = len(glob.glob(f"{autolabel.IMG_DIR}/baseyaw_{batch_id}_*.jpg"))
    print(f"[prune] {len(idxs)}개 삭제, 이 배치 잔여 {remain}장")


def count_images():
    return len(glob.glob(f"{autolabel.IMG_DIR}/*.jpg") + glob.glob(f"{autolabel.IMG_DIR}/*.png"))


def parse_nums(s):
    return [int(x) for x in s.replace(" ", "").split(",") if x.strip() != ""]


# ----------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--class", dest="cls_name", default="cube")
    ap.add_argument("--loop", action="store_true", help="대화형 반복 수집")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--sensor-id", type=int, default=camera_config.BODY)
    ap.add_argument("--yaw-range", type=float, default=35.0)
    ap.add_argument("--shots", type=int, default=8)
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--roll", type=int, default=90)
    ap.add_argument("--grip", type=int, default=90)
    ap.add_argument("--prune", default=None, help="배치 id")
    ap.add_argument("--remove", default="", help="제외할 프레임 번호 (예: 0,7)")
    ap.add_argument("--yolo", action="store_true", help="학습한 YOLO(best.pt)로 라벨 — 박스+클래스 동시(혼합장면 OK)")
    ap.add_argument("--model", default=None, help="YOLO 가중치 경로 (지정시 YOLO 라벨)")
    ap.add_argument("--conf", type=float, default=0.3, help="YOLO 라벨 신뢰도 임계")
    ap.add_argument("--no-label", action="store_true", help="라벨링 없이 촬영만 (모델 안 씀, 원본 몽타주만)")
    args = ap.parse_args()

    if args.cls_name not in autolabel.SHAPES:
        print(f"--class 는 {autolabel.SHAPES} 중 하나여야 함"); return 1
    autolabel.ensure_dirs()
    model = None if args.no_label else (args.model or (find_latest_best() if args.yolo else None))
    if (args.yolo or args.model) and not args.no_label and not model:
        print("YOLO 가중치 못 찾음 — 학습된 best.pt 필요"); return 1
    print("[mode] 촬영만 (라벨링 나중에)" if args.no_label else
          f"[labeler] {'YOLO ' + model + f' (conf={args.conf})' if model else 'FastSAM+SigLIP (--class 고정)'}")

    def do_batch(fc):
        base, paths = capture_batch(args.port, args.sensor_id, args.yaw_range, args.shots,
                                    args.settle, args.roll, args.grip)
        if not paths:
            print("캡처 0장"); return None
        if args.no_label:                       # 촬영만 — 원본 몽타주만(박스 없음)
            make_montage([(idx, cv2.imread(p)) for idx, p in paths], MONT)
            print(f"[batch] id={base} | {len(paths)}장 (촬영만) ▶ 원본 몽타주: {MONT}")
            return base
        tot = label_and_montage(paths, fc, model, args.conf)
        lab = "YOLO(multi-class)" if model else autolabel.SHAPES[fc]
        print(f"[batch] id={base} | {len(paths)}장 | 박스 {tot}개 | {lab}")
        print(f"  ▶ 몽타주 확인: {MONT}")
        return base

    # ---- prune 단독 ----
    if args.prune:
        prune_batch(args.prune, parse_nums(args.remove)); return 0

    # ---- 1배치 (비대화) ----
    if not args.loop:
        base = do_batch(autolabel.SHAPES.index(args.cls_name))
        if base is not None:
            print(f"  잘못된 프레임 제외: python3 scripts/batch_capture.py --prune {base} --remove 0,7")
        return 0

    # ---- 대화형 루프 ----
    cur = args.cls_name
    if args.no_label:
        print("=== 촬영 전용 루프 (라벨링 안함) ===")
        print("  물체 배치 후 Enter=촬영 / q=종료  (~200장 모으면 q → 나중에 한번에 라벨)")
    elif model:
        print("=== 데이터 수집 루프 (YOLO 라벨, 혼합장면 OK) ===")
        print("  물체 배치 후 Enter=촬영 / q=종료")
    else:
        print("=== 데이터 수집 루프 ===")
        print(f"  클래스: {autolabel.SHAPES}  (현재 '{cur}')")
        print("  물체 배치 후 Enter=촬영 / 클래스이름=전환 후 촬영 / q=종료")
    n = 0
    while True:
        if args.no_label:
            prompt = f"\n[촬영] (누적 {count_images()}장) Enter=촬영 / q=종료 > "
        elif model:
            prompt = "\n[YOLO] 물체 배치 후 Enter (종료 q) > "
        else:
            prompt = f"\n[{cur}] 물체 배치 후 Enter (전환: 이름 / 종료: q) > "
        s = input(prompt).strip()
        if s.lower() == "q":
            break
        if not model and not args.no_label and s:
            if s in autolabel.SHAPES:
                cur = s
            else:
                print(f"  '{s}' 모르는 클래스 — '{cur}' 유지")
        fc = autolabel.SHAPES.index(cur)
        base = do_batch(fc)
        if base is None:
            continue
        rm = input("  거를 프레임 번호 (쉼표, 없으면 Enter) > ").strip()
        if rm:
            prune_batch(base, parse_nums(rm))
        n += 1
        print(f"  ✔ 배치 {n} 완료. 누적 이미지 {count_images()}장 (class별 섞임)")
    print(f"\n종료. 총 {count_images()}장.")
    print("학습: yolo detect train data=data/dataset/data.yaml model=yolov8n.pt epochs=80 imgsz=640")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
