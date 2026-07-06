#!/usr/bin/env python3
"""집기·놓기 검증 — 본체캠(앞)으로 '집기', 광각캠(뒤 보관함)으로 '놓기'를 각각 판정.

두 카메라, 두 판정:
  ▶ 집기 판정 (본체캠 sensor0 = 앞 픽존, cube.pt):
      집기 전 앞을 캡처해 타깃 도형 식별 → 집기·놓기 실행 → 다시 캡처.
      타깃이 픽존에서 **사라졌으면** 집기 성공(집어서 뒤로 옮김) / 그대로면 실패.
  ▶ 놓기 판정 (광각캠 sensor1 = 뒤 보관함, wide.pt):
      놓은 뒤 광각캠 캡처 → 타깃 도형이 화면 **중심 영역**(--center-frac)에 들어오면 놓기 성공.
      보관함이 광각 중심에 오도록 배치돼 있다는 전제(사용자 지정 방식).

nested(과일큐브): cube 박스 안 fruit_photo_cube 패치는 그 도형을 fruit_photo_cube 로 본다.
주석 이미지: data/pick/verify_before.png, verify_after.png(집기), verify_place.png(놓기).

  python3 scripts/pick_verify.py                 # 집기+놓기 전체 (팔 동작 포함)
  python3 scripts/pick_verify.py --check-only     # 팔 없이 지금 앞(본체)에 뭐 있나만
  python3 scripts/pick_verify.py --wide-check      # 팔 없이 지금 광각 중심에 뭐 있나만(놓기존 튜닝)
  python3 scripts/pick_verify.py --no-arm          # before→[엔터 수동 픽앤플레이스]→집기·놓기 판정
  python3 scripts/pick_verify.py --no-place         # 집기 판정만(광각 안 씀)
  python3 scripts/pick_verify.py --center-frac 0.5  # 놓기 중심영역 넓히기
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from csi_capture import CsiCamera  # noqa: E402
import camera_config  # noqa: E402

KOR = {  # 리포트용 한글 라벨
    "cube": "정육면체",
    "octahedron": "정팔면체",
    "dodecahedron": "정십이면체",
    "icosahedron": "정이십면체",
    "fruit_photo_cube": "과일사진큐브",
}


def detect_shapes(frame, model, conf, imgsz=640):
    """모델 5클래스 전부 검출 → dict 리스트. 각: name/cls/conf/box(x1,y1,x2,y2)/cx/cy/area."""
    res = model.predict(frame, conf=conf, imgsz=imgsz, verbose=False)
    out = []
    if not res:
        return out
    boxes = res[0].boxes
    if boxes is None or len(boxes) == 0:
        return out
    names = model.names
    xyxy = boxes.xyxy.cpu().numpy()
    confs = boxes.conf.cpu().numpy()
    clses = boxes.cls.cpu().numpy().astype(int)
    for i in range(len(clses)):
        x1, y1, x2, y2 = (float(v) for v in xyxy[i])
        out.append({
            "cls": int(clses[i]),
            "name": names[int(clses[i])],
            "conf": float(confs[i]),
            "box": (x1, y1, x2, y2),
            "cx": (x1 + x2) / 2.0,
            "cy": (y1 + y2) / 2.0,
            "area": (x2 - x1) * (y2 - y1),
        })
    return out


def _center_in_box(cx, cy, box):
    x1, y1, x2, y2 = box
    return x1 <= cx <= x2 and y1 <= cy <= y2


def resolve_nested(det, dets):
    """det 가 cube 인데 그 안에 fruit_photo_cube 패치가 있으면 fruit_photo_cube 로 재해석."""
    if det["name"] != "cube":
        return det
    for d in dets:
        if d["name"] == "fruit_photo_cube" and _center_in_box(d["cx"], d["cy"], det["box"]):
            merged = dict(det)
            merged["name"] = "fruit_photo_cube"
            merged["nested_from_cube"] = True
            return merged
    return det


def base_class(target):
    """타깃의 매칭 기준 클래스(nested면 cube 박스 기준이라 cube)."""
    return "cube" if target.get("nested_from_cube") else target["name"]


def pick_target(dets, by="area"):
    """픽존 타깃 1개 선택. by=area(가장 큰=가장 앞) / conf / bottom(가장 아래=가장 가까움).
    fruit_photo_cube 패치는 cube 와 겹치므로 후보 제외(cube 통해 nested 해석)."""
    cand = [d for d in dets if d["name"] != "fruit_photo_cube"]
    if not cand:
        cand = dets
    if not cand:
        return None
    if by == "conf":
        t = max(cand, key=lambda d: d["conf"])
    elif by == "bottom":
        t = max(cand, key=lambda d: d["box"][3])
    else:
        t = max(cand, key=lambda d: d["area"])
    return resolve_nested(t, dets)


def still_present(target, dets, match_dist):
    """after(본체)에서 타깃과 같은 클래스가 픽존 근처(중심거리<match_dist)에 남아있나."""
    base = base_class(target)
    for d in dets:
        if d["name"] != base:
            continue
        dist = ((d["cx"] - target["cx"]) ** 2 + (d["cy"] - target["cy"]) ** 2) ** 0.5
        if dist < match_dist:
            return d, dist
    return None, None


def in_center(det, w, h, frac):
    """검출 중심이 화면 중앙 frac(가로·세로 비율) 박스 안에 있나."""
    hw, hh = w * frac / 2.0, h * frac / 2.0
    return abs(det["cx"] - w / 2.0) <= hw and abs(det["cy"] - h / 2.0) <= hh


def place_hits(target, dets, w, h, frac):
    """놓기존(광각 중심)에 타깃 클래스가 들어온 검출들."""
    base = base_class(target)
    return [d for d in dets if d["name"] == base and in_center(d, w, h, frac)]


def annotate(frame, dets, highlight, path, title, center_frac=None):
    disp = frame.copy()
    h, w = disp.shape[:2]
    if center_frac is not None:   # 놓기 중심영역 박스
        hw, hh = int(w * center_frac / 2), int(h * center_frac / 2)
        cv2.rectangle(disp, (w // 2 - hw, h // 2 - hh), (w // 2 + hw, h // 2 + hh),
                      (255, 200, 0), 2)
        cv2.putText(disp, "center zone", (w // 2 - hw, h // 2 - hh - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
    hl = highlight or []
    for d in dets:
        x1, y1, x2, y2 = (int(v) for v in d["box"])
        is_hl = any(abs(d["cx"] - g["cx"]) < 2 and abs(d["cy"] - g["cy"]) < 2 for g in hl)
        col = (0, 0, 255) if is_hl else (0, 200, 0)
        cv2.rectangle(disp, (x1, y1), (x2, y2), col, 3 if is_hl else 2)
        cv2.putText(disp, f"{d['name']} {d['conf']:.2f}", (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    cv2.putText(disp, title, (12, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 3)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, disp)


def capture(cam, warmup=5):
    for _ in range(warmup):
        cam.read(1.0)
    return cam.read(1.0)


def summarize(dets):
    if not dets:
        return "(검출 없음)"
    return ", ".join(f"{d['name']}({d['conf']:.2f})" for d in
                     sorted(dets, key=lambda d: -d["area"]))


def label_of(target):
    nm = target["name"]
    tag = " (cube 안 과일패치)" if target.get("nested_from_cube") else ""
    return f"{KOR.get(nm, nm)} [{nm}]{tag}"


def run_wide_check(model, conf, center_frac, out_path):
    """광각 중심영역에 지금 뭐가 있나만 리포트(놓기존 튜닝용)."""
    try:
        cam = CsiCamera(camera_config.WIDE)
    except Exception as e:
        print(f"✗ 광각 카메라 열기 실패: {e} (sensor1 rebind/재부팅 필요할 수 있음)")
        return 1
    try:
        frame = capture(cam)
    finally:
        cam.release()
    if frame is None:
        print("✗ 광각 캡처 실패")
        return 1
    dets = detect_shapes(frame, model, conf)
    h, w = frame.shape[:2]
    centered = [d for d in dets if in_center(d, w, h, center_frac)]
    print(f"[wide] 전체 검출: {summarize(dets)}")
    print(f"[wide] 중심영역({center_frac}) 안: {summarize(centered)}")
    annotate(frame, dets, centered, out_path, "WIDE CENTER", center_frac=center_frac)
    print(f"[wide] 이미지 → {out_path}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="models/cube.pt", help="본체(집기) 검출 모델")
    ap.add_argument("--wide-model", default="models/wide.pt", help="광각(놓기) 검출 모델")
    ap.add_argument("--conf", type=float, default=0.4, help="YOLO conf 임계")
    ap.add_argument("--target-by", choices=["area", "conf", "bottom"], default="area",
                    help="픽존 타깃 선택 기준")
    ap.add_argument("--match-dist", type=float, default=180.0,
                    help="집기 판정: after에서 '같은 도형 남아있음' 중심거리 px")
    ap.add_argument("--center-frac", type=float, default=0.4,
                    help="놓기 판정: 광각 중심영역 비율(0~1). 이 안에 들어오면 놓기 성공")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="팔 시리얼 포트")
    ap.add_argument("--check-only", action="store_true", help="팔 없이 본체 앞 검출만")
    ap.add_argument("--wide-check", action="store_true", help="팔 없이 광각 중심 검출만(놓기존 튜닝)")
    ap.add_argument("--no-arm", action="store_true",
                    help="팔 자동동작 없이 before→[엔터 수동]→집기·놓기 판정")
    ap.add_argument("--no-place", action="store_true", help="집기 판정만(광각 놓기 판정 생략)")
    ap.add_argument("--before", default="data/pick/verify_before.png")
    ap.add_argument("--after", default="data/pick/verify_after.png")
    ap.add_argument("--place-img", default="data/pick/verify_place.png")
    args = ap.parse_args()

    from ultralytics import YOLO  # 느린 import는 필요할 때만

    # 광각 단독 튜닝 모드
    if args.wide_check:
        return run_wide_check(YOLO(args.wide_model), args.conf, args.center_frac, args.place_img)

    model = YOLO(args.model)

    cam = CsiCamera(camera_config.BODY)   # 본체 cam = sensor-id 0 (앞 픽존)
    arm = None
    try:
        # ── 팔 연결(INIT 위로 → 앞 시야 확보) ──
        if not args.check_only and not args.no_arm:
            from arm_pick2r import Arm2R
            arm = Arm2R(args.port)
            print("[arm] 연결 → INIT 자세(위로)로 시야 확보")
            arm.open()

        # ── 1) 집기 전 검출(본체) ──
        frame0 = capture(cam)
        if frame0 is None:
            print("✗ 본체 카메라 캡처 실패 (CSI 스트리밍 확인)")
            return 1
        dets0 = detect_shapes(frame0, model, args.conf)
        print(f"[before] 검출: {summarize(dets0)}")
        target = pick_target(dets0, args.target_by)
        annotate(frame0, dets0, [target] if target else None, args.before, "BEFORE")
        print(f"[before] 이미지 → {args.before}")

        if args.check_only:
            print("→ 픽존 타깃: " + (label_of(target) + f" conf={target['conf']:.2f}"
                                    if target else "없음"))
            return 0

        if target is None:
            print("✗ 앞 픽존에 잡을 도형이 없음 — 중단")
            return 1
        print(f"[target] 잡을 도형 = {label_of(target)} conf={target['conf']:.2f} "
              f"@px({target['cx']:.0f},{target['cy']:.0f})")

        # ── 2) 집기 → 놓기 ──
        if args.no_arm:
            try:
                input(">> 지금 수동으로 집기→놓기 수행 후 [엔터]로 판정...")
            except (EOFError, KeyboardInterrupt):
                print("\n중단"); return 1
        else:
            print("[arm] 집기(grip)..."); arm.grip()
            print("[arm] 놓기(release)..."); arm.release()
            print("[arm] INIT 복귀(시야 확보)..."); arm.go_init()
            time.sleep(0.5)

        # ── 3) 집기 판정(본체 재검출) ──
        frame1 = capture(cam)
        grip_ok = False
        if frame1 is None:
            print("✗ after(본체) 캡처 실패")
        else:
            dets1 = detect_shapes(frame1, model, args.conf)
            print(f"[after]  검출: {summarize(dets1)}")
            annotate(frame1, dets1, None, args.after, "AFTER")
            print(f"[after]  이미지 → {args.after}")
            remain, dist = still_present(target, dets1, args.match_dist)
            grip_ok = remain is None
            if grip_ok:
                print(f"  집기 ✅ (픽존에서 사라짐)")
            else:
                print(f"  집기 ❌ (아직 픽존에 있음, 중심거리 {dist:.0f}px)")
    finally:
        cam.release()   # 본체캠 닫고 광각캠으로 (동시 오픈 회피)

    # ── 4) 놓기 판정(광각 중심영역) ──
    place_ok = None
    if not args.no_place:
        wmodel = YOLO(args.wide_model)
        try:
            wcam = CsiCamera(camera_config.WIDE)
        except Exception as e:
            print(f"⚠ 광각 카메라 열기 실패: {e} — 놓기 판정 생략(sensor1 rebind/재부팅?)")
            wcam = None
        if wcam is not None:
            try:
                wframe = capture(wcam)
            finally:
                wcam.release()
            if wframe is None:
                print("⚠ 광각 캡처 실패 — 놓기 판정 생략")
            else:
                wdets = detect_shapes(wframe, wmodel, args.conf)
                h, w = wframe.shape[:2]
                hits = place_hits(target, wdets, w, h, args.center_frac)
                print(f"[place] 광각 검출: {summarize(wdets)}")
                annotate(wframe, wdets, hits, args.place_img, "PLACE", center_frac=args.center_frac)
                print(f"[place] 이미지 → {args.place_img}")
                place_ok = len(hits) > 0
                if place_ok:
                    print(f"  놓기 ✅ ({base_class(target)} 이(가) 광각 중심영역에 들어옴)")
                else:
                    print(f"  놓기 ❌ ({base_class(target)} 이(가) 광각 중심영역에 없음)")

    # ── 최종 리포트 ──
    print("─" * 52)
    print(f"  대상 도형 : {label_of(target)}")
    print(f"  집기 판정 : {'성공 ✅' if grip_ok else '실패 ❌'}")
    if args.no_place:
        print(f"  놓기 판정 : (생략)")
    elif place_ok is None:
        print(f"  놓기 판정 : (광각 불가로 미판정)")
    else:
        print(f"  놓기 판정 : {'성공 ✅' if place_ok else '실패 ❌'}")
    overall = grip_ok and (place_ok if place_ok is not None else True)
    print(f"  종합      : {'✅ 성공' if overall else '❌ 실패/미완'}")
    print("─" * 52)
    if arm is not None:
        arm.close()
    return 0 if overall else 2


if __name__ == "__main__":
    raise SystemExit(main())
