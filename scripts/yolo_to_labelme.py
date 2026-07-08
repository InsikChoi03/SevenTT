#!/usr/bin/env python3
"""YOLO 검출 라벨(.txt) → X-AnyLabeling / labelme JSON(.json) 변환. (labelme_to_yolo.py 의 역방향)

pre-label 로 나온 YOLO txt(정규화 cx,cy,w,h)를 X-AnyLabeling 이 '열자마자 박스가 보이는'
labelme 포맷 .json 으로 바꾼다(각 이미지 옆에 <img>.json 생성). classes.txt 의 인덱스→클래스명 사용.
imageData 는 null(이미지는 imagePath 로 로드) → 용량 작음. 빈 .txt(검출0) 도 shapes:[] json 생성해
X-AnyLabeling 에서 그대로 열려 손라벨 가능.

  python3 scripts/yolo_to_labelme.py --dir data/roam_capture/s0704pm2/body_labeled
  # 이후 X-AnyLabeling 에서 교정·저장 → labelme_to_yolo.py 로 다시 YOLO 로 역변환
"""
from __future__ import annotations

import argparse
import glob
import json
import os

from PIL import Image

XANY_VERSION = "2.4.4"   # cosmetic; X-AnyLabeling reads shapes regardless


def load_classes(dir_path: str) -> list[str]:
    p = os.path.join(dir_path, "classes.txt")
    if not os.path.exists(p):
        raise SystemExit(f"classes.txt 없음: {p}")
    with open(p) as f:
        return [ln.strip() for ln in f if ln.strip()]


def yolo_to_shapes(txt_path: str, names: list[str], w: int, h: int) -> tuple[list, list]:
    """한 YOLO .txt → (shapes, warnings). 정규화 cx,cy,w,h → 픽셀 대각 2점 rectangle."""
    shapes, warns = [], []
    if not os.path.exists(txt_path):
        return shapes, warns   # 라벨 없음 = 빈 이미지(shapes:[])
    with open(txt_path) as f:
        for ln, line in enumerate(f, 1):
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                cid = int(float(parts[0]))
                cx, cy, bw, bh = (float(v) for v in parts[1:5])
            except ValueError:
                warns.append(f"{os.path.basename(txt_path)}:{ln} 파싱 실패")
                continue
            if not (0 <= cid < len(names)):
                warns.append(f"{os.path.basename(txt_path)}:{ln} 클래스 id {cid} 범위밖")
                continue
            x0 = max(0.0, (cx - bw / 2.0) * w)
            y0 = max(0.0, (cy - bh / 2.0) * h)
            x1 = min(float(w), (cx + bw / 2.0) * w)
            y1 = min(float(h), (cy + bh / 2.0) * h)
            shapes.append({
                "label": names[cid],
                "points": [[round(x0, 2), round(y0, 2)], [round(x1, 2), round(y1, 2)]],
                "group_id": None,
                "description": "",
                "difficult": False,
                "shape_type": "rectangle",
                "flags": {},
                "attributes": {},
            })
    return shapes, warns


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="이미지+*.txt+classes.txt 있는 폴더(_labeled)")
    ap.add_argument("--overwrite", action="store_true", help="기존 .json 도 덮어씀")
    args = ap.parse_args()

    names = load_classes(args.dir)
    imgs = sorted(glob.glob(f"{args.dir}/*.png") + glob.glob(f"{args.dir}/*.jpg")
                  + glob.glob(f"{args.dir}/*.jpeg"))
    print(f"[yolo->json] {len(imgs)}장, classes={names}", flush=True)

    made = skip = empty = boxes = 0
    all_warns: list[str] = []
    for img_path in imgs:
        base, _ = os.path.splitext(img_path)
        json_path = base + ".json"
        if os.path.exists(json_path) and not args.overwrite:
            skip += 1
            continue
        with Image.open(img_path) as im:
            w, h = im.size
        shapes, warns = yolo_to_shapes(base + ".txt", names, w, h)
        all_warns += warns
        boxes += len(shapes)
        if not shapes:
            empty += 1
        doc = {
            "version": XANY_VERSION,
            "flags": {},
            "shapes": shapes,
            "imagePath": os.path.basename(img_path),
            "imageData": None,
            "imageHeight": h,
            "imageWidth": w,
        }
        with open(json_path, "w") as f:
            json.dump(doc, f, ensure_ascii=False, indent=2)
        made += 1

    print(f"[done] json 생성 {made} (기존 skip {skip}), 총 {boxes} 박스, 빈이미지 {empty}", flush=True)
    if all_warns:
        print(f"  경고 {len(all_warns)}건 (앞 10):", flush=True)
        for w in all_warns[:10]:
            print("   -", w, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
