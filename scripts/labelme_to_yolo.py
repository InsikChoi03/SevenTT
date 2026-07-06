#!/usr/bin/env python3
"""X-AnyLabeling / labelme JSON(.json) → YOLO 검출 라벨(images/+labels/) 변환.

X-AnyLabeling에서 박스 교정 후 그냥 '저장'하면 labelme 포맷 `.json`으로 나온다(Export→YOLO를
안 하면 기존 .txt는 안 바뀜). 이 스크립트가 그 .json의 rectangle들을 YOLO txt로 변환한다.
- points는 2점(대각) 또는 4점(꼭짓점) 모두 지원 → min/max로 bbox.
- --classes 순서로 클래스 id 고정(기존 모델과 정합). 목록 밖 라벨은 경고+건너뜀.
- shapes 0개인 json = 빈 .txt(배경 음성)로 유지.
- 출력 = <out>/images/ + <out>/labels/ + classes.txt + data.yaml (merge_datasets/labelImg 호환).

  python3 scripts/labelme_to_yolo.py --src /tmp/wide_corrected/wide0704_labeled \
    --out data/roam_capture/wide0704_yolo \
    --classes cube,octahedron,dodecahedron,icosahedron,fruit_photo_cube
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import shutil


def convert_one(jpath, name2id):
    """한 json → (yolo_lines, [경고라벨...]). imageWidth/Height로 정규화."""
    d = json.load(open(jpath))
    W, H = d.get("imageWidth"), d.get("imageHeight")
    if not W or not H:
        return None, [f"{os.path.basename(jpath)}: imageWidth/Height 없음"]
    lines, warns = [], []
    for sh in d.get("shapes", []):
        if sh.get("shape_type") != "rectangle":
            warns.append(f"{os.path.basename(jpath)}: shape_type={sh.get('shape_type')} 건너뜀")
            continue
        lab = sh.get("label")
        if lab not in name2id:
            warns.append(f"{os.path.basename(jpath)}: 알수없는 라벨 '{lab}' 건너뜀")
            continue
        xs = [p[0] for p in sh["points"]]
        ys = [p[1] for p in sh["points"]]
        x0, x1 = min(xs), max(xs)
        y0, y1 = min(ys), max(ys)
        cx = ((x0 + x1) / 2) / W
        cy = ((y0 + y1) / 2) / H
        w = (x1 - x0) / W
        h = (y1 - y0) / H
        # [0,1] 클립(경계 넘는 박스 방어)
        cx, cy = min(max(cx, 0), 1), min(max(cy, 0), 1)
        w, h = min(max(w, 0), 1), min(max(h, 0), 1)
        lines.append(f"{name2id[lab]} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    return lines, warns


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="json(+이미지) 있는 폴더")
    ap.add_argument("--out", required=True, help="출력(images/+labels/ 생성)")
    ap.add_argument("--classes", required=True, help="쉼표구분, id 순서 고정")
    args = ap.parse_args()

    classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    name2id = {c: i for i, c in enumerate(classes)}

    out_img = os.path.join(args.out, "images")
    out_lbl = os.path.join(args.out, "labels")
    os.makedirs(out_img, exist_ok=True)
    os.makedirs(out_lbl, exist_ok=True)

    jsons = sorted(glob.glob(f"{args.src}/*.json"))
    if not jsons:
        print(f"[err] json 없음: {args.src}")
        return 1

    n_img = n_box = n_empty = miss_img = 0
    cls_count = {c: 0 for c in classes}
    all_warns = []
    for jp in jsons:
        stem = os.path.splitext(os.path.basename(jp))[0]
        # 대응 이미지 찾기(png/jpg)
        src_img = None
        for ext in (".png", ".jpg", ".jpeg"):
            cand = os.path.join(args.src, stem + ext)
            if os.path.exists(cand):
                src_img = cand
                break
        if src_img is None:
            miss_img += 1
            all_warns.append(f"{stem}: 대응 이미지 없음 → 건너뜀")
            continue
        lines, warns = convert_one(jp, name2id)
        all_warns += warns
        if lines is None:
            continue
        shutil.copy(src_img, os.path.join(out_img, os.path.basename(src_img)))
        with open(os.path.join(out_lbl, stem + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        n_img += 1
        n_box += len(lines)
        if not lines:
            n_empty += 1
        for ln in lines:
            cls_count[classes[int(ln.split()[0])]] += 1

    with open(os.path.join(args.out, "classes.txt"), "w") as f:
        f.write("\n".join(classes) + "\n")
    with open(os.path.join(args.out, "data.yaml"), "w") as f:
        f.write(f"path: {os.path.abspath(args.out)}\ntrain: images\nval: images\n")
        f.write(f"nc: {len(classes)}\nnames: {classes}\n")

    print(f"[done] {args.out}: 이미지 {n_img}, 박스 {n_box}, 빈라벨 {n_empty}, 이미지없음 {miss_img}")
    print(f"   클래스 분포: " + ", ".join(f"{c}:{cls_count[c]}" for c in classes))
    if all_warns:
        print(f"   ⚠ 경고 {len(all_warns)}건 (첫5): " + " | ".join(all_warns[:5]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
