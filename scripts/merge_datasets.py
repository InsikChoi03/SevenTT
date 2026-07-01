#!/usr/bin/env python3
"""여러 YOLO 데이터셋(images/+labels/)을 하나로 병합 + data.yaml/classes.txt 재작성.

기존 4클래스 + 신규 5클래스(fruit_photo_cube 추가) 병합용. --classes로 클래스 순서를 고정해
id 정합을 보장(기존 0-3 그대로, 4=fruit_photo_cube). 라벨 없는 이미지는 빈 .txt=배경 음성으로 유지.
이미지 기준으로 매칭 라벨만 복사(orphan 라벨 무시). 파일명 충돌 감지.

  python3 scripts/merge_datasets.py --out /tmp/pack/dataset \
    --classes cube,octahedron,dodecahedron,icosahedron,fruit_photo_cube \
    --src data/dataset_v2 --src /path/to/body_roam
"""
from __future__ import annotations

import argparse
import glob
import os
import shutil
from collections import Counter


def find_dir(root, name):
    if os.path.isdir(os.path.join(root, name)):
        return os.path.join(root, name)
    hits = [d for d in glob.glob(f"{root}/**/{name}", recursive=True) if os.path.isdir(d)]
    return hits[0] if hits else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--src", action="append", required=True, help="images/+labels/ 있는 폴더 (여러 번)")
    ap.add_argument("--classes", required=True)
    args = ap.parse_args()
    classes = [c.strip() for c in args.classes.split(",") if c.strip()]

    out_img = os.path.join(args.out, "images")
    out_lbl = os.path.join(args.out, "labels")
    os.makedirs(out_img, exist_ok=True)
    os.makedirs(out_lbl, exist_ok=True)

    seen = set()
    collisions = []
    cls_count = Counter()
    n_img = n_lbl = n_empty = 0
    for src in args.src:
        idir = find_dir(src, "images")
        ldir = find_dir(src, "labels")
        if not idir or not ldir:
            print(f"[skip] images/labels 못 찾음: {src}")
            continue
        si = sl = 0
        for p in sorted(glob.glob(f"{idir}/*")):
            if not p.lower().endswith((".jpg", ".jpeg", ".png")):
                continue
            base = os.path.basename(p)
            if base in seen:
                collisions.append(base)
                continue
            seen.add(base)
            shutil.copy(p, os.path.join(out_img, base)); si += 1; n_img += 1
            stem = os.path.splitext(base)[0]
            lp = os.path.join(ldir, stem + ".txt")
            dst = os.path.join(out_lbl, stem + ".txt")
            if os.path.exists(lp):
                shutil.copy(lp, dst); sl += 1; n_lbl += 1
                lines = [ln for ln in open(lp).read().splitlines() if ln.strip()]
                if not lines:
                    n_empty += 1
                for ln in lines:
                    cls_count[int(ln.split()[0])] += 1
            else:
                open(dst, "w").close(); n_empty += 1     # 빈 라벨 = 배경 음성
        print(f"   {src}: img {si}, lbl {sl}")

    with open(os.path.join(args.out, "data.yaml"), "w") as f:
        f.write(f"path: {os.path.abspath(args.out)}\ntrain: images\nval: images\n")
        f.write(f"nc: {len(classes)}\nnames: {classes}\n")
    with open(os.path.join(args.out, "classes.txt"), "w") as f:
        f.write("\n".join(classes) + "\n")

    print(f"[done] {args.out}: images={n_img} labels={n_lbl} (빈라벨 {n_empty})")
    dist = {(classes[k] if k < len(classes) else f"id{k}"): v for k, v in sorted(cls_count.items())}
    print(f"   클래스 분포: {dist}")
    bad = [k for k in cls_count if k >= len(classes)]
    if bad:
        print(f"   ⚠ nc={len(classes)} 범위밖 클래스 {bad}")
    if collisions:
        print(f"   ⚠ 파일명 충돌 {len(collisions)}개 (첫5 {collisions[:5]}) — 건너뜀")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
