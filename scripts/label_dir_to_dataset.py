#!/usr/bin/env python3
"""임의 이미지 폴더를 학습된 YOLO로 일괄 pre-label → labelImg 바로 열리는 자립 데이터셋 폴더.

yolo_label_dir.py는 data/dataset/images 고정이라, roam 수집분 같은 임의 폴더를 별도 데이터셋으로
라벨링하려고 만듦. 출력 = 이미지 + .txt + classes.txt + data.yaml 평면 배치(labelImg YOLO 모드 그대로)
+ review/ QC 이미지. classes.txt 끝에 추가 클래스(기본 fruit_photo_cube)를 넣어 labelImg에서 바로
사진패치 손라벨 가능(모델은 기본 도형만 자동, 패치는 사람이 추가).

  python3 scripts/label_dir_to_dataset.py --images data/roam_capture/body \
      --model models/cube.pt --out data/roam_capture/body_labeled
  # 나중에 wide 모델 생기면:
  python3 scripts/label_dir_to_dataset.py --images data/roam_capture/wide \
      --model models/wide.pt --out data/roam_capture/wide_labeled
"""
from __future__ import annotations

import argparse
import glob
import os
import shutil
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import batch_capture  # noqa: E402  (yolo_label: 모델 1회 로드 캐시 + dedupe)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True, help="라벨링할 이미지 폴더")
    ap.add_argument("--model", default="models/cube.pt")
    ap.add_argument("--conf", type=float, default=0.3)
    ap.add_argument("--imgsz", type=int, default=640,
                    help="추론 해상도 — 학습 해상도와 맞출 것(광각 wide.pt=1280, 본체 cube.pt=640)")
    ap.add_argument("--out", required=True, help="출력 데이터셋 폴더(labelImg로 열 곳)")
    ap.add_argument("--extra-classes", default="fruit_photo_cube",
                    help="모델 클래스 뒤에 추가(쉼표). labelImg 손라벨용. 끄려면 빈 문자열")
    ap.add_argument("--no-review", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(args.model):
        print(f"모델 없음: {args.model}"); return 1
    imgs = sorted(glob.glob(f"{args.images}/*.jpg") + glob.glob(f"{args.images}/*.png"))
    if not imgs:
        print(f"이미지 없음: {args.images}"); return 1
    os.makedirs(args.out, exist_ok=True)
    rev_dir = os.path.join(args.out, "review")
    if not args.no_review:
        os.makedirs(rev_dir, exist_ok=True)

    # 첫 호출로 모델 로드(_yolo) + 클래스 이름 확보.
    print(f"[init] 모델 로드 {args.model} (imgsz={args.imgsz}) ...", flush=True)
    batch_capture.yolo_label(imgs[0], args.model, args.conf, args.imgsz)
    names = batch_capture._yolo.names                 # {id: name}
    base_classes = [names[i] for i in range(len(names))]
    extra = [c.strip() for c in args.extra_classes.split(",") if c.strip()]
    classes = base_classes + [c for c in extra if c not in base_classes]
    print(f"[init] 모델 클래스={base_classes}  + 추가={extra}", flush=True)

    tot = weak_imgs = empty = 0
    per_cls = {c: 0 for c in classes}
    for i, p in enumerate(imgs):
        name = os.path.splitext(os.path.basename(p))[0]
        out = batch_capture.yolo_label(p, args.model, args.conf, args.imgsz)
        if out is None:
            print(f"  읽기실패 skip: {p}"); continue
        lines, review, flags = out
        shutil.copy(p, os.path.join(args.out, os.path.basename(p)))
        with open(os.path.join(args.out, f"{name}.txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        if not args.no_review:
            cv2.imwrite(os.path.join(rev_dir, f"{name}.png"), review)
        tot += len(lines)
        if not lines:
            empty += 1
        for ln in lines:
            cid = int(ln.split()[0])
            if 0 <= cid < len(classes):
                per_cls[classes[cid]] += 1
        if flags:
            weak_imgs += 1
        if (i + 1) % 20 == 0:
            print(f"  ...{i+1}/{len(imgs)}", flush=True)

    with open(os.path.join(args.out, "classes.txt"), "w") as f:
        f.write("\n".join(classes) + "\n")
    with open(os.path.join(args.out, "data.yaml"), "w") as f:
        f.write(f"path: {os.path.abspath(args.out)}\ntrain: .\nval: .\n")
        f.write(f"nc: {len(classes)}\nnames: {classes}\n")

    print(f"\n[done] {len(imgs)}장 라벨 -> {args.out}")
    print(f"  총 {tot}박스, 빈(검출0) {empty}장, 약한확신 포함 {weak_imgs}장")
    print(f"  클래스별: " + ", ".join(f"{c}:{per_cls[c]}" for c in classes))
    print(f"  classes.txt={classes}  (마지막 {extra}=labelImg 손라벨용)")
    if not args.no_review:
        print(f"  QC 리뷰: {rev_dir}/*.png  (labelImg는 review/ 무시)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
