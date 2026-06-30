#!/usr/bin/env python3
"""디렉토리의 이미지를 학습 YOLO로 일괄 pre-label (박스+클래스, dedupe 적용). 카메라 불필요.

기본은 '라벨 없는 이미지'만 (새로 촬영분). --relabel 주면 전부 다시.
이후 labelImg로 틀린 것만 교정.

  python3 scripts/yolo_label_dir.py --model models/cube.pt --conf 0.3
  python3 scripts/yolo_label_dir.py --model models/cube.pt --relabel    # 기존 라벨도 다시
"""
from __future__ import annotations
import argparse, glob, os, sys
import cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autolabel        # noqa: E402  (IMG_DIR/LBL_DIR/REV_DIR/write_data_yaml/ensure_dirs)
import batch_capture    # noqa: E402  (yolo_label: 모델 로드+예측+dedupe)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="models/cube.pt")
    ap.add_argument("--conf", type=float, default=0.3)
    ap.add_argument("--relabel", action="store_true", help="이미 라벨된 것도 다시")
    args = ap.parse_args()
    if not os.path.exists(args.model):
        print(f"모델 없음: {args.model}"); return 1
    autolabel.ensure_dirs()

    imgs = sorted(glob.glob(f"{autolabel.IMG_DIR}/*.jpg") + glob.glob(f"{autolabel.IMG_DIR}/*.png"))
    print(f"[yolo-label] 대상 이미지 {len(imgs)}장, model={args.model}, conf={args.conf}", flush=True)
    done = skip = tot = flagged = 0
    for p in imgs:
        name = os.path.splitext(os.path.basename(p))[0]
        lbl = f"{autolabel.LBL_DIR}/{name}.txt"
        if os.path.exists(lbl) and not args.relabel:
            skip += 1; continue
        out = batch_capture.yolo_label(p, args.model, args.conf)   # (lines, review, flags)
        if out is None:
            continue
        lines, review, flags = out
        open(lbl, "w").write("\n".join(lines) + ("\n" if lines else ""))
        cv2.imwrite(f"{autolabel.REV_DIR}/{name}.png", review)
        done += 1; tot += len(lines)
        if flags: flagged += 1
        if done % 20 == 0:
            print(f"  ...{done}장 라벨", flush=True)
    autolabel.write_data_yaml()
    print(f"[yolo-label] 완료: {done}장 라벨({tot}박스), 건너뜀(이미라벨) {skip}장, 약한확신 포함 {flagged}장")
    print(f"  교정: labelImg / 리뷰이미지 {autolabel.REV_DIR}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
