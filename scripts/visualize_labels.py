#!/usr/bin/env python3
"""데이터셋 이미지에 '현재 라벨'을 그려 몽타주 페이지로 저장 (검증용).

  python3 scripts/visualize_labels.py                 # 전체
  python3 scripts/visualize_labels.py --new 152       # 최신 N장만 (새로 라벨한 것)
  python3 scripts/visualize_labels.py --cls cube      # 특정 클래스 든 것만
출력: data/cam_test/verify_page_00.png ...
"""
from __future__ import annotations
import argparse, glob, os, sys
import cv2, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autolabel  # noqa: E402  (SHAPES)

IMGD, LBLD = "data/dataset/images", "data/dataset/labels"
OUTD = "data/cam_test"
COL = [(0, 255, 0), (0, 200, 255), (255, 150, 0), (255, 0, 200)]


def read_label(stem):
    p = f"{LBLD}/{stem}.txt"
    if not os.path.exists(p):
        return None  # 라벨파일 없음
    rows = []
    for r in open(p).read().splitlines():
        s = r.split()
        if len(s) >= 5:
            rows.append((int(s[0]), *map(float, s[1:5])))
    return rows


def draw(im, rows):
    H, W = im.shape[:2]
    for c, cx, cy, w, h in rows:
        x1, y1 = int((cx - w/2) * W), int((cy - h/2) * H)
        x2, y2 = int((cx + w/2) * W), int((cy + h/2) * H)
        col = COL[c % len(COL)]
        cv2.rectangle(im, (x1, y1), (x2, y2), col, 4)
        cv2.putText(im, autolabel.SHAPES[c], (x1, max(34, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 7)
        cv2.putText(im, autolabel.SHAPES[c], (x1, max(34, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, col, 2)
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--new", type=int, default=0, help="최신 N장만")
    ap.add_argument("--cls", default=None, help="이 클래스 든 이미지만")
    ap.add_argument("--per", type=int, default=20, help="페이지당 타일")
    ap.add_argument("--cols", type=int, default=5)
    ap.add_argument("--tw", type=int, default=380)
    args = ap.parse_args()

    imgs = sorted(glob.glob(f"{IMGD}/*.jpg") + glob.glob(f"{IMGD}/*.png"))
    if args.new:
        imgs = sorted(imgs, key=os.path.getmtime, reverse=True)[:args.new]
        imgs = sorted(imgs)
    cls_id = autolabel.SHAPES.index(args.cls) if args.cls in autolabel.SHAPES else None

    tiles = []
    for p in imgs:
        stem = os.path.splitext(os.path.basename(p))[0]
        rows = read_label(stem) or []
        if cls_id is not None and not any(r[0] == cls_id for r in rows):
            continue
        im = cv2.imread(p)
        if im is None:
            continue
        draw(im, rows)
        H, W = im.shape[:2]; t = cv2.resize(im, (args.tw, int(H * args.tw / W)))
        # 빈 라벨 표시(테두리 빨강) + 짧은 id
        if not rows:
            cv2.rectangle(t, (0, 0), (t.shape[1]-1, t.shape[0]-1), (0, 0, 255), 4)
        sid = stem.replace("baseyaw_", "")[-6:]
        cv2.rectangle(t, (0, 0), (132, 26), (0, 0, 0), -1)
        cv2.putText(t, sid + ("·empty" if not rows else ""), (3, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        tiles.append(t)

    if not tiles:
        print("대상 없음"); return 0
    th = tiles[0].shape[0]; cols = args.cols
    pages = (len(tiles) + args.per - 1) // args.per
    os.makedirs(OUTD, exist_ok=True)
    for pg in range(pages):
        chunk = tiles[pg*args.per:(pg+1)*args.per]
        rows_n = (len(chunk) + cols - 1) // cols
        grid = np.full((rows_n*th, cols*args.tw, 3), 40, np.uint8)
        for i, t in enumerate(chunk):
            r, c = divmod(i, cols)
            grid[r*th:r*th+t.shape[0], c*args.tw:c*args.tw+args.tw] = t
        cv2.imwrite(f"{OUTD}/verify_page_{pg:02d}.png", grid)
    print(f"{len(tiles)}장 → {pages}페이지: {OUTD}/verify_page_00.png ~ _{pages-1:02d}.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
