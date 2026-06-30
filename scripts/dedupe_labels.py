#!/usr/bin/env python3
"""기존 YOLO 라벨(.txt)의 겹친/중첩 박스 제거 — 재촬영·모델 불필요.

정규화 좌표(cx,cy,w,h)로 box_utils.dedupe 적용 (IoU>=0.6 또는 IoMin>=0.7 = 같은 물체).
conf가 없으니 '큰 박스(전체 물체) 우선'으로 유지. 서로 다른 물체 부분겹침은 보존.

  python3 scripts/dedupe_labels.py                    # data/dataset/labels 정리(덮어씀)
  python3 scripts/dedupe_labels.py --dry-run          # 바뀌는 것만 출력, 안 씀
"""
from __future__ import annotations
import argparse, glob, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_utils  # noqa: E402

LBL = "data/dataset/labels"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--labels", default=LBL)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    before = after = changed = 0
    for f in sorted(glob.glob(f"{args.labels}/*.txt")):
        rows = [r.split() for r in open(f).read().splitlines() if r.strip()]
        if len(rows) < 2:
            continue
        dets = []
        for p in rows:
            c = int(p[0]); cx, cy, w, h = map(float, p[1:5])
            dets.append((cx - w/2, cy - h/2, cx + w/2, cy + h/2, c, w * h))  # conf=area → 큰 박스 우선
        kept = box_utils.dedupe(dets)
        before += len(dets); after += len(kept)
        if len(kept) != len(dets):
            changed += 1
            print(f"  {os.path.basename(f)}: {len(dets)} -> {len(kept)} "
                  f"(cls {[d[4] for d in dets]} -> {[d[4] for d in kept]})")
            if not args.dry_run:
                out = []
                for (x1, y1, x2, y2, c, _) in kept:
                    out.append(f"{c} {(x1+x2)/2:.6f} {(y1+y2)/2:.6f} {x2-x1:.6f} {y2-y1:.6f}")
                open(f, "w").write("\n".join(out) + "\n")
    tag = "(dry-run, 안 씀)" if args.dry_run else "정리 완료"
    print(f"\n{tag}: 박스 {before} -> {after}, 바뀐 파일 {changed}개")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
