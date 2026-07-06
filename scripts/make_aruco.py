#!/usr/bin/env python3
"""인쇄용 ArUco 마커 PDF 생성 — 정확한 실제 크기(mm)로 뽑히게.

용도:
  · 온-로봇 기준 마커(광각 흔들림 보정): 로봇 윗면, 광각 FOV 안. ~60~80mm 권장.
  · 바닥 캘리브 타겟: 바닥에 알려진 위치로. ~50~80mm.
  · 그리퍼 마커(픽 폐루프): 작게 ~20~30mm.

각 마커 = 한 페이지. 흰 여백(quiet zone) + 컷선 + 라벨(ID/크기/딕셔너리) + **100mm 자**(인쇄
스케일 확인용). **반드시 프린터에서 "실제 크기/100%/배율 없음"으로 인쇄**하고 자로 100mm 확인.

  python3 scripts/make_aruco.py                                   # id 0~5, 60mm, 4x4_50
  python3 scripts/make_aruco.py --ids 0 1 2 --size-mm 70 --out data/aruco_robot.pdf
  python3 scripts/make_aruco.py --ids 10 --size-mm 25 --out data/aruco_gripper.pdf
"""
from __future__ import annotations
import argparse, os
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

_DICTS = {
    "4x4_50": cv2.aruco.DICT_4X4_50, "4x4_100": cv2.aruco.DICT_4X4_100,
    "5x5_50": cv2.aruco.DICT_5X5_50, "5x5_100": cv2.aruco.DICT_5X5_100,
    "6x6_50": cv2.aruco.DICT_6X6_50, "apriltag_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}
PAGES = {"a4": (210.0, 297.0), "letter": (215.9, 279.4)}   # mm


def _font(px):
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, px)
    return ImageFont.load_default()


def mm2px(mm, dpi):
    return int(round(mm / 25.4 * dpi))


def make_page(marker_id, size_mm, dict_name, dpi, page_mm):
    adict = cv2.aruco.getPredefinedDictionary(_DICTS[dict_name])
    m_px = mm2px(size_mm, dpi)
    marker = cv2.aruco.generateImageMarker(adict, marker_id, m_px, borderBits=1)  # black cells

    W, H = mm2px(page_mm[0], dpi), mm2px(page_mm[1], dpi)
    page = Image.new("L", (W, H), 255)
    draw = ImageDraw.Draw(page)

    quiet = mm2px(max(8, size_mm * 0.15), dpi)   # 흰 quiet zone
    mx = (W - m_px) // 2
    my = mm2px(35, dpi)
    page.paste(Image.fromarray(marker), (mx, my))
    # 컷선(quiet zone 바깥)
    draw.rectangle([mx - quiet, my - quiet, mx + m_px + quiet, my + m_px + quiet],
                   outline=0, width=max(1, mm2px(0.3, dpi)))

    f_big, f_sm = _font(mm2px(6, dpi)), _font(mm2px(4, dpi))
    draw.text((mx - quiet, my + m_px + quiet + mm2px(4, dpi)),
              f"ArUco  ID={marker_id}   {size_mm:.0f} mm   {dict_name}", font=f_big, fill=0)
    draw.text((mx - quiet, my + m_px + quiet + mm2px(12, dpi)),
              "PRINT AT 100% (no scaling). Then check the ruler below reads exactly 100 mm.",
              font=f_sm, fill=0)

    # 100mm 스케일 자 (10mm 눈금)
    ry = H - mm2px(30, dpi)
    rx0 = mm2px(20, dpi)
    draw.line([rx0, ry, rx0 + mm2px(100, dpi), ry], fill=0, width=max(1, mm2px(0.4, dpi)))
    for i in range(11):
        x = rx0 + mm2px(10 * i, dpi)
        draw.line([x, ry - mm2px(3, dpi), x, ry + mm2px(3, dpi)], fill=0, width=max(1, mm2px(0.3, dpi)))
    draw.text((rx0, ry + mm2px(5, dpi)), "0 |-------| 100 mm  (10 mm ticks)", font=f_sm, fill=0)
    return page.convert("RGB")


def make_grid_page(ids, size_mm, dict_name, dpi, page_mm):
    """한 페이지에 여러 마커를 그리드로(바닥 캘리브용, 종이 절약). 각자 quiet zone+컷선+ID."""
    adict = cv2.aruco.getPredefinedDictionary(_DICTS[dict_name])
    m_px = mm2px(size_mm, dpi)
    quiet = mm2px(max(6, size_mm * 0.12), dpi)
    cell = m_px + 2 * quiet + mm2px(8, dpi)
    W, H = mm2px(page_mm[0], dpi), mm2px(page_mm[1], dpi)
    margin = mm2px(12, dpi)
    cols = max(1, (W - 2 * margin) // cell)
    rows = max(1, (H - 2 * margin - mm2px(22, dpi)) // cell)
    cap = cols * rows
    page = Image.new("L", (W, H), 255)
    draw = ImageDraw.Draw(page)
    f_sm = _font(mm2px(3.5, dpi))
    for i, mid in enumerate(ids[:cap]):
        r, c = divmod(i, cols)
        marker = cv2.aruco.generateImageMarker(adict, mid, m_px, borderBits=1)
        x = margin + c * cell + quiet
        y = margin + r * cell + quiet
        page.paste(Image.fromarray(marker), (x, y))
        draw.rectangle([x - quiet, y - quiet, x + m_px + quiet, y + m_px + quiet],
                       outline=0, width=max(1, mm2px(0.25, dpi)))
        draw.text((x, y + m_px + mm2px(1, dpi)), f"ID={mid}  {size_mm:.0f}mm", font=f_sm, fill=0)
    # 스케일 자
    ry = H - mm2px(14, dpi); rx0 = margin
    draw.line([rx0, ry, rx0 + mm2px(100, dpi), ry], fill=0, width=max(1, mm2px(0.4, dpi)))
    for i in range(11):
        x = rx0 + mm2px(10 * i, dpi)
        draw.line([x, ry - mm2px(3, dpi), x, ry + mm2px(3, dpi)], fill=0, width=max(1, mm2px(0.3, dpi)))
    draw.text((rx0, ry + mm2px(5, dpi)), f"0 |-------| 100mm  ({dict_name}, print 100%)", font=f_sm, fill=0)
    return page.convert("RGB"), cap


def make_mixed_grid(specs, dict_name, dpi, page_mm):
    """크기 섞인 마커들을 그리드로(여러 크기 한 PDF에). specs=[(id,size_mm),...]. 셀=최대크기 기준."""
    adict = cv2.aruco.getPredefinedDictionary(_DICTS[dict_name])
    max_mm = max(s for _, s in specs)
    m_max = mm2px(max_mm, dpi)
    gutter = mm2px(max(6, max_mm * 0.12), dpi)
    cell = m_max + 2 * gutter + mm2px(8, dpi)
    W, H = mm2px(page_mm[0], dpi), mm2px(page_mm[1], dpi)
    margin = mm2px(12, dpi)
    cols = max(1, (W - 2 * margin) // cell)
    rows = max(1, (H - 2 * margin - mm2px(22, dpi)) // cell)
    cap = cols * rows
    f_sm = _font(mm2px(3.5, dpi))
    pages = []
    for start in range(0, len(specs), cap):
        chunk = specs[start:start + cap]
        page = Image.new("L", (W, H), 255)
        draw = ImageDraw.Draw(page)
        for i, (mid, sz) in enumerate(chunk):
            r, c = divmod(i, cols)
            m_px = mm2px(sz, dpi)
            marker = cv2.aruco.generateImageMarker(adict, mid, m_px, borderBits=1)
            x = margin + c * cell + gutter + (m_max - m_px) // 2
            y = margin + r * cell + gutter + (m_max - m_px) // 2
            page.paste(Image.fromarray(marker), (x, y))
            q = mm2px(max(6, sz * 0.12), dpi)
            draw.rectangle([x - q, y - q, x + m_px + q, y + m_px + q],
                           outline=0, width=max(1, mm2px(0.25, dpi)))
            draw.text((margin + c * cell + gutter, margin + r * cell + gutter + m_max + mm2px(1, dpi)),
                      f"ID={mid}  {sz:.0f}mm", font=f_sm, fill=0)
        ry = H - mm2px(14, dpi); rx0 = margin
        draw.line([rx0, ry, rx0 + mm2px(100, dpi), ry], fill=0, width=max(1, mm2px(0.4, dpi)))
        for i in range(11):
            xx = rx0 + mm2px(10 * i, dpi)
            draw.line([xx, ry - mm2px(3, dpi), xx, ry + mm2px(3, dpi)], fill=0, width=max(1, mm2px(0.3, dpi)))
        draw.text((rx0, ry + mm2px(5, dpi)), f"0 |-------| 100mm  ({dict_name}, print 100%)", font=f_sm, fill=0)
        pages.append(page.convert("RGB"))
    return pages


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2, 3, 4, 5])
    ap.add_argument("--size-mm", type=float, default=60.0)
    ap.add_argument("--dict", default="4x4_50", choices=list(_DICTS))
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--page", default="a4", choices=list(PAGES))
    ap.add_argument("--grid", action="store_true", help="한 페이지에 여러 개(바닥 캘리브용 종이 절약)")
    ap.add_argument("--spec", nargs="+", default=None,
                    help="크기 섞어 한 PDF에: 'id:size' 나열 (예: 0:45 1:45 2:45 10:30 11:30)")
    ap.add_argument("--out", default="data/aruco_markers.pdf")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    if args.spec:
        specs = [(int(t.split(":")[0]), float(t.split(":")[1])) for t in args.spec]
        pages = make_mixed_grid(specs, args.dict, args.dpi, PAGES[args.page])
        pages[0].save(args.out, "PDF", resolution=args.dpi, save_all=True, append_images=pages[1:])
        print(f"저장 -> {args.out}   ({len(pages)}장, {args.dict}, mixed sizes, grid)")
        print("spec:", args.spec)
        print("⚠️ '실제 크기/100%'로 인쇄 후 하단 자로 100mm 확인. 가위로 컷선 따라 자르기.")
        return 0
    if args.grid:
        pages, ids = [], list(args.ids)
        while ids:
            pg, cap = make_grid_page(ids, args.size_mm, args.dict, args.dpi, PAGES[args.page])
            pages.append(pg); ids = ids[cap:]
    else:
        pages = [make_page(i, args.size_mm, args.dict, args.dpi, PAGES[args.page]) for i in args.ids]
    pages[0].save(args.out, "PDF", resolution=args.dpi, save_all=True, append_images=pages[1:])
    print(f"저장 -> {args.out}   ({len(pages)}장, {args.dict}, {args.size_mm:.0f}mm, {args.dpi}dpi, "
          f"{args.page}{', grid' if args.grid else ''})")
    print("ID:", args.ids)
    print("⚠️ 프린트 대화상자에서 '실제 크기/100%/맞춤 안함'으로 인쇄 후, 페이지 하단 자로 100mm 확인.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
