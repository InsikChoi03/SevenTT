#!/usr/bin/env python3
"""Compare body-camera input with the fruit light OFF vs ON.

Default capture settings match the intended fixed-light body camera setup:
fixed exposure 8 ms, gain 1x, wbmode=8, software WB gains enabled.

Examples:
  python3 scripts/lighting_compare.py
  python3 scripts/lighting_compare.py --frames 40 --siglip
  python3 scripts/lighting_compare.py --exposuretimerange "6000000 6000000" --gainrange "1 1"
  python3 scripts/lighting_compare.py --auto-exposure
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402
from csi_capture import CsiCamera  # noqa: E402


DEFAULT_LABELS = "apple,orange,banana,pineapple"


def roi_box(frame: np.ndarray, frac: float) -> tuple[int, int, int, int]:
    h, w = frame.shape[:2]
    side = max(16, int(min(h, w) * frac))
    cx, cy = w // 2, h // 2
    x1 = max(0, cx - side // 2)
    y1 = max(0, cy - side // 2)
    x2 = min(w, x1 + side)
    y2 = min(h, y1 + side)
    return x1, y1, x2, y2


def crop_roi(frame: np.ndarray, frac: float) -> tuple[np.ndarray, tuple[int, int, int, int]]:
    box = roi_box(frame, frac)
    x1, y1, x2, y2 = box
    return frame[y1:y2, x1:x2], box


def image_stats(frame: np.ndarray, roi_frac: float) -> dict[str, float]:
    crop, _ = crop_roi(frame, roi_frac)
    bgr = crop.reshape(-1, 3).astype(np.float32)
    luma = 0.114 * bgr[:, 0] + 0.587 * bgr[:, 1] + 0.299 * bgr[:, 2]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    return {
        "b_mean": float(bgr[:, 0].mean()),
        "g_mean": float(bgr[:, 1].mean()),
        "r_mean": float(bgr[:, 2].mean()),
        "luma_mean": float(luma.mean()),
        "luma_std": float(luma.std()),
        "luma_p05": float(np.percentile(luma, 5)),
        "luma_p95": float(np.percentile(luma, 95)),
        "sat_mean": float(hsv[:, :, 1].mean()),
        "clip_low_pct": float(np.mean(np.any(crop <= 5, axis=2)) * 100.0),
        "clip_high_pct": float(np.mean(np.any(crop >= 250, axis=2)) * 100.0),
        "sharpness_lap_var": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
    }


def summarize(rows: list[dict[str, object]], fields: list[str]) -> dict[str, float]:
    out: dict[str, float] = {}
    for field in fields:
        values = [float(row[field]) for row in rows if row.get(field) not in ("", None)]
        if not values:
            continue
        arr = np.array(values, dtype=np.float32)
        out[f"{field}_mean"] = float(arr.mean())
        out[f"{field}_std"] = float(arr.std())
    return out


def draw_panel(frame: np.ndarray, condition: str, stats: dict[str, float],
               roi_frac: float, width: int = 640) -> np.ndarray:
    panel = frame.copy()
    x1, y1, x2, y2 = roi_box(panel, roi_frac)
    cv2.rectangle(panel, (x1, y1), (x2, y2), (0, 255, 0), 3)
    scale = width / panel.shape[1]
    panel = cv2.resize(panel, (width, int(panel.shape[0] * scale)))
    lines = [
        condition,
        f"luma {stats['luma_mean']:.1f} +/- {stats['luma_std']:.1f}",
        f"clip hi {stats['clip_high_pct']:.2f}% lo {stats['clip_low_pct']:.2f}%",
        f"sat {stats['sat_mean']:.1f} sharp {stats['sharpness_lap_var']:.0f}",
    ]
    y = 28
    for line in lines:
        cv2.putText(panel, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4,
                    cv2.LINE_AA)
        cv2.putText(panel, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1,
                    cv2.LINE_AA)
        y += 28
    return panel


def load_siglip(labels: list[str]):
    from siglip_live import Siglip

    return Siglip(labels)


def classify_samples(clf, labels: list[str], frames: list[np.ndarray], roi_frac: float,
                     max_samples: int) -> dict[int, dict[str, object]]:
    if not frames or max_samples <= 0:
        return {}
    count = min(max_samples, len(frames))
    sample_idxs = sorted(set(int(i) for i in np.linspace(0, len(frames) - 1, count)))
    results: dict[int, dict[str, object]] = {}
    for idx in sample_idxs:
        crop, _ = crop_roi(frames[idx], roi_frac)
        t0 = time.time()
        sig, soft = clf.classify(crop)
        infer_ms = (time.time() - t0) * 1000.0
        order = np.argsort(-soft)
        best = int(order[0])
        second = int(order[1]) if len(order) > 1 else best
        results[idx] = {
            "siglip_best": labels[best],
            "siglip_margin": float(soft[best] - soft[second]),
            "siglip_best_softmax": float(soft[best]),
            "siglip_best_sigmoid": float(sig[best]),
            "siglip_ms": float(infer_ms),
            "siglip_softmax": {labels[i]: float(soft[i]) for i in range(len(labels))},
            "siglip_sigmoid": {labels[i]: float(sig[i]) for i in range(len(labels))},
        }
    return results


def capture_condition(cam: CsiCamera, condition: str, args) -> list[np.ndarray]:
    if not args.no_prompt:
        input(f"\nSet light state to [{condition}], place the fruit cube in the ROI, then press Enter...")
    for _ in range(max(0, args.warmup)):
        cam.read(timeout_s=args.timeout)
    frames = []
    for i in range(args.frames):
        frame = cam.read(timeout_s=args.timeout)
        if frame is None:
            print(f"[warn] {condition}: frame {i} timed out")
            continue
        frames.append(frame)
    if not frames:
        raise RuntimeError(f"no frames captured for condition {condition}")
    return frames


def open_camera(args) -> CsiCamera:
    exposuretimerange = "" if args.auto_exposure else args.exposuretimerange
    gainrange = "" if args.auto_exposure else args.gainrange
    aelock = False if args.auto_exposure else args.aelock
    awblock = False if args.auto_exposure else args.awblock
    wb_gains = None if args.raw else camera_config.WB_GAINS.get(args.sensor_id)
    return CsiCamera(
        sensor_id=args.sensor_id,
        sensor_mode=args.sensor_mode,
        width=args.width,
        height=args.height,
        fps=args.fps,
        flip=args.flip,
        wbmode=args.wbmode,
        exposuretimerange=exposuretimerange,
        gainrange=gainrange,
        aelock=aelock,
        awblock=awblock,
        nvargus_extra=args.nvargus_extra,
        wb_gains=wb_gains,
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor-id", type=int, default=camera_config.BODY)
    ap.add_argument("--sensor-mode", type=int, default=3)
    ap.add_argument("--width", type=int, default=1640)
    ap.add_argument("--height", type=int, default=1232)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--flip", type=int, default=0)
    ap.add_argument("--wbmode", type=int, default=8)
    ap.add_argument("--raw", action="store_true", help="disable software WB gains")
    ap.add_argument("--auto-exposure", action="store_true",
                    help="do not pass exposuretimerange/gainrange/aelock/awblock")
    ap.add_argument("--exposuretimerange", default="8000000 8000000",
                    help='fixed nvargus exposure range in ns, e.g. "8000000 8000000"')
    ap.add_argument("--gainrange", default="1 1", help='fixed nvargus gain range, e.g. "1 1"')
    ap.add_argument("--aelock", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--awblock", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--nvargus-extra", default="", help="extra nvarguscamerasrc properties")
    ap.add_argument("--conditions", default="light_off,light_on")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--warmup", type=int, default=15)
    ap.add_argument("--timeout", type=float, default=1.0)
    ap.add_argument("--roi", type=float, default=0.5)
    ap.add_argument("--out-dir", default="data/lighting_compare")
    ap.add_argument("--no-prompt", action="store_true")
    ap.add_argument("--siglip", action="store_true")
    ap.add_argument("--labels", default=DEFAULT_LABELS)
    ap.add_argument("--siglip-samples", type=int, default=5)
    args = ap.parse_args()

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(args.out_dir, run_id)
    os.makedirs(out_dir, exist_ok=True)

    labels = [label.strip() for label in args.labels.split(",") if label.strip()]
    clf = load_siglip(labels) if args.siglip else None
    conditions = [item.strip() for item in args.conditions.split(",") if item.strip()]

    config = vars(args).copy()
    config["run_id"] = run_id
    config["out_dir"] = out_dir
    rows: list[dict[str, object]] = []
    summary: dict[str, object] = {"config": config, "conditions": {}}
    panels = []

    print(f"[init] output: {out_dir}")
    print("[hint] Stop ROS camera_csi_node first; only one process can own the CSI sensor.")
    cam = open_camera(args)
    try:
        for condition in conditions:
            frames = capture_condition(cam, condition, args)
            sample_results = (
                classify_samples(clf, labels, frames, args.roi, args.siglip_samples)
                if clf is not None else {}
            )
            condition_rows: list[dict[str, object]] = []
            for idx, frame in enumerate(frames):
                image_path = os.path.join(out_dir, f"{condition}_{idx:03d}.png")
                cv2.imwrite(image_path, frame)
                row: dict[str, object] = {
                    "condition": condition,
                    "frame": idx,
                    "image": image_path,
                    **image_stats(frame, args.roi),
                }
                if idx in sample_results:
                    sig = sample_results[idx]
                    row.update({k: v for k, v in sig.items() if not isinstance(v, dict)})
                    row["siglip_softmax"] = json.dumps(sig["siglip_softmax"], sort_keys=True)
                    row["siglip_sigmoid"] = json.dumps(sig["siglip_sigmoid"], sort_keys=True)
                condition_rows.append(row)
                rows.append(row)

            fields = [
                "b_mean", "g_mean", "r_mean", "luma_mean", "luma_std", "luma_p05",
                "luma_p95", "sat_mean", "clip_low_pct", "clip_high_pct",
                "sharpness_lap_var", "siglip_margin", "siglip_best_softmax",
                "siglip_best_sigmoid", "siglip_ms",
            ]
            condition_summary = summarize(condition_rows, fields)
            sampled_best = [row.get("siglip_best") for row in condition_rows if row.get("siglip_best")]
            if sampled_best:
                condition_summary["siglip_best_votes"] = {
                    label: sampled_best.count(label) for label in sorted(set(sampled_best))
                }
            summary["conditions"][condition] = condition_summary
            mid = len(frames) // 2
            panels.append(draw_panel(frames[mid], condition, image_stats(frames[mid], args.roi), args.roi))
            print(
                f"[{condition}] luma={condition_summary.get('luma_mean_mean', 0):.1f} "
                f"clip_hi={condition_summary.get('clip_high_pct_mean', 0):.2f}% "
                f"margin={condition_summary.get('siglip_margin_mean', 0):.3f}"
            )
    finally:
        cam.release()

    csv_path = os.path.join(out_dir, "frames.csv")
    fieldnames = sorted({key for row in rows for key in row.keys()})
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    json_path = os.path.join(out_dir, "summary.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    if panels:
        max_w = max(panel.shape[1] for panel in panels)
        padded = []
        for panel in panels:
            if panel.shape[1] < max_w:
                pad = np.zeros((panel.shape[0], max_w - panel.shape[1], 3), dtype=np.uint8)
                panel = np.hstack([panel, pad])
            padded.append(panel)
        contact_path = os.path.join(out_dir, "contact_sheet.jpg")
        cv2.imwrite(contact_path, np.vstack(padded))

    print(f"[done] csv: {csv_path}")
    print(f"[done] summary: {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
