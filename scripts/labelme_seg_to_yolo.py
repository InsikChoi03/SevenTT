#!/usr/bin/env python3
"""Convert X-AnyLabeling/LabelMe polygon JSON labels to YOLO segmentation txt.

Expected input folder:
  input_dir/
    image_001.jpg
    image_001.json
    image_002.jpg
    image_002.json

Output folder:
  output_dir/
    images/
      image_001.jpg
    labels/
      image_001.txt
    classes.txt

Example:
  python3 scripts/labelme_seg_to_yolo.py \
    --src /path/to/labeled_files \
    --out data/wallseg_dataset_v1 \
    --classes wall_floor_boundary \
    --zip data/wallseg_dataset_v1.zip
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import zipfile
from pathlib import Path


IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def clamp01(v: float) -> float:
    return min(max(float(v), 0.0), 1.0)


def image_path_for_json(jpath: Path, data: dict) -> Path | None:
    image_path = data.get("imagePath")
    candidates: list[Path] = []
    if image_path:
        candidates.append(jpath.parent / image_path)
        candidates.append(jpath.parent / Path(image_path).name)
    for ext in IMAGE_EXTS:
        candidates.append(jpath.with_suffix(ext))
    for cand in candidates:
        if cand.exists() and cand.is_file():
            return cand
    return None


def rectangle_to_polygon(points: list[list[float]]) -> list[list[float]]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def normalize_polygon(points: list[list[float]], width: int, height: int) -> list[float]:
    vals: list[float] = []
    for x, y in points:
        vals.append(clamp01(x / width))
        vals.append(clamp01(y / height))
    return vals


def convert_one(jpath: Path, name_to_id: dict[str, int], allow_rectangles: bool) -> tuple[list[str], list[str], Path | None]:
    data = json.loads(jpath.read_text(encoding="utf-8"))
    width = data.get("imageWidth")
    height = data.get("imageHeight")
    warns: list[str] = []
    if not width or not height:
        return [], [f"{jpath.name}: imageWidth/imageHeight missing"], None

    img = image_path_for_json(jpath, data)
    if img is None:
        return [], [f"{jpath.name}: matching image missing"], None

    lines: list[str] = []
    for shape in data.get("shapes", []):
        label = shape.get("label")
        if label not in name_to_id:
            warns.append(f"{jpath.name}: unknown label '{label}' skipped")
            continue

        shape_type = shape.get("shape_type", "polygon")
        points = shape.get("points") or []
        if shape_type == "polygon":
            if len(points) < 3:
                warns.append(f"{jpath.name}: polygon with fewer than 3 points skipped")
                continue
        elif shape_type == "rectangle" and allow_rectangles:
            points = rectangle_to_polygon(points)
        else:
            warns.append(f"{jpath.name}: shape_type '{shape_type}' skipped")
            continue

        norm = normalize_polygon(points, int(width), int(height))
        if len(norm) < 6:
            warns.append(f"{jpath.name}: too few polygon coordinates skipped")
            continue
        coords = " ".join(f"{v:.6f}" for v in norm)
        lines.append(f"{name_to_id[label]} {coords}")

    return lines, warns, img


def make_zip(out_dir: Path, zip_path: Path) -> None:
    if zip_path.exists():
        zip_path.unlink()
    root_name = out_dir.name
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(out_dir.rglob("*")):
            if path.is_file():
                zf.write(path, Path(root_name) / path.relative_to(out_dir))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--src", required=True, help="Folder containing images and LabelMe/X-AnyLabeling json files")
    parser.add_argument("--out", required=True, help="Output dataset folder")
    parser.add_argument("--classes", default="wall_floor_boundary", help="Comma-separated class names in YOLO id order")
    parser.add_argument("--zip", default="", help="Optional output zip path")
    parser.add_argument("--allow-rectangles", action="store_true", help="Convert rectangle shapes to 4-point polygons")
    args = parser.parse_args()

    src = Path(args.src)
    out = Path(args.out)
    classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    name_to_id = {name: idx for idx, name in enumerate(classes)}

    if not src.exists():
        raise SystemExit(f"[err] source folder not found: {src}")
    jsons = sorted(src.rglob("*.json"))
    if not jsons:
        raise SystemExit(f"[err] no json files found under: {src}")

    out_images = out / "images"
    out_labels = out / "labels"
    if out.exists():
        shutil.rmtree(out)
    out_images.mkdir(parents=True, exist_ok=True)
    out_labels.mkdir(parents=True, exist_ok=True)

    converted = 0
    total_objects = 0
    empty = 0
    missing_image = 0
    warns: list[str] = []

    for jpath in jsons:
        lines, shape_warns, img = convert_one(jpath, name_to_id, args.allow_rectangles)
        warns.extend(shape_warns)
        if img is None:
            missing_image += 1
            continue

        out_img = out_images / img.name
        out_lbl = out_labels / f"{img.stem}.txt"
        shutil.copy2(img, out_img)
        out_lbl.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

        converted += 1
        total_objects += len(lines)
        if not lines:
            empty += 1

    (out / "classes.txt").write_text("\n".join(classes) + "\n", encoding="utf-8")

    if args.zip:
        make_zip(out, Path(args.zip))

    print(f"[done] json: {len(jsons)}, images converted: {converted}, objects: {total_objects}")
    print(f"[info] empty label files: {empty}, missing images: {missing_image}")
    print(f"[out] {out}")
    if args.zip:
        print(f"[zip] {args.zip}")
    if warns:
        print(f"[warn] {len(warns)} warnings. First 10:")
        for warn in warns[:10]:
            print(f"  - {warn}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
