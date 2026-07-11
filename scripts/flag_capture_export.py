#!/usr/bin/env python3
"""Taeguk-flag dataset capture/export helper.

WASD drives the mecanum base for one short open-loop step, then stops and captures
the body/wide cameras. On export, the current per-camera YOLO models pre-label the
known object classes and LabelMe/X-AnyLabeling JSON files are written next to each
image. The new arrival class is included in classes.txt so the user can add/correct
`arrival` boxes before converting back to YOLO for training.

Typical use on the Jetson:
  python3 scripts/flag_capture_export.py --session flag0711
  python3 scripts/flag_capture_export.py --session flag0711 --export-only

Keys:
  w/s = forward/back, a/d = left/right, j/l = rotate left/right in place,
  q = stop + export, Ctrl-C = stop only.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402
from csi_capture import CsiCamera  # noqa: E402
from roam_capture import BaseDriver, load_lateral_scales, grab  # noqa: E402


DEFAULT_CLASSES = [
    "cube",
    "octahedron",
    "dodecahedron",
    "icosahedron",
    "fruit_photo_cube",
    "arrival",
]
KEY_TO_MOTION = {
    "w": "forward",
    "s": "back",
    "a": "left",
    "d": "right",
    "j": "ccw",
    "l": "cw",
}
XANY_VERSION = "2.4.4"


class MjpegStreamer:
    """Tiny MJPEG server for browser live preview."""

    def __init__(self, port: int) -> None:
        self.latest: bytes | None = None
        streamer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path == "/favicon.ico":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache, private")
                self.end_headers()
                try:
                    while True:
                        frame = streamer.latest
                        if frame is not None:
                            self.wfile.write(
                                b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                + str(len(frame)).encode("ascii") + b"\r\n\r\n" + frame + b"\r\n"
                            )
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self._httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()


def parse_classes(value: str) -> list[str]:
    names = [v.strip() for v in value.split(",") if v.strip()]
    return names or list(DEFAULT_CLASSES)


def ordered_classes(model_names: Any, fallback: list[str], arrival_class: str) -> list[str]:
    """Use the model's class order when available, then append the new arrival class."""
    names: list[str] = []
    if isinstance(model_names, dict):
        for idx in sorted(model_names):
            names.append(str(model_names[idx]))
    elif isinstance(model_names, (list, tuple)):
        names = [str(v) for v in model_names]
    if not names:
        names = [c for c in fallback if c != arrival_class]
    for c in fallback:
        if c not in names and c != arrival_class:
            names.append(c)
    if arrival_class not in names:
        names.append(arrival_class)
    return names


def ensure_session(root: Path, session: str, cams: list[str]) -> dict[str, Path]:
    paths = {}
    for cam in cams:
        d = root / session / cam / "images"
        d.mkdir(parents=True, exist_ok=True)
        paths[cam] = d
    return paths


def open_cameras(cams: list[str], wide_flip: int, raw_body: bool) -> dict[str, CsiCamera]:
    opened: dict[str, CsiCamera] = {}
    if "body" in cams:
        opened["body"] = CsiCamera(camera_config.BODY, wb_gains=None) if raw_body else CsiCamera(camera_config.BODY)
    if "wide" in cams:
        opened["wide"] = CsiCamera(camera_config.WIDE, wb_gains=None, flip=wide_flip)
    for cam in opened.values():
        for _ in range(8):
            cam.read(1.0)
    return opened


def next_index(image_dir: Path, session: str, cam: str) -> int:
    pat = f"{session}_{cam}_*.jpg"
    existing = sorted(image_dir.glob(pat))
    if not existing:
        return 0
    nums = []
    for p in existing:
        try:
            nums.append(int(p.stem.rsplit("_", 1)[1]))
        except (IndexError, ValueError):
            pass
    return max(nums, default=-1) + 1


def capture_all(cams: dict[str, CsiCamera], out_dirs: dict[str, Path], session: str, counters: dict[str, int]) -> None:
    for tag, cam in cams.items():
        frame = grab(cam)
        if frame is None:
            print(f"  {tag}: capture failed", flush=True)
            continue
        path = out_dirs[tag] / f"{session}_{tag}_{counters[tag]:04d}.jpg"
        cv2.imwrite(str(path), frame)
        counters[tag] += 1
        print(f"  {tag}: {path}", flush=True)


def make_preview(frames: dict[str, Any], counters: dict[str, int], session: str, width: int) -> Any | None:
    panels = []
    for tag in ("wide", "body"):
        frame = frames.get(tag)
        if frame is None:
            continue
        h, w = frame.shape[:2]
        if w <= 0 or h <= 0:
            continue
        out_w = max(320, width)
        out_h = int(h * out_w / w)
        panel = cv2.resize(frame, (out_w, out_h))
        label = f"{tag.upper()}  saved:{counters.get(tag, 0)}  session:{session}"
        for color, thick in (((0, 0, 0), 5), ((255, 255, 255), 2)):
            cv2.putText(panel, label, (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, thick, cv2.LINE_AA)
        panels.append(panel)
    if not panels:
        return None
    max_w = max(p.shape[1] for p in panels)
    padded = []
    for panel in panels:
        if panel.shape[1] == max_w:
            padded.append(panel)
            continue
        pad = max_w - panel.shape[1]
        padded.append(cv2.copyMakeBorder(panel, 0, 0, 0, pad, cv2.BORDER_CONSTANT, value=(25, 25, 25)))
    hud = cv2.copyMakeBorder(padded[0][:1, :], 0, 43, 0, 0, cv2.BORDER_CONSTANT, value=(20, 20, 20))
    text = "W/A/S/D = ~10cm + capture     J/L = rotate + capture     Q = stop + export"
    cv2.putText(hud, text, (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (80, 230, 255), 2, cv2.LINE_AA)
    return cv2.vconcat([hud] + padded)


def update_stream(cams: dict[str, CsiCamera], frames: dict[str, Any], streamer: MjpegStreamer | None,
                  counters: dict[str, int], session: str, width: int) -> None:
    if streamer is None:
        return
    for tag, cam in cams.items():
        frame = cam.read(0.02)
        if frame is not None:
            frames[tag] = frame
    preview = make_preview(frames, counters, session, width)
    if preview is None:
        return
    ok, buf = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 82])
    if ok:
        streamer.latest = buf.tobytes()


def motion_duration(args: argparse.Namespace, motion: str) -> float:
    if motion in ("left", "right"):
        return args.strafe_dur
    if motion in ("cw", "ccw"):
        return args.turn_dur
    return args.move_dur


def do_motion(base: BaseDriver, motion: str, args: argparse.Namespace, lateral_scales: dict | None) -> None:
    scales = None
    if motion in ("left", "right") and lateral_scales:
        scales = lateral_scales.get(motion)
    base.nudge(motion, motion_duration(args, motion), args.speed, scales=scales)


def run_capture(args: argparse.Namespace, cams: list[str], out_dirs: dict[str, Path]) -> None:
    import select
    import termios
    import tty

    if not sys.stdin.isatty():
        raise RuntimeError("WASD capture needs an interactive terminal. Use --export-only for an existing session.")
    opened = open_cameras(cams, args.wide_flip, args.raw_body)
    if not opened:
        raise RuntimeError("no camera opened")
    counters = {cam: next_index(out_dirs[cam], args.session, cam) for cam in opened}
    lateral_scales = load_lateral_scales()
    latest_frames: dict[str, Any] = {}
    streamer = None
    if args.mjpeg_port > 0:
        try:
            streamer = MjpegStreamer(args.mjpeg_port)
            streamer.start()
            print(
                f"[live] http://<jetson-ip>:{args.mjpeg_port}/  "
                f"(local: http://127.0.0.1:{args.mjpeg_port}/)",
                flush=True,
            )
        except OSError as exc:
            print(f"[live] MJPEG stream disabled: port {args.mjpeg_port} unavailable ({exc})", flush=True)

    base = BaseDriver(args.base_port, args.speed)
    print(
        "[capture] focus this terminal: w/a/s/d = ~10cm move + shot, "
        "j/l = in-place rotate + shot, q = stop + export, Ctrl-C = stop only",
        flush=True,
    )
    print(
        f"[capture] session={args.session} counters={counters} speed={args.speed} "
        f"dur(fwd={args.move_dur}s, strafe={args.strafe_dur}s, turn={args.turn_dur}s)",
        flush=True,
    )

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    aborted = False
    last_stream = 0.0
    try:
        tty.setcbreak(fd)
        while True:
            now = time.time()
            if now - last_stream >= args.stream_interval:
                update_stream(opened, latest_frames, streamer, counters, args.session, args.stream_width)
                last_stream = now
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                continue
            key = sys.stdin.read(1).lower()
            if key in ("q", "\x1b"):
                break
            motion = KEY_TO_MOTION.get(key)
            if motion is None:
                continue
            do_motion(base, motion, args, lateral_scales)
            time.sleep(args.settle)
            capture_all(opened, out_dirs, args.session, counters)
            update_stream(opened, latest_frames, streamer, counters, args.session, args.stream_width)
            print("  saved: " + " ".join(f"{c}:{counters[c]}" for c in counters), flush=True)
    except KeyboardInterrupt:
        aborted = True
        print("\n[stop] Ctrl-C: base stopped, export skipped unless --export-only is run later.", flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        base.close()
        if streamer is not None:
            streamer.stop()
        for cam in opened.values():
            cam.release()
    if aborted:
        raise SystemExit(130)


def yolo_to_labelme_doc(image_path: Path, image_w: int, image_h: int, detections: list[tuple], classes: list[str]) -> dict:
    shapes = []
    for x1, y1, x2, y2, cls_id, conf in detections:
        if not (0 <= int(cls_id) < len(classes)):
            continue
        x0 = max(0.0, min(float(image_w), float(x1)))
        y0 = max(0.0, min(float(image_h), float(y1)))
        x3 = max(0.0, min(float(image_w), float(x2)))
        y3 = max(0.0, min(float(image_h), float(y2)))
        if x3 <= x0 or y3 <= y0:
            continue
        shapes.append({
            "label": classes[int(cls_id)],
            "points": [[round(x0, 2), round(y0, 2)], [round(x3, 2), round(y3, 2)]],
            "group_id": None,
            "description": f"prelabel_conf={conf:.3f}",
            "difficult": False,
            "shape_type": "rectangle",
            "flags": {},
            "attributes": {},
        })
    return {
        "version": XANY_VERSION,
        "flags": {},
        "shapes": shapes,
        "imagePath": image_path.name,
        "imageData": None,
        "imageHeight": image_h,
        "imageWidth": image_w,
    }


def prelabel_cam(cam: str, image_dir: Path, export_dir: Path, model_path: str, fallback_classes: list[str],
                 arrival_class: str, imgsz: int, conf: float) -> tuple[int, int, list[str]]:
    from ultralytics import YOLO

    model = YOLO(model_path)
    classes = ordered_classes(getattr(model, "names", None), fallback_classes, arrival_class)
    export_dir.mkdir(parents=True, exist_ok=True)
    (export_dir / "classes.txt").write_text("\n".join(classes) + "\n", encoding="utf-8")

    images = sorted([*image_dir.glob("*.jpg"), *image_dir.glob("*.png"), *image_dir.glob("*.jpeg")])
    made = boxes = 0
    for src in images:
        frame = cv2.imread(str(src))
        if frame is None:
            continue
        h, w = frame.shape[:2]
        result = model.predict(frame, imgsz=imgsz, conf=conf, verbose=False, agnostic_nms=True, iou=0.6)[0]
        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detections.append((x1, y1, x2, y2, int(box.cls[0]), float(box.conf[0])))
        dst_img = export_dir / src.name
        shutil.copy2(src, dst_img)
        doc = yolo_to_labelme_doc(dst_img, w, h, detections, classes)
        (export_dir / f"{src.stem}.json").write_text(
            json.dumps(doc, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        made += 1
        boxes += len(doc["shapes"])
    print(f"[prelabel] {cam}: images={made}, boxes={boxes}, classes={classes}", flush=True)
    return made, boxes, classes


def export_dataset(args: argparse.Namespace, cams: list[str], root: Path) -> Path:
    session_root = root / args.session
    export_root = session_root / "labeled_json"
    if export_root.exists() and args.overwrite_export:
        shutil.rmtree(export_root)
    export_root.mkdir(parents=True, exist_ok=True)

    fallback = parse_classes(args.classes)
    models = {"body": args.body_model, "wide": args.wide_model}
    imgsz = {"body": args.body_imgsz, "wide": args.wide_imgsz}
    total_images = 0
    total_boxes = 0
    for cam in cams:
        image_dir = session_root / cam / "images"
        if not image_dir.exists():
            print(f"[export] skip {cam}: no images at {image_dir}", flush=True)
            continue
        made, boxes, _ = prelabel_cam(
            cam,
            image_dir,
            export_root / cam,
            models[cam],
            fallback,
            args.arrival_class,
            imgsz[cam],
            args.conf,
        )
        total_images += made
        total_boxes += boxes

    manifest = {
        "session": args.session,
        "created_at_kst": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
        "arrival_class": args.arrival_class,
        "classes": parse_classes(args.classes),
        "body_model": args.body_model,
        "wide_model": args.wide_model,
        "body_imgsz": args.body_imgsz,
        "wide_imgsz": args.wide_imgsz,
        "conf": args.conf,
        "images": total_images,
        "prelabel_boxes": total_boxes,
        "next_step": (
            "Open labeled_json/body and labeled_json/wide in X-AnyLabeling, add/correct "
            f"{args.arrival_class} rectangles, then convert JSON to YOLO with scripts/labelme_to_yolo.py."
        ),
    }
    (export_root / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")

    zip_base = session_root / f"{args.session}_body_wide_labeled_json"
    zip_path = Path(shutil.make_archive(str(zip_base), "zip", export_root))
    print(f"[export] zip={zip_path} images={total_images} prelabel_boxes={total_boxes}", flush=True)
    return zip_path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", default=time.strftime("flag_%Y%m%d_%H%M%S"), help="capture/export session name")
    ap.add_argument("--root", default="data/flag_capture", help="session root")
    ap.add_argument("--cams", default="body,wide", help="body,wide / body / wide")
    ap.add_argument("--base-port", default="/dev/ttyUSB0")
    ap.add_argument("--speed", type=float, default=0.375, help="wheel speed for each key step")
    ap.add_argument("--move-dur", type=float, default=0.29, help="forward/back duration per key; ~10cm at default speed")
    ap.add_argument("--strafe-dur", type=float, default=0.56, help="left/right duration per key; ~10cm at default speed")
    ap.add_argument("--turn-dur", type=float, default=0.25, help="in-place rotation duration per j/l key")
    ap.add_argument("--settle", type=float, default=0.15, help="pause after stop before capture")
    ap.add_argument("--wide-flip", type=int, default=2)
    ap.add_argument("--mjpeg-port", type=int, default=8090, help="live browser MJPEG port; 0 disables")
    ap.add_argument("--stream-width", type=int, default=960, help="preview panel width in pixels")
    ap.add_argument("--stream-interval", type=float, default=0.15, help="seconds between preview updates")
    ap.add_argument("--raw-body", action="store_true", help="disable body fixed white-balance gains")
    ap.add_argument("--export-only", action="store_true", help="skip capture, export an existing session")
    ap.add_argument("--no-export", action="store_true", help="capture only")
    ap.add_argument("--overwrite-export", action="store_true")
    ap.add_argument("--body-model", default="models/cube.pt")
    ap.add_argument("--wide-model", default="models/wide.pt")
    ap.add_argument("--body-imgsz", type=int, default=640)
    ap.add_argument("--wide-imgsz", type=int, default=1280)
    ap.add_argument("--conf", type=float, default=0.20)
    ap.add_argument("--arrival-class", default="arrival")
    ap.add_argument("--flag-class", dest="arrival_class", help=argparse.SUPPRESS)
    ap.add_argument("--classes", default=",".join(DEFAULT_CLASSES), help="comma-separated final class order")
    args = ap.parse_args()

    cams = [c.strip() for c in args.cams.split(",") if c.strip() in ("body", "wide")]
    if not cams:
        print("--cams must include body and/or wide", file=sys.stderr)
        return 1
    root = Path(args.root)
    out_dirs = ensure_session(root, args.session, cams)

    if not args.export_only:
        run_capture(args, cams, out_dirs)
    if not args.no_export:
        for cam, model in (("body", args.body_model), ("wide", args.wide_model)):
            if cam in cams and not Path(model).exists():
                print(f"[err] missing {cam} model: {model}", file=sys.stderr)
                return 1
        export_dataset(args, cams, root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
