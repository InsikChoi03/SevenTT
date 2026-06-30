#!/usr/bin/env python3
"""저장 이미지에 YOLO-World를 대량 클래스로 돌려 무엇이 잡히는지 본다.

  python3 scripts/yolo_image.py data/cam_test/closeup.png --conf 0.05
"""
import argparse
import cv2

DEFAULT = ("cube,box,white cube,dice,block,white block,paper box,carton,pyramid,"
           "white pyramid,tetrahedron,octahedron,icosahedron,dodecahedron,polyhedron,"
           "white polyhedron,geometric solid,3d printed object,white object,"
           "object on the floor,white object on the floor,white toy,white ornament,"
           "white sculpture,crystal,white ball,paper model")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--weights", default="models/yolov8s-world.pt")
    ap.add_argument("--classes", default=DEFAULT)
    ap.add_argument("--conf", type=float, default=0.05)
    ap.add_argument("--imgsz", type=int, default=960)
    args = ap.parse_args()

    classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    img = cv2.imread(args.src)
    if img is None:
        print(f"읽기 실패: {args.src}"); return 1

    from ultralytics import YOLOWorld
    model = YOLOWorld(args.weights)
    model.set_classes(classes)
    print(f"[init] {len(classes)} classes, conf={args.conf}, imgsz={args.imgsz}")

    r = model.predict(img, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
    n = 0 if r.boxes is None else len(r.boxes)
    print(f"detections={n}")
    if n:
        for b in r.boxes:
            cls = int(b.cls[0]); conf = float(b.conf[0])
            x1, y1, x2, y2 = (int(v) for v in b.xyxy[0].tolist())
            label = classes[cls] if cls < len(classes) else str(cls)
            print(f"  {label:24s} {conf:.3f}  bbox=({x1},{y1},{x2},{y2})")
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.putText(img, f"{label} {conf:.2f}", (x1, max(14, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
    out = args.src.rsplit(".", 1)[0] + "_yolo.png"
    cv2.imwrite(out, img)
    print(f"-> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
