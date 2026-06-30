#!/usr/bin/env python3
"""자동 라벨링: FastSAM(검출) + SigLIP(분류) → YOLO 학습 라벨 생성.

수백 장을 자동으로 박스+클래스 라벨링하고, 사람이 검토 이미지로 확인/수정만 하면 됨.
(특히 12/20면체는 SigLIP이 약해 review_flag 표시)

워크플로:
  1) 캡처:   python3 scripts/autolabel.py --capture 60 --interval 1.5 --sensor-id 1
  2) 자동라벨: python3 scripts/autolabel.py --process
  3) 사람 검토: data/dataset/review/*.png 보고 labels/*.txt 수정
  4) 학습:    yolo detect train data=data/dataset/data.yaml model=yolov8n.pt epochs=80 imgsz=640

  단일 이미지 시연: python3 scripts/autolabel.py --image data/cam_test/dc_raw.png
"""
from __future__ import annotations
import argparse, os, sys, time, glob
import cv2, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402
import box_utils  # noqa: E402

FASTSAM_W = "models/FastSAM-s.pt"
SIGLIP_ID = "google/siglip-base-patch16-224"
SHAPES = ["cube", "octahedron", "dodecahedron", "icosahedron"]   # YOLO class id = index
REJECTS = ["background", "wooden floor", "a cardboard box", "an empty surface", "a wall", "a shadow", "a hand", "a cable"]
ROOT = "data/dataset"
IMG_DIR, LBL_DIR, REV_DIR = f"{ROOT}/images", f"{ROOT}/labels", f"{ROOT}/review"
_COL = [(0,255,0),(0,200,255),(255,150,0),(255,0,200)]


def ensure_dirs():
    for d in (IMG_DIR, LBL_DIR, REV_DIR):
        os.makedirs(d, exist_ok=True)


def write_data_yaml():
    p = f"{ROOT}/data.yaml"
    with open(p, "w") as f:
        f.write(f"path: {os.path.abspath(ROOT)}\ntrain: images\nval: images\n")
        f.write(f"nc: {len(SHAPES)}\nnames: {SHAPES}\n")
    return p


# ---------------- capture ----------------
def do_capture(n, interval, sensor_id):
    from csi_capture import CsiCamera
    cam = CsiCamera(sensor_id)
    for _ in range(10): cam.read(1.0)
    print(f"[capture] {n}장, {interval}s 간격. 매 컷마다 물체 위치/각도/조명 바꾸세요.", flush=True)
    base = int(time.time())
    saved = 0
    try:
        for i in range(n):
            print(f"  {i+1}/{n} ...", flush=True)
            f = None
            for _ in range(4):
                g = cam.read(1.0)
                if g is not None: f = g
            if f is None:
                print("   프레임 실패"); continue
            path = f"{IMG_DIR}/cap_{base}_{i:03d}.jpg"
            cv2.imwrite(path, f); saved += 1
            time.sleep(interval)
    finally:
        cam.release()
    print(f"[capture] {saved}장 저장 -> {IMG_DIR}")


# ---------------- model singletons ----------------
_fs = _proc = _sg = _texts = None
def load_models():
    global _fs, _proc, _sg, _texts
    if _fs is not None: return
    import torch
    from ultralytics import FastSAM
    from transformers import AutoProcessor, AutoModel
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[init] FastSAM + SigLIP on {dev}", flush=True)
    _fs = FastSAM(FASTSAM_W)
    _proc = AutoProcessor.from_pretrained(SIGLIP_ID)
    _sg = AutoModel.from_pretrained(SIGLIP_ID).to(dev).eval()
    labels = SHAPES + REJECTS
    _texts = [f"This is a photo of a {l}." if not l.startswith(("a ","an ")) else f"This is a photo of {l}." for l in labels]


def iou(a,b):
    ix0,iy0=max(a[0],b[0]),max(a[1],b[1]); ix1,iy1=min(a[2],b[2]),min(a[3],b[3])
    iw,ih=max(0,ix1-ix0),max(0,iy1-iy0); inter=iw*ih
    ua=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/ua if ua>0 else 0
def nms(boxes,scores,thr=0.5):
    order=sorted(range(len(boxes)),key=lambda i:scores[i],reverse=True); keep=[]
    while order:
        i=order.pop(0); keep.append(i); order=[j for j in order if iou(boxes[i],boxes[j])<thr]
    return keep


def label_image(path, force_class=None, keep_thresh=0.45, min_af=0.004, max_af=0.30, min_sol=0.78):
    """한 이미지 → (yolo_lines, review_img, flags). FastSAM 박스 → SigLIP.

    force_class 지정 시: SigLIP은 '물체 vs 배경'만 판정(거름), 라벨은 force_class로 고정
    (클래스별 배치 촬영 — 종류 추측 불필요, 손수정 거의 없음)."""
    import torch
    from PIL import Image
    frame = cv2.imread(path)
    if frame is None: return None
    H, W = frame.shape[:2]
    r = _fs(frame, device="cuda" if torch.cuda.is_available() else "cpu",
            imgsz=768, conf=0.4, iou=0.9, retina_masks=True, verbose=False)[0]
    cand = []
    if r.masks is not None:
        for m in r.masks.data.cpu().numpy():
            mb=(m>0.5).astype(np.uint8)
            if mb.shape[:2]!=(H,W): mb=cv2.resize(mb,(W,H),interpolation=cv2.INTER_NEAREST)
            a=int(mb.sum())
            if a<min_af*H*W or a>max_af*H*W: continue
            cs,_=cv2.findContours(mb,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            if not cs: continue
            c=max(cs,key=cv2.contourArea); ha=cv2.contourArea(cv2.convexHull(c))
            if ha<=0 or cv2.contourArea(c)/ha<min_sol: continue
            x,y,bw,bh=cv2.boundingRect(c); cand.append(((x,y,x+bw,y+bh),a))
    boxes=[b for b,_ in cand]; scores=[s for _,s in cand]
    sel=[boxes[i] for i in nms(boxes,scores,0.5)]

    lines=[]; review=frame.copy(); flags=[]
    kept=[]   # (x0,y0,x1,y1,cls,conf)
    if sel:
        crops=[Image.fromarray(cv2.cvtColor(frame[y0:y1,x0:x1],cv2.COLOR_BGR2RGB)) for (x0,y0,x1,y1) in sel]
        inp=_proc(text=_texts,images=crops,return_tensors="pt",padding="max_length").to(_sg.device)
        with torch.no_grad(): soft=torch.softmax(_sg(**inp).logits_per_image,dim=-1).cpu().numpy()
        for bi,(x0,y0,x1,y1) in enumerate(sel):
            j=int(np.argmax(soft[bi])); p=float(soft[bi][j])
            is_obj = j < len(SHAPES)                  # SigLIP: 도형(=물체) vs background
            if force_class is not None:
                keep, cls, conf = is_obj, force_class, float(soft[bi][force_class])
            else:
                keep, cls, conf = (is_obj and p>=keep_thresh), j, p
            if keep:
                kept.append((x0,y0,x1,y1,cls,conf))
            else:
                cv2.rectangle(review,(x0,y0),(x1,y1),(0,0,255),1)   # 거부=얇은 빨강
    # 같은 물체 중복/박스속박스 제거 (부분가림은 유지)
    for (x0,y0,x1,y1,cls,conf) in box_utils.dedupe(kept):
        cx=((x0+x1)/2)/W; cy=((y0+y1)/2)/H; w=(x1-x0)/W; h=(y1-y0)/H
        lines.append(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
        col=_COL[cls%len(_COL)]
        cv2.rectangle(review,(x0,y0),(x1,y1),col,3)
        weak = (force_class is None) and (conf<0.7 or SHAPES[cls] in ("dodecahedron","icosahedron"))
        tag = SHAPES[cls] + ("" if force_class is not None else f" {conf:.2f}") + (" ?" if weak else "")
        if weak: flags.append(f"{SHAPES[cls]}({conf:.2f})")
        cv2.putText(review,tag,(x0,max(30,y0-10)),cv2.FONT_HERSHEY_SIMPLEX,1.6,(0,0,0),6)
        cv2.putText(review,tag,(x0,max(30,y0-10)),cv2.FONT_HERSHEY_SIMPLEX,1.6,col,2)
    return lines, review, flags


def do_process(images_dir, force_class=None, relabel=False):
    ensure_dirs(); load_models()
    imgs=sorted(glob.glob(f"{images_dir}/*.jpg")+glob.glob(f"{images_dir}/*.png"))
    mode = SHAPES[force_class] if force_class is not None else "SigLIP자동분류"
    print(f"[process] {len(imgs)}장 (class={mode})")
    tot=flagged=skip=0
    for p in imgs:
        name=os.path.splitext(os.path.basename(p))[0]
        if not relabel and os.path.exists(f"{LBL_DIR}/{name}.txt"):
            skip+=1; continue        # 이미 라벨됨 → 배치별 처리 보호(다른 클래스 배치 덮어쓰기 방지)
        out=label_image(p, force_class=force_class)
        if out is None: continue
        lines,review,flags=out
        with open(f"{LBL_DIR}/{name}.txt","w") as f: f.write("\n".join(lines) + ("\n" if lines else ""))
        cv2.imwrite(f"{REV_DIR}/{name}.png",review)
        tot+=len(lines)
        fl = " ⚠검토:"+",".join(flags) if flags else ""
        if flags: flagged+=1
        print(f"  {name}: {len(lines)}개{fl}")
    yml=write_data_yaml()
    print(f"[process] 라벨 {tot}개, 건너뜀(이미라벨) {skip}장, 검토필요 {flagged}장. data.yaml={yml}")
    print(f"  검토: {REV_DIR}/*.png 보고 {LBL_DIR}/*.txt 수정")


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--capture",type=int,default=0)
    ap.add_argument("--interval",type=float,default=1.5)
    ap.add_argument("--sensor-id",type=int,default=camera_config.BODY)
    ap.add_argument("--process",action="store_true")
    ap.add_argument("--image",help="단일 이미지 시연(데이터셋에 복사 후 라벨)")
    ap.add_argument("--images-dir",default=IMG_DIR)
    ap.add_argument("--class", dest="cls_name", default=None,
                    help=f"배치 전체를 이 클래스로 고정 {SHAPES} (SigLIP 종류추측 무시, 배경만 거름)")
    ap.add_argument("--relabel", action="store_true", help="이미 라벨된 것도 다시 라벨")
    args=ap.parse_args()
    ensure_dirs()
    fc=None
    if args.cls_name:
        if args.cls_name not in SHAPES:
            print(f"--class 는 {SHAPES} 중 하나여야 함"); return 1
        fc=SHAPES.index(args.cls_name)
    if args.capture>0: do_capture(args.capture,args.interval,args.sensor_id); return 0
    if args.image:
        import shutil
        dst=f"{IMG_DIR}/{os.path.basename(args.image)}"; shutil.copy(args.image,dst)
        load_models(); out=label_image(dst, force_class=fc)
        lines,review,flags=out; name=os.path.splitext(os.path.basename(dst))[0]
        open(f"{LBL_DIR}/{name}.txt","w").write("\n".join(lines) + ("\n" if lines else ""))
        cv2.imwrite(f"{REV_DIR}/{name}.png",review); write_data_yaml()
        print(f"라벨 {len(lines)}개 -> {LBL_DIR}/{name}.txt");
        for l in lines: print("  ",l)
        print(f"검토필요: {flags}"); print(f"리뷰img: {REV_DIR}/{name}.png")
        return 0
    if args.process: do_process(args.images_dir, force_class=fc, relabel=args.relabel); return 0
    ap.print_help(); return 0


if __name__=="__main__":
    raise SystemExit(main())
