#!/usr/bin/env python3
"""로밍 데이터 수집 — 메카넘 베이스로 조금 움직이고 정지→본체+광각 둘 다 촬영, 반복.

도형/과일큐브를 로봇 주위에 둘러놓고 실행. 한 사이클 = [모션 1개 짧게] → 정지·세틀 →
본체캠(sensor0)+광각캠(sensor1) 각 1장 저장. 베이스가 pivot(제자리회전)하면 로봇 전체가
돌아 두 카메라가 주변 도형을 여러 각도로 훑는다. 앞뒤는 소폭으로 시점/거리 다양화.

안전:
  - 도형이 로봇을 둘러싸므로 **이동은 짧고 저속**, pivot 위주(앞뒤는 드리프트 상쇄되게 번갈아).
  - Ctrl-C / 예외 시 즉시 (0,0,0,0) 정지. 매 사이클 정지 후 촬영(모션블러 없음).
  - 먼저 --dry-run 으로 두 카메라·시야 확인하고(움직이지 않음), 바닥 공간 확보 후 본실행.

사용:
  python3 scripts/roam_capture.py --dry-run                 # 안 움직이고 두 캠 1장씩 저장+계획출력
  python3 scripts/roam_capture.py                           # 본실행 (기본 30사이클)
  python3 scripts/roam_capture.py --cycles 48 --speed 0.18 --move-dur 0.5
  python3 scripts/roam_capture.py --cams body               # 한 캠만
  python3 scripts/roam_capture.py --no-arm-home             # 팔 HOME 안 시킴
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402  BODY=0, WIDE=1 (현재 배선 기준 권위)
from csi_capture import CsiCamera  # noqa: E402  gi/appsink + 역할별 WB

# 메카넘 4휠 부호 패턴 (fl,fr,rl,rr) — base_controller_node 역기구학과 동일.
MOTIONS = {
    "forward": (+1, +1, +1, +1),
    "back":    (-1, -1, -1, -1),
    "left":    (-1, +1, +1, -1),   # 횡이동(바닥에서만 방향 검증됨)
    "right":   (+1, -1, -1, +1),
    "ccw":     (-1, +1, -1, +1),   # 제자리 좌회전
    "cw":      (+1, -1, +1, -1),   # 제자리 우회전
    "stop":    (0, 0, 0, 0),
}
# 기본 로밍 패턴: pivot 위주로 주위를 훑고, 앞뒤는 번갈아 넣어 순(純)드리프트 ~0.
DEFAULT_PATTERN = "cw,cw,cw,forward,cw,cw,cw,back"


class BaseDriver:
    """<BASE,...> 텍스트 프로토콜. 모션은 20Hz 스트리밍, 정지는 0벡터 반복."""

    def __init__(self, port, speed):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        time.sleep(2.2)                       # MCU 리셋 대기
        self.ser.reset_input_buffer()
        self.speed = speed

    def _send(self, sp):
        fl, fr, rl, rr = sp
        self.ser.write(f"<BASE,{fl:.3f},{fr:.3f},{rl:.3f},{rr:.3f}>\n".encode("ascii"))

    def _drain(self):
        try:
            self.ser.reset_input_buffer()     # 펌웨어 ACK 비워 버퍼 교착 방지
        except Exception:
            pass

    def move(self, name, dur, hz=20.0):
        sp = tuple(p * self.speed for p in MOTIONS[name])
        end = time.time() + dur
        period = 1.0 / hz
        while time.time() < end:
            self._send(sp)
            time.sleep(period)
        self.stop()
        self._drain()

    def stop(self, n=3):
        for _ in range(n):
            self._send(MOTIONS["stop"])
            time.sleep(0.02)
        self.ser.flush()

    def close(self):
        try:
            self.stop(5)
        finally:
            self.ser.close()


def arm_home_once(port="/dev/ttyACM0"):
    """팔을 HOME으로 1회 이동(본체 eye-in-hand 캠 시야 일관화). best-effort."""
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                        "ros2_ws", "src", "robot_control", "robot_control"))
        import arm_ik
        import serial
        s = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(2.3)
        end = time.time() + 0.5
        while time.time() < end:
            s.read(256)
        s.write(("<ARM," + ",".join(str(int(round(v))) for v in arm_ik.HOME_CMD) + ">\n").encode())
        time.sleep(1.2)
        s.read(256)
        s.close()
        print("[arm] HOME 이동 완료")
        return True
    except Exception as exc:
        print(f"[arm] HOME 생략 (팔 제어 불가: {exc}) — 본체캠은 현재 팔 자세 기준으로 촬영됨")
        return False


def open_cams(which, wide_flip, raw):
    cams = {}
    if "body" in which:
        try:
            # raw면 WB 끔, 아니면 미지정 → CsiCamera가 본체 고정게인 WB 자동 적용.
            cams["body"] = (CsiCamera(camera_config.BODY, wb_gains=None) if raw
                            else CsiCamera(camera_config.BODY))
        except Exception as exc:
            print(f"[err] 본체캠(sensor {camera_config.BODY}) 열기 실패: {exc}", file=sys.stderr)
    if "wide" in which:
        try:
            cams["wide"] = CsiCamera(camera_config.WIDE, wb_gains=None, flip=wide_flip)
        except Exception as exc:
            print(f"[err] 광각캠(sensor {camera_config.WIDE}) 열기 실패: {exc}", file=sys.stderr)
            print("[hint] 광각은 marginal — 꽂은 채 reboot 또는 rebind 필요할 수 있음 "
                  "(reference-jetson-csi-hotplug).", file=sys.stderr)
    for c in cams.values():
        for _ in range(8):
            c.read(1.0)                       # 3A(노출/WB) 세틀
    return cams


def grab(cam, tries=4):
    f = None
    for _ in range(tries):
        g = cam.read(1.0)
        if g is not None:
            f = g                             # appsink drop=true라 마지막=최신
    return f


def _fname(batch, ts, camtag, i):
    """캡처 파일명: [배치라벨_]세션ts_카메라태그_인덱스 → 배치·캠 간 충돌·혼동 방지."""
    pfx = f"roam_{batch}_{ts}" if batch else f"roam_{ts}"
    return f"{pfx}_{camtag}_{i:03d}.jpg"


def return_to_start(base, fwd_motion, steps, dur):
    """fwd_motion 으로 steps번 돈 것을 반대로 풀어 시작 위치 복귀(선 꼬임 해제)."""
    opp = {"cw": "ccw", "ccw": "cw"}.get(fwd_motion)
    if not opp or steps == 0:
        return
    undo = opp if steps > 0 else fwd_motion
    print(f"[return] 시작 위치로 복귀 ({undo} x{abs(steps)}, 선 풀기)", flush=True)
    for _ in range(abs(steps)):
        base.move(undo, dur)
        time.sleep(0.15)


def run_manual(base, cams, outdir, step_motion, dur, settle, batch=""):
    """수동 스텝: SPACE=현재방향 회전+촬영, c=방향전환(cw<->ccw), b=반대로 한스텝(촬영X),
    r=시작복귀, q=복귀후종료. net=부호있는 cw스텝(시작기준)으로 선꼬임 추적 → q/r에서 복귀로 풀림.
    cw로 ~180도 쓸고 c로 ccw 전환해 되쓸면 ±범위 안에서 무한 반복 수집 가능."""
    pv = "wide" if "wide" in cams else next(iter(cams))   # 미리보기 캠(광각 우선)
    win = "roam MANUAL  SPACE=shot  c=flip dir  b=back  r=home  q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    ts = int(time.time())
    net = 0                                               # 부호있는 cw 스텝(시작 기준) = 선꼬임 지표
    cur = step_motion                                     # 현재 회전 방향
    shots = {t: 0 for t in cams}
    print("[manual] SPACE=회전+촬영 / c=방향전환 / b=반대로 / r=복귀 / q=복귀후종료. "
          "한쪽으로 ~180도면 c로 전환해 되쓸기(무한 반복).", flush=True)
    try:
        while True:
            f = grab(cams[pv], tries=1)
            if f is not None:
                hud = f"dir:{cur.upper()}  net-cw:{net}  shots:{sum(shots.values())}  (한쪽 180deg면 c)"
                for col, th in (((0, 0, 0), 4), ((255, 255, 255), 1)):
                    cv2.putText(f, hud, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, th, cv2.LINE_AA)
                if f.shape[1] > 960:
                    s = 960 / f.shape[1]
                    f = cv2.resize(f, (960, int(f.shape[0] * s)))
                cv2.imshow(win, f)
            k = cv2.waitKey(50) & 0xFF
            if k in (ord("q"), 27):
                break
            if k in (32, 13):                                 # SPACE/Enter: 현재방향 회전+촬영
                base.move(cur, dur)
                time.sleep(settle)
                net += 1 if cur == "cw" else -1
                for tag, cam in cams.items():
                    g = grab(cam)
                    if g is None:
                        print(f"  {tag} 캡처실패"); continue
                    cv2.imwrite(f"{outdir[tag]}/" + _fname(batch, ts, tag, shots[tag]), g)
                    shots[tag] += 1
                print(f"  {cur} net-cw {net}: " + " ".join(f"{t}:{shots[t]}" for t in cams), flush=True)
            elif k == ord("c"):                               # 방향 전환 (cw <-> ccw)
                cur = "ccw" if cur == "cw" else "cw"
                print(f"  방향 전환 -> {cur}", flush=True)
            elif k == ord("b"):                               # 반대로 한 스텝(촬영 안 함)
                opp = "ccw" if cur == "cw" else "cw"
                base.move(opp, dur); time.sleep(settle)
                net += 1 if opp == "cw" else -1
                print(f"  back({opp}) -> net-cw {net}", flush=True)
            elif k == ord("r"):                               # 시작 위치 복귀(선 풀기)
                return_to_start(base, "cw", net, dur); net = 0
    finally:
        return_to_start(base, "cw", net, dur)                 # 종료 시 선 풀기
        cv2.destroyAllWindows()
    print("[manual] 종료. 저장: " + ", ".join(f"{t}={shots[t]} ({outdir[t]})" for t in cams), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base-port", default="/dev/ttyUSB0", help="메카넘 베이스(CH340)")
    ap.add_argument("--cams", default="body,wide", help="모을 카메라: body,wide / body / wide")
    ap.add_argument("--cycles", type=int, default=30, help="정지·촬영 횟수")
    ap.add_argument("--speed", type=float, default=0.18, help="베이스 속도(저속 권장)")
    ap.add_argument("--move-dur", type=float, default=0.5, help="한 모션 지속(s, 짧게)")
    ap.add_argument("--settle", type=float, default=0.8, help="정지 후 촬영 전 대기(s)")
    ap.add_argument("--pattern", default=DEFAULT_PATTERN, help="사이클별 모션 시퀀스(쉼표)")
    ap.add_argument("--wide-flip", type=int, default=2, help="광각 flip-method (wide_capture_label과 동일=2)")
    ap.add_argument("--raw", action="store_true", help="본체캠 WB 보정 끔")
    ap.add_argument("--out-body", default="data/roam_capture/body")
    ap.add_argument("--out-wide", default="data/roam_capture/wide")
    ap.add_argument("--no-arm-home", action="store_true", help="팔 HOME 이동 생략")
    ap.add_argument("--manual", action="store_true",
                    help="수동 스텝 모드: 창에서 SPACE=회전+촬영, q=복귀후종료 (180도만 돌릴 때 권장)")
    ap.add_argument("--step-motion", default="cw", choices=["cw", "ccw"], help="수동 회전 방향")
    ap.add_argument("--batch", default="", help="파일명 배치 라벨(예: bodyextra2) — 배치 구분·중복 방지. 영숫자/_ 권장")
    ap.add_argument("--dry-run", action="store_true", help="안 움직이고 두 캠 1장씩만 저장+계획")
    args = ap.parse_args()

    which = [s.strip() for s in args.cams.split(",") if s.strip() in ("body", "wide")]
    if not which:
        print("--cams 는 body,wide 중 하나 이상"); return 1
    pattern = [m.strip() for m in args.pattern.split(",") if m.strip() in MOTIONS and m.strip() != "stop"]
    if not pattern:
        print("--pattern 에 유효 모션 없음"); return 1
    outdir = {"body": args.out_body, "wide": args.out_wide}
    for t in which:
        os.makedirs(outdir[t], exist_ok=True)

    print(f"[plan] cams={which}  cycles={args.cycles}  speed={args.speed}  move-dur={args.move_dur}s  settle={args.settle}s")
    print(f"[plan] pattern={pattern}  (사이클 i 모션 = pattern[i % {len(pattern)}])")
    print(f"[plan] 예상 저장 = {args.cycles} x {len(which)} = {args.cycles*len(which)}장")

    if not args.no_arm_home and not args.dry_run:
        arm_home_once()

    cams = open_cams(which, args.wide_flip, args.raw)
    if not cams:
        print("[err] 열린 카메라 없음 — 중단"); return 1
    if len(cams) < len(which):
        print(f"[warn] 요청 {which} 중 {list(cams)} 만 열림 — 그것만 수집")
    print(f"[init] 카메라 {list(cams)} 스트리밍 (본체 WB={'OFF' if args.raw else 'ON'})", flush=True)

    # ---- dry-run: 안 움직이고 두 캠 한 장씩 저장(시야 확인) ----
    if args.dry_run:
        for tag, cam in cams.items():
            f = grab(cam)
            if f is None:
                print(f"[dry] {tag} 프레임 실패"); continue
            p = f"{outdir[tag]}/dryrun_{args.batch + '_' if args.batch else ''}{tag}.jpg"
            cv2.imwrite(p, f)
            print(f"[dry] {tag} {f.shape[1]}x{f.shape[0]} -> {p}")
        for c in cams.values():
            c.release()
        print("[dry] 베이스 미연결/미이동. 시야 확인 후 --dry-run 빼고 본실행하세요.")
        return 0

    # ---- 베이스 연결 ----
    if not args.manual:
        print("[safety] 바닥 공간 확보 확인. 3초 후 자동주행 시작 (Ctrl-C 즉시정지) ...", flush=True)
        time.sleep(3.0)
    try:
        base = BaseDriver(args.base_port, args.speed)
    except Exception as exc:
        print(f"[err] 베이스 포트 {args.base_port} 열기 실패: {exc}", file=sys.stderr)
        print("[hint] CH340 인식/포트 확인 (reference-jetson-ch340-driver). --base-port 로 변경 가능.",
              file=sys.stderr)
        for c in cams.values():
            c.release()
        return 1

    # ---- 수동 스텝 모드: SPACE로 한 스텝씩, 종료 시 시작위치 복귀(선 풀기) ----
    if args.manual:
        try:
            run_manual(base, cams, outdir, args.step_motion, args.move_dur, args.settle, args.batch)
        finally:
            base.close()
            for c in cams.values():
                c.release()
        return 0

    # ---- 자동 로밍 ----
    ts = int(time.time())
    saved = {t: 0 for t in cams}
    try:
        for i in range(args.cycles):
            m = pattern[i % len(pattern)]
            base.move(m, args.move_dur)
            time.sleep(args.settle)
            for tag, cam in cams.items():
                f = grab(cam)
                if f is None:
                    print(f"  {i+1}/{args.cycles} [{m}] {tag} 캡처실패"); continue
                p = f"{outdir[tag]}/" + _fname(args.batch, ts, tag, i)
                cv2.imwrite(p, f); saved[tag] += 1
            done = " ".join(f"{t}:{saved[t]}" for t in cams)
            print(f"  {i+1}/{args.cycles} [{m}] -> {done}", flush=True)
    except KeyboardInterrupt:
        print("\n[stop] 사용자 중단 — 베이스 정지")
    finally:
        base.close()
        for c in cams.values():
            c.release()
    print(f"[done] 저장: " + ", ".join(f"{t}={saved[t]} ({outdir[t]})" for t in cams))
    print("       labelImg(노트북)로 패치/큐브 박싱 → 학습.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
