#!/usr/bin/env python3
"""모의 경기 주행 파이프라인 (dead-reckoning opening + 검증된 boost/brake 프리미티브).

motion_tune.py 가 개별 모션을 "튜닝"하는 도구였다면, 이 스크립트는 튜닝으로 수렴한
프리미티브를 이어붙여 "경기 시작 직후의 정해진 동선"을 재생한다.

가정한 시작 자세:
    4m x 4m 맵의 **우측 하단**에서, 로봇 정면(+x)이 필드 안쪽(전방)을 향한다.
    경기가 시작되면 아래 ROUTE 를 순서대로 실행한다. 최초 두 스텝은 하드코딩:
        1) 전진 30 cm      (forward 30)
        2) 우측 횡이동 30 cm (right   30)

모든 스텝은 motion_tune 의 검증된 실행 패턴을 그대로 탄다:
    boost(정지마찰 킥) -> steady(정속) -> brake(역펄스 제동)
즉 "부스터/브레이크 제어"가 모든 움직임에 들어간다. --dry-run 으로 하드웨어 없이
각 스텝의 boost/brake 구성을 표로 확인할 수 있다 (boost_ms/brake_ms 가 0이면 경고).

회전(cw/ccw)은 의도적으로 **느리게**: 정속 speed 를 0.5 -> ROT_SPEED(0.32) 로 낮추고
(정지마찰은 boost 로 깨고 나서 크롤), duration seed 를 그만큼 늘려 잡는다.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# 리포 루트에서 실행해도 형제 모듈을 찾도록 자기 디렉터리를 경로에 추가.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import serial  # noqa: E402

from encoder_distance_test import stop_base  # noqa: E402
from motion_tune import (  # noqa: E402
    DEFAULT_PARAMS,
    DEFAULT_SCALES,
    KINDS,
    duration_seed,
    run_continuous,
    wheel_cmd,
)

# --- 느린 회전 설정 (사용자 요구: 시계/반시계 충분히 천천히, 단 움직이긴 해야 함) ------------
# 회전이 빨랐던 원인은 (a) 높은 정속 speed(0.5) + (b) 강한 boost 가 짧은 구간에 큰 각도를 실어서다.
# 그래서 정속을 base_controller 검증 바닥값(wheel_min=0.30)까지 낮춰 크롤하고, boost 는 정지마찰만
# 깨는 짧은 킥으로 최소화한다. 정지 후 coast(무제동 관성)로 과회전하던 것은 brake 펄스로 잘라낸다.
ROT_SPEED = 0.30            # 정속(크롤) — 이 밑은 스톨(버즈) 위험
ROT_BOOST_SPEED = 0.60      # 정지마찰 킥
ROT_BOOST_MS = 90           # 딱 출발만. 길면 boost 가 회전을 지배해 다시 빨라짐
ROT_BRAKE_MS = 120          # 정지 coast 컷 (튜닝 땐 brake=0 이라 짧은 회전이 과회전했음)
ROT_BRAKE_SCALE = 0.30

# 각속도(deg/s) ≈ ROT_RATE_PER_SPEED * wheel_speed 가정. 0.5 정속에서 ~270deg/s(180°=665ms,
# 120ms boost@0.65 + 545ms steady@0.5) 로부터 역산한 경험 seed. 실측으로 재보정 대상.
ROT_RATE_PER_SPEED = 550.0


def rotation_duration_ms(direction: str, deg: float) -> int:
    """느린 회전 duration seed. boost 구간이 이미 만드는 각도를 빼고 정속 구간만 산출해 더한다
    (전체 duration 을 그냥 스케일하면 boost 기여를 이중 계산해 여전히 빨라짐)."""
    boost_deg = ROT_RATE_PER_SPEED * ROT_BOOST_SPEED * (ROT_BOOST_MS / 1000.0)
    remaining_deg = max(0.0, abs(deg) - boost_deg)
    steady_ms = remaining_deg / max(ROT_RATE_PER_SPEED * ROT_SPEED, 1e-6) * 1000.0
    return max(1, int(round(ROT_BOOST_MS + steady_ms)))


def make_step(direction: str, magnitude: float, **overrides) -> dict:
    """한 주행 스텝. 기본값은 motion_tune 의 수렴 파라미터, overrides 로 스텝별 조정."""
    base = dict(DEFAULT_PARAMS[direction])
    if direction in {"cw", "ccw"}:
        base.update(
            speed=ROT_SPEED,
            boost_speed=ROT_BOOST_SPEED,
            boost_ms=ROT_BOOST_MS,
            brake_ms=ROT_BRAKE_MS,
            brake_scale=ROT_BRAKE_SCALE,
        )
    step = {
        "direction": direction,
        "magnitude": float(magnitude),         # cm(거리) 또는 deg(각도)
        "scales": list(DEFAULT_SCALES[direction]),
        **base,
    }
    step.update(overrides)
    return step


# ============================ 경기 동선 (ROUTE) ============================
# 최초 두 스텝은 하드코딩된 오프닝. right(횡이동)은 튜닝 시 boost/brake 가 0으로 꺼져
# 있었으므로(정지마찰이 큰 방향), 여기서 boost/brake 를 명시적으로 넣어준다.
ROUTE = [
    make_step("forward", 30),                          # ① 전진 30 cm (boost 100ms + brake 150ms 기본 내장)
    make_step("right", 30, boost_ms=120, brake_ms=120, brake_scale=0.35),  # ② 우측 횡이동 30 cm (+boost/brake)
    # --- 이후 동선은 여기에 추가/주석해제. 회전은 자동으로 느린 프로파일 ---
    # make_step("cw", 90),
    # make_step("ccw", 90),
    # make_step("forward", 30),
]


def resolve_duration_ms(step: dict) -> int:
    """스텝의 duration_ms. 명시값 우선, 없으면 seed(회전은 저속 스케일)로 산출."""
    if step.get("duration_ms"):
        return int(step["duration_ms"])
    d, mag = step["direction"], step["magnitude"]
    return rotation_duration_ms(d, mag) if d in {"cw", "ccw"} else duration_seed(d, mag)


def describe_step(idx: int, step: dict) -> str:
    d = step["direction"]
    unit = "deg" if KINDS[d] == "angle" else "cm"
    dur = resolve_duration_ms(step)
    steady = wheel_cmd(d, step["speed"], step["scales"])
    boost = wheel_cmd(d, step["boost_speed"], step["scales"])
    boost_txt = (f"boost {step['boost_speed']:.2f}x{step['boost_ms']}ms"
                 if step["boost_ms"] > 0 else "boost —OFF—")
    brake_txt = (f"brake {step['brake_scale']:.2f}x{step['brake_ms']}ms"
                 if step["brake_ms"] > 0 else "brake —OFF—")
    warn = ""
    if step["boost_ms"] <= 0:
        warn += "  ⚠ no boost"
    if step["brake_ms"] <= 0:
        warn += "  ⚠ no brake"
    return (
        f"[{idx}] {d:<8} {step['magnitude']:>5.0f}{unit}  dur={dur:>4}ms  "
        f"speed={step['speed']:.2f}  {boost_txt}  {brake_txt}{warn}\n"
        f"      steady FL={steady[0]:+.2f} FR={steady[1]:+.2f} RL={steady[2]:+.2f} RR={steady[3]:+.2f}"
        f"   boost FL={boost[0]:+.2f} FR={boost[1]:+.2f} RL={boost[2]:+.2f} RR={boost[3]:+.2f}"
    )


def run_step(ser: serial.Serial, step: dict, hz: float) -> None:
    d = step["direction"]
    dur = resolve_duration_ms(step)
    steady = wheel_cmd(d, step["speed"], step["scales"])
    boost = wheel_cmd(d, step["boost_speed"], step["scales"])
    run_continuous(
        ser,
        steady_cmd=steady,
        boost_cmd=boost,
        duration_ms=dur,
        boost_ms=int(step["boost_ms"]),
        brake_ms=int(step["brake_ms"]),
        brake_scale=float(step["brake_scale"]),
        hz=hz,
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="모의 경기 주행 파이프라인 (하드코딩 오프닝 + boost/brake)")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--settle", type=float, default=0.6,
                    help="스텝 사이 정지 대기(초). 무거운 베이스의 관성이 잦아든 뒤 다음 스텝")
    ap.add_argument("--dry-run", action="store_true",
                    help="시리얼 없이 동선/부스터/브레이크 구성만 출력해 확인")
    ap.add_argument("--yes", action="store_true", help="시작 확인 프롬프트 건너뜀")
    args = ap.parse_args()

    print("=== 모의 경기 파이프라인 동선 ===")
    print("시작: 4x4m 맵 우측 하단, 정면 +x = 필드 안쪽")
    for i, step in enumerate(ROUTE, 1):
        print(describe_step(i, step))
    print("=" * 34)

    if args.dry_run:
        print("dry-run: 하드웨어 미구동. 위 구성 확인 후 --dry-run 없이 실행.")
        return

    if not args.yes:
        try:
            if input("경로가 안전한지 확인 후 Enter(실행) / q(취소): ").strip().lower() in {"q", "quit", "exit"}:
                print("취소됨.")
                return
        except EOFError:
            print("비대화형 입력 — --yes 로 실행하세요. 취소됨.")
            return

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except (serial.SerialException, OSError) as e:
        print(f"port open failed: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"opened {args.port} @ {args.baud}")
    print("waiting for MCU reboot...")
    time.sleep(2.0)
    ser.reset_input_buffer()
    stop_base(ser)

    try:
        for i, step in enumerate(ROUTE, 1):
            print(f"\n>>> 스텝 {i}/{len(ROUTE)}: {step['direction']} {step['magnitude']:g}")
            run_step(ser, step, args.hz)
            if i < len(ROUTE) and args.settle > 0:
                stop_base(ser)
                time.sleep(args.settle)       # 관성 소산 대기
        print("\n동선 완료.")
    except KeyboardInterrupt:
        print("\n중단됨 (Ctrl-C).")
    finally:
        for _ in range(5):
            stop_base(ser)
            time.sleep(0.02)
        ser.flush()
        ser.close()
        print("stopped and closed")


if __name__ == "__main__":
    main()
