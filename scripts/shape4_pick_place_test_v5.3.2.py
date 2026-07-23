#!/usr/bin/env python3
"""GRASP 물리 완료 뒤에만 LIFT하는 독립 집기→놓기 테스트.

메인 코드는 변경하지 않는다. 포즈와 전체 단계는 v5.0.0 독립 프로파일을 유지하되,
GRASP만 다음처럼 실행한다.

  1. REACH 보간을 완료하고 최하점 [25,150,118]을 전송한다.
  2. 닫힘 최종 목표 [25,150,50]을 즉시 전송한다.
  3. 어깨/손목을 최하점에 고정한 채 같은 닫힘 목표를 1초간 계속 전송한다.
  4. 1초가 끝난 뒤에만 LIFT를 시작한다.

3번은 닫힌 뒤 추가로 쉬는 시간이 아니라 MCU가 실제 GRASP를 수행하는 시간이다.
"""
from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
PROFILE_TEST = ROOT / "scripts/shape4_pick_place_test_v5.3.1.py"
RATE_HZ = 20.0


def load_profile_module():
    spec = importlib.util.spec_from_file_location("shape4_pick_place_v5_0_profile", PROFILE_TEST)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"v5.0.0 프로파일을 불러올 수 없습니다: {PROFILE_TEST}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def countdown(delay: float) -> None:
    for remaining in range(int(delay), 0, -1):
        print(f"집기 시작까지 {remaining}초 — 손을 치우세요.")
        time.sleep(1.0)
    fractional = delay - int(delay)
    if fractional > 0.0:
        time.sleep(fractional)


def run_ramped_step(base, arm, step) -> None:
    print(
        f"[{step.name}] {tuple(round(v) for v in step.start.values())} -> "
        f"{tuple(round(v) for v in step.target.values())} ({step.duration:.2f}s ramp)"
    )
    started = time.monotonic()
    period = 1.0 / RATE_HZ
    while True:
        elapsed = time.monotonic() - started
        if elapsed >= step.duration:
            break
        arm.pose(base.lerp(step.start, step.target, elapsed / step.duration))
        arm.drain()
        time.sleep(period)
    arm.pose(step.target)


def run_grasp_then_lift(base, arm, config, steps, delay: float) -> None:
    arm.pose(config.init)
    time.sleep(1.5)
    countdown(delay)

    print("PICK_PLACE 시작: GRASP 물리 수행이 끝난 뒤 LIFT")
    period = 1.0 / RATE_HZ
    for step in steps:
        if step.name != "GRASP":
            run_ramped_step(base, arm, step)
            continue

        # REACH의 최종 어깨/손목을 바꾸지 않고 닫힘 최종각을 즉시 지시한다. 기존처럼
        # 118 -> 50 목표 자체를 1초간 host-ramp하지 않으므로 MCU의 물리 보간이 즉시 시작된다.
        print(
            f"[GRASP] immediate target {tuple(round(v) for v in step.start.values())} -> "
            f"{tuple(round(v) for v in step.target.values())}; "
            f"hold pick height while closing ({step.duration:.2f}s)"
        )
        arm.pose(step.target)
        started = time.monotonic()
        while time.monotonic() - started < step.duration:
            arm.pose(step.target)
            arm.drain()
            time.sleep(period)
        arm.pose(step.target)
        print("[GRASP COMPLETE] 1.00s command window finished -> LIFT allowed")

    print("PICK_PLACE 완료: 집기→놓기→대기 자세 복귀")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="통합 MCU 직렬 포트")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--delay", type=float, default=3.0, help="INIT 안착 후 집기 전 대기시간")
    parser.add_argument("--dry-run", action="store_true", help="시퀀스와 GRASP 정책만 출력")
    args = parser.parse_args()
    if args.delay < 0.0:
        parser.error("--delay must be >= 0")
    return args


def main() -> int:
    args = parse_args()
    try:
        profile = load_profile_module()
        base = profile.load_base_module()
        config = profile.v5_0_config(base)
        steps = profile.v5_0_steps(base, config)
    except (OSError, RuntimeError) as exc:
        print(f"테스트 로드 오류: {exc}", file=sys.stderr)
        return 2

    print("독립 프로파일: v5.0.0 pose/timing + immediate GRASP target")
    print("메인 motion_tuning.yaml과 pick_sequencer_node는 사용하거나 수정하지 않습니다.")
    base.describe(config, steps, Path("embedded:v5.0.0+grasp-before-lift"))
    print("GRASP 정책: 50도 즉시 명령 → 최하점에서 1.00초 실제 닫힘 수행 → LIFT")
    if args.dry_run:
        print("DRY RUN: 하드웨어 명령을 보내지 않았습니다.")
        return 0

    print("주의: 메인 경기를 종료하고 물체를 기존 정면 집기점에 두세요.")
    arm = base.SerialArm(args.port, args.baud)
    try:
        arm.open()
        run_grasp_then_lift(base, arm, config, steps, args.delay)
    except KeyboardInterrupt:
        print("\n사용자 중단: 베이스 정지 및 STANDBY를 전송합니다.", file=sys.stderr)
        return 130
    except Exception as exc:  # 직렬 오류에도 finally의 안전 종료를 실행한다.
        print(f"실행 오류: {exc}", file=sys.stderr)
        return 1
    finally:
        arm.safe_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
