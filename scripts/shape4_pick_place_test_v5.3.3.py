#!/usr/bin/env python3
"""물리 서보 속도로 REACH/GRASP/LIFT 시간을 보정한 독립 집기 테스트.

현재 메인 코드는 변경하지 않는다. v5.0.0 포즈를 사용하되 통합 MCU의 약 67°/s
제한보다 느린 60°/s를 기준으로 큰 관절 이동을 2.5초에 보간한다. REACH가 끝나면
그리퍼 50° 최종 목표를 즉시 보내 1.2초 동안 실제 닫힘을 수행한 뒤 LIFT한다.
"""
from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
PREVIOUS_TEST = ROOT / "scripts/shape4_pick_place_test_v5.3.2.py"
RATE_HZ = 20.0


def load_previous_module():
    spec = importlib.util.spec_from_file_location("shape4_pick_place_grasp_first", PREVIOUS_TEST)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"이전 독립 테스트를 불러올 수 없습니다: {PREVIOUS_TEST}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def physical_timing_steps(base, config):
    ish, iwr, _ = config.init.values()
    psh, pwr = config.pick_sw
    plsh, plwr = config.place_sw
    requested = [
        ("PREOPEN", 0.20, base.Pose(ish, iwr, config.grip_open)),
        ("REACH", 2.50, base.Pose(psh, pwr, config.grip_open)),
        ("GRASP", 1.20, base.Pose(psh, pwr, config.grip_closed)),
        ("LIFT", 2.50, base.Pose(ish, iwr, config.grip_closed)),
        ("TO_PLACE", 1.25, base.Pose(plsh, plwr, config.grip_closed)),
        ("PLACE", 1.20, base.Pose(plsh, plwr, config.grip_open)),
        ("STOW", 1.00, config.init),
    ]
    steps = []
    current = config.init
    for name, duration, target in requested:
        steps.append(base.Step(name, duration, current, target))
        current = target
    return steps


def countdown(delay: float) -> None:
    for remaining in range(int(delay), 0, -1):
        print(f"집기 시작까지 {remaining}초 — 손을 치우세요.")
        time.sleep(1.0)
    fractional = delay - int(delay)
    if fractional > 0.0:
        time.sleep(fractional)


def run_sequence(previous, base, arm, config, steps, delay: float) -> None:
    arm.pose(config.init)
    time.sleep(1.5)
    countdown(delay)

    print("PICK_PLACE 시작: physical timing REACH -> GRASP -> LIFT")
    period = 1.0 / RATE_HZ
    for step in steps:
        if step.name != "GRASP":
            previous.run_ramped_step(base, arm, step)
            continue

        print(
            f"[GRASP] immediate target {tuple(round(v) for v in step.start.values())} -> "
            f"{tuple(round(v) for v in step.target.values())}; "
            f"pick height fixed for physical close ({step.duration:.2f}s)"
        )
        arm.pose(step.target)
        started = time.monotonic()
        while time.monotonic() - started < step.duration:
            arm.pose(step.target)
            arm.drain()
            time.sleep(period)
        arm.pose(step.target)
        print(f"[GRASP COMPLETE] {step.duration:.2f}s finished -> LIFT allowed")

    print("PICK_PLACE 완료: 집기→놓기→대기 자세 복귀")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="통합 MCU 직렬 포트")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--delay", type=float, default=3.0, help="INIT 안착 후 집기 전 대기시간")
    parser.add_argument("--dry-run", action="store_true", help="보정된 단계와 시간만 출력")
    args = parser.parse_args()
    if args.delay < 0.0:
        parser.error("--delay must be >= 0")
    return args


def main() -> int:
    args = parse_args()
    try:
        previous = load_previous_module()
        profile = previous.load_profile_module()
        base = profile.load_base_module()
        config = profile.v5_0_config(base)
        steps = physical_timing_steps(base, config)
    except (OSError, RuntimeError) as exc:
        print(f"테스트 로드 오류: {exc}", file=sys.stderr)
        return 2

    print("독립 프로파일: v5.0.0 pose + physical servo timing")
    print("메인 motion_tuning.yaml과 pick_sequencer_node는 사용하거나 수정하지 않습니다.")
    base.describe(config, steps, Path("embedded:v5.0.0+physical-timing"))
    print("핵심: REACH 2.50s → GRASP 즉시 50°/1.20s → LIFT 2.50s")
    if args.dry_run:
        print("DRY RUN: 하드웨어 명령을 보내지 않았습니다.")
        return 0

    print("주의: 메인 경기를 종료하고 물체를 기존 정면 집기점에 두세요.")
    arm = base.SerialArm(args.port, args.baud)
    try:
        arm.open()
        run_sequence(previous, base, arm, config, steps, args.delay)
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
