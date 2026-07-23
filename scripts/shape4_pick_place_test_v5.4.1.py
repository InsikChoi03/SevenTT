#!/usr/bin/env python3
"""어깨 20%, 손목/그리퍼 30%용 자동 집기→놓기 독립 테스트.

REACH가 끝나면 [25,150,45]를 즉시 명령하고 0.75초 동안 최하점에서 유지한
뒤 Enter 입력 없이 자동으로 LIFT한다. 메인 경기 코드와 설정은 변경하지 않는다.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import importlib.util
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
GRASP_FIRST_TEST = ROOT / "scripts/shape4_pick_place_test_v5.3.2.py"
RATE_HZ = 20.0
MOVE_SEC = 1.30
GRASP_SEC = 0.75
GRIP_CLOSED = 45.0


def load_grasp_first_module():
    spec = importlib.util.spec_from_file_location("shape4_grasp_first_auto30", GRASP_FIRST_TEST)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"즉시 GRASP 테스트를 불러올 수 없습니다: {GRASP_FIRST_TEST}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def automatic_steps(base, config):
    ish, iwr, _ = config.init.values()
    psh, pwr = config.pick_sw
    plsh, plwr = config.place_sw
    requested = [
        ("PREOPEN", 0.20, base.Pose(ish, iwr, config.grip_open)),
        ("REACH", MOVE_SEC, base.Pose(psh, pwr, config.grip_open)),
        ("GRASP", GRASP_SEC, base.Pose(psh, pwr, config.grip_closed)),
        ("LIFT", MOVE_SEC, base.Pose(ish, iwr, config.grip_closed)),
        ("TO_PLACE", 1.25, base.Pose(plsh, plwr, config.grip_closed)),
        ("PLACE", GRASP_SEC, base.Pose(plsh, plwr, config.grip_open)),
        ("STOW", 1.00, config.init),
    ]
    steps = []
    current = config.init
    for name, duration, target in requested:
        steps.append(base.Step(name, duration, current, target))
        current = target
    return steps


def run_automatic(grasp_first, base, arm, config, steps, delay: float) -> None:
    arm.pose(config.init)
    time.sleep(1.5)
    grasp_first.countdown(delay)

    print("PICK_PLACE 시작: REACH -> AUTO GRASP 0.75s -> AUTO LIFT")
    period = 1.0 / RATE_HZ
    for step in steps:
        if step.name != "GRASP":
            grasp_first.run_ramped_step(base, arm, step)
            continue

        print(
            f"[GRASP] immediate target {tuple(round(v) for v in step.start.values())} -> "
            f"{tuple(round(v) for v in step.target.values())}; "
            f"hold pick height ({step.duration:.2f}s)"
        )
        arm.pose(step.target)
        started = time.monotonic()
        while time.monotonic() - started < step.duration:
            arm.pose(step.target)
            arm.drain()
            time.sleep(period)
        arm.pose(step.target)
        print(f"[GRASP COMPLETE] {step.duration:.2f}s finished -> automatic LIFT")

    print("PICK_PLACE 완료: 자동 집기→상승→놓기→대기 자세 복귀")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="통합 MCU 직렬 포트")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--delay", type=float, default=3.0, help="INIT 안착 후 집기 전 대기시간")
    parser.add_argument("--dry-run", action="store_true", help="자동 시퀀스만 출력")
    args = parser.parse_args()
    if args.delay < 0.0:
        parser.error("--delay must be >= 0")
    return args


def main() -> int:
    args = parse_args()
    try:
        grasp_first = load_grasp_first_module()
        profile = grasp_first.load_profile_module()
        base = profile.load_base_module()
        config = replace(
            profile.v5_0_config(base),
            grip_closed=GRIP_CLOSED,
            grasp_sec=GRASP_SEC,
        )
        steps = automatic_steps(base, config)
    except (OSError, RuntimeError) as exc:
        print(f"테스트 로드 오류: {exc}", file=sys.stderr)
        return 2

    print("독립 프로파일: shoulder 20% + wrist/gripper 30% + automatic LIFT")
    print("필수 펌웨어: MAX_STEP[3] = {2.4, 3.6, 3.6}")
    print("메인 motion_tuning.yaml과 pick_sequencer_node는 사용하거나 수정하지 않습니다.")
    base.describe(config, steps, Path("embedded:v5.4.1+automatic-grasp"))
    print("GRASP 118->45도: 이론 0.61초, 명령 유지 0.75초 후 자동 LIFT")
    if args.dry_run:
        print("DRY RUN: 하드웨어 명령을 보내지 않았습니다.")
        return 0

    print("주의: 새 속도 펌웨어를 먼저 플래시하고 메인 경기를 종료하세요.")
    print("첫 실행은 물체 없이 하고, Enter를 기다리지 않으므로 팔에서 손을 치우세요.")
    arm = base.SerialArm(args.port, args.baud)
    try:
        arm.open()
        run_automatic(grasp_first, base, arm, config, steps, args.delay)
    except KeyboardInterrupt:
        print("\n사용자 중단: 베이스 정지 및 STANDBY를 전송합니다.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"실행 오류: {exc}", file=sys.stderr)
        return 1
    finally:
        arm.safe_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
