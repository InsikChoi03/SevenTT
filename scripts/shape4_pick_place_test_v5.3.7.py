#!/usr/bin/env python3
"""그리퍼 45도, REACH/LIFT 1.5초의 독립 전체 집기 테스트.

v5.3.4의 Enter 기반 GRASP 확인 흐름을 유지한다. 메인 경기 코드와 설정은
변경하지 않는다. 1.5초는 현재 펌웨어의 66.7deg/s 제한상 145도 이동의 물리
최소시간(약 2.18초)보다 짧으므로 비교 실험용이다.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
MANUAL_TEST = ROOT / "scripts/shape4_pick_place_test_v5.3.4.py"
GRIP_CLOSED = 45.0
FAST_MOVE_SEC = 1.5


def load_manual_module():
    spec = importlib.util.spec_from_file_location("shape4_pick_place_manual_fast", MANUAL_TEST)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"수동 GRASP 테스트를 불러올 수 없습니다: {MANUAL_TEST}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def fast_steps(base, config):
    ish, iwr, _ = config.init.values()
    psh, pwr = config.pick_sw
    plsh, plwr = config.place_sw
    requested = [
        ("PREOPEN", 0.20, base.Pose(ish, iwr, config.grip_open)),
        ("REACH", FAST_MOVE_SEC, base.Pose(psh, pwr, config.grip_open)),
        ("GRASP", 1.20, base.Pose(psh, pwr, config.grip_closed)),
        ("LIFT", FAST_MOVE_SEC, base.Pose(ish, iwr, config.grip_closed)),
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="통합 MCU 직렬 포트")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--delay", type=float, default=3.0, help="INIT 안착 후 집기 전 대기시간")
    parser.add_argument("--dry-run", action="store_true", help="1.5초 프로파일만 출력")
    args = parser.parse_args()
    if args.delay < 0.0:
        parser.error("--delay must be >= 0")
    return args


def main() -> int:
    args = parse_args()
    try:
        manual = load_manual_module()
        physical = manual.load_physical_module()
        previous = physical.load_previous_module()
        profile = previous.load_profile_module()
        base = profile.load_base_module()
        config = replace(profile.v5_0_config(base), grip_closed=GRIP_CLOSED)
        steps = fast_steps(base, config)
    except (OSError, RuntimeError) as exc:
        print(f"테스트 로드 오류: {exc}", file=sys.stderr)
        return 2

    print("독립 프로파일: manual GRASP + grip 45deg + REACH/LIFT 1.50s")
    print("메인 motion_tuning.yaml과 pick_sequencer_node는 사용하거나 수정하지 않습니다.")
    base.describe(config, steps, Path("embedded:v5.3.4+grip45+fast1.5"))
    print("경고: 145도/1.5초=96.7deg/s로 펌웨어 상한 66.7deg/s보다 빠릅니다.")
    print("따라서 1.5초 종료 시 실제 관절은 목표에 아직 도달하지 않았을 수 있습니다.")
    if args.dry_run:
        print("DRY RUN: 하드웨어 명령을 보내지 않았습니다.")
        return 0

    print("주의: 메인 경기를 종료하고 물체를 기존 정면 집기점에 두세요.")
    arm = base.SerialArm(args.port, args.baud)
    try:
        arm.open()
        manual.run_manual_sequence(physical, previous, base, arm, config, steps, args.delay)
    except manual.TestAborted as exc:
        print(f"\n테스트 중단: {exc}", file=sys.stderr)
        return 3
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
