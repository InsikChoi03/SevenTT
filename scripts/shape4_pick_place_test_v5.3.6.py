#!/usr/bin/env python3
"""v5.3.4 수동 GRASP 확인 흐름에서 닫힘 각도만 45도로 조정한 독립 테스트.

메인 경기 코드와 설정은 변경하지 않는다. REACH/LIFT 2.5초와 Enter 기반 LIFT
허용 방식을 그대로 유지하고, GRASP 목표만 [25,150,45]로 바꾼다.
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


def load_manual_module():
    spec = importlib.util.spec_from_file_location("shape4_pick_place_manual", MANUAL_TEST)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"수동 GRASP 테스트를 불러올 수 없습니다: {MANUAL_TEST}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="통합 MCU 직렬 포트")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--delay", type=float, default=3.0, help="INIT 안착 후 집기 전 대기시간")
    parser.add_argument("--dry-run", action="store_true", help="45도 프로파일만 출력")
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
        steps = physical.physical_timing_steps(base, config)
    except (OSError, RuntimeError) as exc:
        print(f"테스트 로드 오류: {exc}", file=sys.stderr)
        return 2

    print("독립 프로파일: v5.3.4 manual confirmation + gripper closed 45deg")
    print("메인 motion_tuning.yaml과 pick_sequencer_node는 사용하거나 수정하지 않습니다.")
    base.describe(config, steps, Path("embedded:v5.3.4+grip45"))
    print("REACH/LIFT는 각각 2.50초이며, GRASP 확인 전 자동 LIFT는 차단됩니다.")
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
