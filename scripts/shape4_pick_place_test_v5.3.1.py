#!/usr/bin/env python3
"""v5.0.0 메인 집기 프로파일을 재현하는 독립 집기→놓기 하드웨어 테스트.

현재 메인 설정과 ROS 노드는 건드리지 않는다. 안전한 직렬 연결·종료 처리는 v5.3.0
독립 테스트에서 재사용하고, 팔 단계와 각도·시간만 릴리스 커밋 50bcc07의 v5.0.0
유효 motion_tuning 값으로 고정한다.

v5.0.0 순서:
    PREOPEN 0.20s -> REACH 1.25s -> GRASP 1.00s -> LIFT 1.25s
    -> TO_PLACE 1.25s -> PLACE 1.00s -> STOW 1.00s

PICK_SETTLE과 이동시간 자동 연장은 의도적으로 사용하지 않는다.
"""
from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
BASE_TEST = ROOT / "scripts/shape4_pick_place_test_v5.3.0.py"
SOURCE_REVISION = "50bcc07 (v5.0.0)"


def load_base_module():
    spec = importlib.util.spec_from_file_location("shape4_pick_place_test_base", BASE_TEST)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"기반 테스트를 불러올 수 없습니다: {BASE_TEST}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def v5_0_config(base):
    return base.PickConfig(
        init=base.Pose(120.0, 5.0, 100.0),
        pick_sw=(25.0, 150.0),
        place_sw=(110.0, 30.0),
        grip_open=118.0,
        grip_closed=50.0,
        move_sec=1.25,
        grasp_sec=1.0,
        preopen_sec=0.2,
        # PickConfig 형식을 재사용하되 v5.0.0 단계에는 아래 두 값이 사용되지 않는다.
        pick_settle_sec=0.25,
        servo_speed_dps=60.0,
    )


def v5_0_steps(base, config):
    ish, iwr, _ = config.init.values()
    psh, pwr = config.pick_sw
    plsh, plwr = config.place_sw
    requested = [
        ("PREOPEN", 0.20, base.Pose(ish, iwr, config.grip_open)),
        ("REACH", 1.25, base.Pose(psh, pwr, config.grip_open)),
        ("GRASP", 1.00, base.Pose(psh, pwr, config.grip_closed)),
        ("LIFT", 1.25, base.Pose(ish, iwr, config.grip_closed)),
        ("TO_PLACE", 1.25, base.Pose(plsh, plwr, config.grip_closed)),
        ("PLACE", 1.00, base.Pose(plsh, plwr, config.grip_open)),
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
    parser.add_argument("--dry-run", action="store_true", help="고정 프로파일만 출력")
    args = parser.parse_args()
    if args.delay < 0.0:
        parser.error("--delay must be >= 0")
    return args


def main() -> int:
    args = parse_args()
    try:
        base = load_base_module()
        config = v5_0_config(base)
        steps = v5_0_steps(base, config)
    except (OSError, RuntimeError) as exc:
        print(f"테스트 로드 오류: {exc}", file=sys.stderr)
        return 2

    print(f"고정 집기 프로파일: {SOURCE_REVISION}")
    print("현재 motion_tuning.yaml과 메인 pick_sequencer_node는 사용하거나 수정하지 않습니다.")
    base.describe(config, steps, Path(f"embedded:{SOURCE_REVISION}"))
    if args.dry_run:
        print("DRY RUN: 하드웨어 명령을 보내지 않았습니다.")
        return 0

    print("주의: 메인 경기를 종료하고 물체를 기존 정면 집기점에 두세요.")
    print("v5.0.0처럼 1.25초 하강 뒤 정착 대기 없이 그리퍼를 닫기 시작합니다.")
    arm = base.SerialArm(args.port, args.baud)
    try:
        arm.open()
        base.run_sequence(arm, config, steps, args.delay)
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
