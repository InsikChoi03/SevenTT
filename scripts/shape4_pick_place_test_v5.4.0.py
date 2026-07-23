#!/usr/bin/env python3
"""관절별 25/30% 펌웨어 속도용 1.3초 독립 전체 집기 테스트.

v5.3.4의 Enter 기반 GRASP 확인과 그리퍼 45도 설정을 유지한다. 펌웨어의
어깨 100deg/s, 손목 120deg/s 제한을 기준으로 145도 손목 이동에 필요한
1.21초보다 약간 여유 있는 1.30초를 REACH/LIFT에 사용한다.
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
MOVE_SEC = 1.30


def load_manual_module():
    spec = importlib.util.spec_from_file_location("shape4_pick_place_manual_30pct", MANUAL_TEST)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"수동 GRASP 테스트를 불러올 수 없습니다: {MANUAL_TEST}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def tuned_steps(base, config):
    ish, iwr, _ = config.init.values()
    psh, pwr = config.pick_sw
    plsh, plwr = config.place_sw
    requested = [
        ("PREOPEN", 0.20, base.Pose(ish, iwr, config.grip_open)),
        ("REACH", MOVE_SEC, base.Pose(psh, pwr, config.grip_open)),
        ("GRASP", 1.20, base.Pose(psh, pwr, config.grip_closed)),
        ("LIFT", MOVE_SEC, base.Pose(ish, iwr, config.grip_closed)),
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
    parser.add_argument("--dry-run", action="store_true", help="30% 속도 프로파일만 출력")
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
        steps = tuned_steps(base, config)
    except (OSError, RuntimeError) as exc:
        print(f"테스트 로드 오류: {exc}", file=sys.stderr)
        return 2

    print("독립 프로파일: shoulder 25% + wrist 30% + grip 45deg")
    print("필수 펌웨어: MAX_STEP[3] = {3.0, 3.6, 2.0}")
    print("메인 motion_tuning.yaml과 pick_sequencer_node는 사용하거나 수정하지 않습니다.")
    base.describe(config, steps, Path("embedded:v5.4.0+joint-speed-limits"))
    print("REACH/LIFT 1.30초: 손목 145도/120deg/s=1.21초보다 0.09초 여유")
    if args.dry_run:
        print("DRY RUN: 하드웨어 명령을 보내지 않았습니다.")
        return 0

    print("주의: 속도 변경 펌웨어를 먼저 플래시하고 메인 경기를 종료하세요.")
    print("첫 실행은 물체 없이 하고, 팔 주변에서 손을 치우세요.")
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
