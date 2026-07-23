#!/usr/bin/env python3
"""GRASP 완료를 사람이 확인해야만 LIFT하는 독립 집기 테스트.

메인 코드는 변경하지 않는다. REACH는 물리 속도 보정값 2.5초로 수행하고, 최하점에
도달하면 그리퍼 50° 목표를 즉시 보낸다. 그 뒤에는 시간으로 LIFT하지 않고 사용자가
실제 닫힘을 확인해 Enter를 누를 때까지 [25,150,50] 목표를 유지한다. GRASP 명령부터
Enter까지의 실측 시간을 출력한다.
"""
from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import select
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
PHYSICAL_TEST = ROOT / "scripts/shape4_pick_place_test_v5.3.3.py"


class TestAborted(Exception):
    """사용자가 수동 확인 단계에서 테스트를 중단했다."""


def load_physical_module():
    spec = importlib.util.spec_from_file_location("shape4_pick_place_physical", PHYSICAL_TEST)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"물리 시간 테스트를 불러올 수 없습니다: {PHYSICAL_TEST}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def confirm_grasp_complete(arm, target) -> float:
    print(
        "[GRASP] 닫힘 최종 목표를 즉시 전송했습니다. "
        "팔 높이는 [25,150]에 고정되고 자동 LIFT는 차단됩니다."
    )
    arm.pose(target)
    started = time.monotonic()
    print(
        "그리퍼가 실제로 완전히 닫혀 물체를 잡은 것을 확인한 뒤 Enter "
        "(q=상승 없이 중단): ",
        end="",
        flush=True,
    )
    # 입력 대기 중에도 같은 최하점/닫힘 목표를 재전송하고 MCU 텔레메트리를 비운다.
    # 단순 input()으로 막으면 오래 기다릴 때 직렬 수신 버퍼가 차 MCU loop에 영향을 줄 수 있다.
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0.05)
        arm.pose(target)
        arm.drain()
        if not ready:
            continue
        line = sys.stdin.readline()
        if line == "":
            raise TestAborted("입력 터미널이 없어 LIFT를 차단했습니다")
        answer = line.strip().lower()
        break
    elapsed = time.monotonic() - started
    if answer == "q":
        raise TestAborted("사용자가 GRASP 확인 단계에서 중단했습니다")
    arm.pose(target)
    print(f"[GRASP CONFIRMED] 실측 {elapsed:.2f}초 -> 지금 LIFT를 시작합니다.")
    print(f"자동화 후보 시간: {elapsed + 0.20:.2f}초 (실측 + 0.20초 여유)")
    return elapsed


def run_manual_sequence(physical, previous, base, arm, config, steps, delay: float) -> float:
    arm.pose(config.init)
    time.sleep(1.5)
    physical.countdown(delay)

    print("PICK_PLACE 시작: REACH -> MANUAL GRASP CONFIRM -> LIFT")
    grasp_elapsed = 0.0
    for step in steps:
        if step.name == "GRASP":
            grasp_elapsed = confirm_grasp_complete(arm, step.target)
            continue
        previous.run_ramped_step(base, arm, step)
    print(f"PICK_PLACE 완료: 수동 확인 GRASP 시간={grasp_elapsed:.2f}초")
    return grasp_elapsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="통합 MCU 직렬 포트")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--delay", type=float, default=3.0, help="INIT 안착 후 집기 전 대기시간")
    parser.add_argument("--dry-run", action="store_true", help="수동 확인 흐름만 출력")
    args = parser.parse_args()
    if args.delay < 0.0:
        parser.error("--delay must be >= 0")
    return args


def main() -> int:
    args = parse_args()
    try:
        physical = load_physical_module()
        previous = physical.load_previous_module()
        profile = previous.load_profile_module()
        base = profile.load_base_module()
        config = profile.v5_0_config(base)
        steps = physical.physical_timing_steps(base, config)
    except (OSError, RuntimeError) as exc:
        print(f"테스트 로드 오류: {exc}", file=sys.stderr)
        return 2

    print("독립 프로파일: physical REACH + manual GRASP confirmation")
    print("메인 motion_tuning.yaml과 pick_sequencer_node는 사용하거나 수정하지 않습니다.")
    base.describe(config, steps, Path("embedded:manual-grasp-confirm"))
    print("GRASP의 표시된 1.20초는 사용하지 않습니다: Enter 전까지 LIFT가 시작되지 않습니다.")
    if args.dry_run:
        print("DRY RUN: 하드웨어 명령을 보내지 않았습니다.")
        return 0

    print("주의: 메인 경기를 종료하고 물체를 기존 정면 집기점에 두세요.")
    arm = base.SerialArm(args.port, args.baud)
    try:
        arm.open()
        run_manual_sequence(physical, previous, base, arm, config, steps, args.delay)
    except TestAborted as exc:
        print(f"\n테스트 중단: {exc}", file=sys.stderr)
        return 3
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
