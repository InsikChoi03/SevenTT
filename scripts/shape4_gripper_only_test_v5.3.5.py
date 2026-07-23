#!/usr/bin/env python3
"""어깨/손목을 집기점에 고정하고 그리퍼만 닫는 독립 진단 테스트.

메인 경기 코드는 사용하거나 수정하지 않는다. 팔을 안전한 INIT 자세에서
[25,150,118]로 천천히 이동한 다음 사용자 Enter를 기다린다. Enter 이후에는
[25,150,50]만 반복 전송하며, 어떤 경우에도 LIFT 자세를 명령하지 않는다.
"""
from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import select
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
BASE_TEST = ROOT / "scripts/shape4_pick_place_test_v5.3.0.py"
RATE_HZ = 20.0
INIT_POSE = (120.0, 5.0, 100.0)
FIXED_OPEN_POSE = (25.0, 150.0, 118.0)
FIXED_CLOSED_POSE = (25.0, 150.0, 50.0)


class TestAborted(Exception):
    """사용자가 진단을 중단했다."""


def load_base_module():
    spec = importlib.util.spec_from_file_location("shape4_pick_place_base", BASE_TEST)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"기본 팔 테스트를 불러올 수 없습니다: {BASE_TEST}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def pose(base, values: tuple[float, float, float]):
    return base.Pose(*values)


def send_for(arm, target, duration: float) -> None:
    period = 1.0 / RATE_HZ
    started = time.monotonic()
    while time.monotonic() - started < duration:
        arm.pose(target)
        arm.drain()
        time.sleep(period)
    arm.pose(target)


def ramp_to(base, arm, start, target, duration: float) -> None:
    period = 1.0 / RATE_HZ
    print(
        f"[REACH] {tuple(round(v) for v in start.values())} -> "
        f"{tuple(round(v) for v in target.values())} ({duration:.2f}s)"
    )
    started = time.monotonic()
    while True:
        elapsed = time.monotonic() - started
        if elapsed >= duration:
            break
        arm.pose(base.lerp(start, target, elapsed / duration))
        arm.drain()
        time.sleep(period)
    arm.pose(target)


def prompt_while_holding(arm, target, message: str) -> str:
    print(message, end="", flush=True)
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0.05)
        arm.pose(target)
        arm.drain()
        if not ready:
            continue
        line = sys.stdin.readline()
        if line == "":
            raise TestAborted("입력 터미널이 없어 안전하게 중단합니다")
        return line.strip().lower()


def print_serial_lines(arm) -> None:
    if arm.ser is None:
        return
    while arm.ser.in_waiting:
        raw = arm.ser.readline()
        if not raw:
            break
        line = raw.decode("ascii", errors="replace").strip()
        if line:
            print(f"  MCU {line}")


def close_until_confirmed(arm, target) -> float:
    print("[GRASP] 송신 시작: <ARM,25,150,50>")
    print("어깨=25, 손목=150을 유지하고 세 번째 그리퍼 채널만 50으로 닫습니다.")
    print("ARMACK은 실제 서보각이 아니라 MCU가 받은 목표값의 echo입니다.")
    print("관찰이 끝나면 Enter, 즉시 중단은 q: ", end="", flush=True)
    started = time.monotonic()
    next_report = started
    period = 1.0 / RATE_HZ
    while True:
        arm.pose(target)
        now = time.monotonic()
        if now >= next_report:
            print_serial_lines(arm)
            next_report = now + 0.20
        ready, _, _ = select.select([sys.stdin], [], [], period)
        if not ready:
            continue
        line = sys.stdin.readline()
        if line == "":
            raise TestAborted("입력 터미널이 없어 안전하게 중단합니다")
        answer = line.strip().lower()
        elapsed = time.monotonic() - started
        if answer == "q":
            raise TestAborted(f"사용자가 GRASP {elapsed:.2f}초에 중단했습니다")
        arm.pose(target)
        print(f"[GRASP 관찰 종료] {elapsed:.2f}초, LIFT 명령 없음")
        return elapsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="통합 MCU 직렬 포트")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--reach-sec", type=float, default=2.5, help="고정 집기점까지 이동시간")
    parser.add_argument("--dry-run", action="store_true", help="명령 흐름만 출력")
    args = parser.parse_args()
    if args.reach_sec <= 0.0:
        parser.error("--reach-sec must be > 0")
    return args


def main() -> int:
    args = parse_args()
    print("독립 그리퍼 진단: shoulder=25, wrist=150 고정, gripper 118 -> 50")
    print("이 테스트에는 LIFT/PLACE/STOW 자세 명령이 없습니다.")
    if args.dry_run:
        print(f"INIT {INIT_POSE}")
        print(f"REACH {FIXED_OPEN_POSE} ({args.reach_sec:.2f}s)")
        print(f"GRASP {FIXED_CLOSED_POSE} (사용자 확인까지 유지)")
        print("DRY RUN: 하드웨어 명령을 보내지 않았습니다.")
        return 0

    try:
        base = load_base_module()
    except (OSError, RuntimeError) as exc:
        print(f"테스트 로드 오류: {exc}", file=sys.stderr)
        return 2

    init = pose(base, INIT_POSE)
    opened = pose(base, FIXED_OPEN_POSE)
    closed = pose(base, FIXED_CLOSED_POSE)
    print("주의: 메인 경기와 다른 팔 테스트를 모두 종료하고 손을 가동 범위에서 치우세요.")
    arm = base.SerialArm(args.port, args.baud)
    try:
        arm.open()
        print("[INIT] 첫 명령으로 안전 대기 자세를 활성화합니다.")
        send_for(arm, init, 1.5)
        ramp_to(base, arm, init, opened, args.reach_sec)
        answer = prompt_while_holding(
            arm,
            opened,
            "[FIXED OPEN] (25,150,118) 도달. 그리퍼 닫기=Enter, 중단=q: ",
        )
        if answer == "q":
            raise TestAborted("사용자가 GRASP 전에 중단했습니다")
        close_until_confirmed(arm, closed)
        print("테스트 완료: 팔을 올리지 않고 연결을 종료합니다.")
        return 0
    except TestAborted as exc:
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


if __name__ == "__main__":
    raise SystemExit(main())
