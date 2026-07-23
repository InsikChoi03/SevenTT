#!/usr/bin/env python3
"""메인 경기의 2R 집기→놓기 시퀀스만 한 번 실행하는 독립 하드웨어 테스트.

카메라, YOLO, 주행, mission_fsm_node 없이 통합 MCU에 직접 연결한다. 팔의 포즈와
시간은 실행할 때마다 메인 경기의 motion_tuning.yaml에서 읽으며, 동작 순서는
pick_sequencer_node.py의 PICK_PLACE 시퀀스와 동일하다.

    python3 scripts/shape4_pick_place_test_v5.3.0.py
    python3 scripts/shape4_pick_place_test_v5.3.0.py --dry-run
    python3 scripts/shape4_pick_place_test_v5.3.0.py --port /dev/ttyUSB0 --delay 5

물체는 로봇 정면의 기존 메인 집기 위치에 둔다. 이 테스트는 물체를 인식하거나 로봇을
접근시키지 않으므로, 물체가 그리퍼 집기점에 맞아 있어야 한다.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
import time
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "ros2_ws/src/robot_bringup/config/motion_tuning.yaml"
RATE_HZ = 20.0


@dataclass(frozen=True)
class Pose:
    shoulder: float
    wrist: float
    gripper: float

    def values(self) -> tuple[float, float, float]:
        return self.shoulder, self.wrist, self.gripper


@dataclass(frozen=True)
class PickConfig:
    init: Pose
    pick_sw: tuple[float, float]
    place_sw: tuple[float, float]
    grip_open: float
    grip_closed: float
    move_sec: float
    grasp_sec: float
    preopen_sec: float
    pick_settle_sec: float
    servo_speed_dps: float


@dataclass(frozen=True)
class Step:
    name: str
    duration: float
    start: Pose
    target: Pose


def _float_list(value: object, name: str, count: int) -> list[float]:
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{name} must contain exactly {count} values")
    values = [float(item) for item in value]
    if any(item < 0.0 or item > 180.0 for item in values):
        raise ValueError(f"{name} angles must be in [0, 180]")
    return values


def load_config(path: Path) -> PickConfig:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML이 필요합니다: sudo apt install python3-yaml") from exc

    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    params = raw["pick_sequencer_node"]["ros__parameters"]
    init = _float_list(params["init_pose"], "init_pose", 3)
    pick = _float_list(params["pick_shoulder_wrist"], "pick_shoulder_wrist", 2)
    place = _float_list(params["place_shoulder_wrist"], "place_shoulder_wrist", 2)

    config = PickConfig(
        init=Pose(*init),
        pick_sw=(pick[0], pick[1]),
        place_sw=(place[0], place[1]),
        grip_open=float(params["grip_open"]),
        grip_closed=float(params["grip_closed"]),
        move_sec=float(params["move_sec"]),
        grasp_sec=float(params["grasp_sec"]),
        preopen_sec=float(params["preopen_sec"]),
        pick_settle_sec=float(params["pick_settle_sec"]),
        servo_speed_dps=float(params["servo_speed_dps"]),
    )
    angles = (config.grip_open, config.grip_closed)
    if any(value < 0.0 or value > 180.0 for value in angles):
        raise ValueError("grip_open/grip_closed must be in [0, 180]")
    timings = (
        config.move_sec,
        config.grasp_sec,
        config.preopen_sec,
        config.pick_settle_sec,
        config.servo_speed_dps,
    )
    if any(value <= 0.0 for value in timings):
        raise ValueError("all pick timing and speed values must be positive")
    return config


def build_steps(config: PickConfig) -> list[Step]:
    ish, iwr, _ = config.init.values()
    psh, pwr = config.pick_sw
    plsh, plwr = config.place_sw
    requested = [
        ("PREOPEN", config.preopen_sec, Pose(ish, iwr, config.grip_open)),
        ("REACH", config.move_sec, Pose(psh, pwr, config.grip_open)),
        ("PICK_SETTLE", config.pick_settle_sec, Pose(psh, pwr, config.grip_open)),
        ("GRASP", config.grasp_sec, Pose(psh, pwr, config.grip_closed)),
        ("LIFT", config.move_sec, Pose(ish, iwr, config.grip_closed)),
        ("TO_PLACE", config.move_sec, Pose(plsh, plwr, config.grip_closed)),
        ("PLACE", config.grasp_sec, Pose(plsh, plwr, config.grip_open)),
        ("STOW", config.grasp_sec, config.init),
    ]
    move_steps = {"REACH", "LIFT", "TO_PLACE", "STOW"}
    steps: list[Step] = []
    current = config.init
    for name, configured_duration, target in requested:
        duration = configured_duration
        if name in move_steps:
            travel = max(abs(a - b) for a, b in zip(current.values(), target.values()))
            duration = max(duration, travel / config.servo_speed_dps)
        steps.append(Step(name, duration, current, target))
        current = target
    return steps


def lerp(start: Pose, target: Pose, progress: float) -> Pose:
    u = max(0.0, min(1.0, progress))
    return Pose(*(a + (b - a) * u for a, b in zip(start.values(), target.values())))


class SerialArm:
    def __init__(self, port: str, baud: int) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial이 필요합니다: sudo apt install python3-serial") from exc
        self._serial_module = serial
        self.port = port
        self.baud = baud
        self.ser = None

    def open(self) -> None:
        try:
            self.ser = self._serial_module.Serial(
                self.port,
                self.baud,
                timeout=0.02,
                write_timeout=1.0,
                exclusive=True,
            )
        except TypeError:
            self.ser = self._serial_module.Serial(
                self.port, self.baud, timeout=0.02, write_timeout=1.0
            )
        print(f"MCU 연결: {self.port} @ {self.baud} (부팅 대기 2.5초)")
        time.sleep(2.5)
        self.drain()
        self.line("<STOP>")
        self.line("<LIFT,0>")
        self.line("<STATUS,RUNNING>")
        self.line("<BASE,0.000,0.000,0.000,0.000>")

    def line(self, command: str) -> None:
        if self.ser is None:
            return
        self.ser.write((command.rstrip() + "\n").encode("ascii"))
        self.ser.flush()

    def pose(self, target: Pose) -> None:
        values = (int(round(value)) for value in target.values())
        self.line("<ARM," + ",".join(str(value) for value in values) + ">")

    def drain(self) -> None:
        if self.ser is not None and self.ser.in_waiting:
            self.ser.read(self.ser.in_waiting)

    def safe_close(self) -> None:
        if self.ser is None:
            return
        try:
            for _ in range(3):
                self.line("<BASE,0.000,0.000,0.000,0.000>")
                self.line("<STOP>")
                time.sleep(0.03)
            self.line("<LIFT,0>")
            self.line("<STATUS,STANDBY>")
        finally:
            self.ser.close()
            self.ser = None


def describe(config: PickConfig, steps: Iterable[Step], config_path: Path) -> None:
    steps = list(steps)
    print(f"설정: {config_path}")
    print(f"INIT={config.init.values()} PICK={config.pick_sw} PLACE={config.place_sw}")
    print(f"GRIP open={config.grip_open} closed={config.grip_closed}")
    print("시퀀스:")
    for step in steps:
        print(f"  {step.name:11s} {step.duration:4.2f}s -> {step.target.values()}")
    print(f"예상 동작 시간: {sum(step.duration for step in steps):.2f}초")


def run_sequence(arm: SerialArm, config: PickConfig, steps: list[Step], delay: float) -> None:
    # 메인 노드처럼 첫 명령을 INIT로 보내 boot-limp의 첫 스냅을 안전한 대기 자세로 흡수한다.
    arm.pose(config.init)
    time.sleep(1.5)
    for remaining in range(int(delay), 0, -1):
        print(f"집기 시작까지 {remaining}초 — 손을 치우세요.")
        time.sleep(1.0)
    fractional = delay - int(delay)
    if fractional > 0.0:
        time.sleep(fractional)

    period = 1.0 / RATE_HZ
    print("PICK_PLACE 시작")
    for step in steps:
        print(
            f"[{step.name}] {tuple(round(v) for v in step.start.values())} -> "
            f"{tuple(round(v) for v in step.target.values())} ({step.duration:.2f}s)"
        )
        started = time.monotonic()
        while True:
            elapsed = time.monotonic() - started
            if elapsed >= step.duration:
                break
            arm.pose(lerp(step.start, step.target, elapsed / step.duration))
            arm.drain()
            time.sleep(period)
        arm.pose(step.target)
    print("PICK_PLACE 완료: 집기→놓기→대기 자세 복귀")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="통합 MCU 직렬 포트")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--delay", type=float, default=3.0, help="INIT 안착 후 집기 전 대기시간")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--dry-run", action="store_true", help="설정과 시퀀스만 출력")
    args = parser.parse_args()
    if args.delay < 0.0:
        parser.error("--delay must be >= 0")
    return args


def main() -> int:
    args = parse_args()
    try:
        config_path = args.config.expanduser().resolve()
        config = load_config(config_path)
        steps = build_steps(config)
        describe(config, steps, config_path)
    except (OSError, KeyError, TypeError, ValueError, RuntimeError) as exc:
        print(f"설정 오류: {exc}", file=sys.stderr)
        return 2

    if args.dry_run:
        print("DRY RUN: 하드웨어 명령을 보내지 않았습니다.")
        return 0

    print("주의: 메인 경기/팔 테스트를 모두 종료하고, 물체와 보관함을 기존 위치에 두세요.")
    print("이 테스트는 인식·정렬 없이 현재 위치에서 바로 집습니다.")
    arm = SerialArm(args.port, args.baud)
    try:
        arm.open()
        run_sequence(arm, config, steps, args.delay)
    except KeyboardInterrupt:
        print("\n사용자 중단: 베이스 정지 및 STANDBY를 전송합니다.", file=sys.stderr)
        return 130
    except Exception as exc:  # serial errors must still pass through the safe shutdown below.
        print(f"실행 오류: {exc}", file=sys.stderr)
        return 1
    finally:
        arm.safe_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
