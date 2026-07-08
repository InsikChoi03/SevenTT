#!/usr/bin/env python3
"""비대화형 단발 모션 드라이버 (대화형 튜닝 루프용).

motion_tune.py 는 사람이 프롬프트에 타이핑하는 REPL이라 원격/자동 루프에 못 쓴다.
이 스크립트는 motion_tune 의 **검증된 프리미티브(wheel_cmd/run_continuous/DEFAULT_*)**를
그대로 import 해서, 인자로 받은 **한 모션만** 실행하고 즉시 종료한다(프롬프트 없음).

한 트라이얼:
    python3 tune_drive.py --direction forward --target-cm 30
    -> seed duration(=duration_seed)로 boost->steady->brake 실행. 실측은 사람이 잰다.

다음 트라이얼(측정값 반영):
    python3 tune_drive.py --direction forward --target-cm 30 --duration-ms 471

측정만으로 다음 duration 계산(하드웨어 미구동):
    python3 tune_drive.py --direction forward --target-cm 30 --duration-ms 424 --suggest 27
    -> "used 424ms -> 27.0cm -> for 30cm suggest 471ms" 출력만.

모든 실구동은 세션 CSV(--csv, 기본 control_retune_log.csv)에 params+used_duration 기록.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import serial  # noqa: E402

from encoder_distance_test import stop_base  # noqa: E402
from motion_tune import (  # noqa: E402
    DEFAULT_PARAMS,
    DEFAULT_SCALES,
    KINDS,
    duration_seed,
    format_cmd,
    parse_scales,
    run_continuous,
    suggest_duration_ms,
    wheel_cmd,
)


def append_row(path: Path, row: dict) -> None:
    import csv
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            w.writeheader()
        w.writerow(row)


def main() -> None:
    ap = argparse.ArgumentParser(description="single-shot mecanum motion (non-interactive)")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--direction", required=True, choices=list(KINDS))
    ap.add_argument("--target-cm", type=float, default=0.0)
    ap.add_argument("--target-deg", type=float, default=0.0)
    ap.add_argument("--duration-ms", type=int, default=0, help="0 = empirical seed")
    ap.add_argument("--speed", type=float, default=-1.0)
    ap.add_argument("--boost-ms", type=int, default=-1)
    ap.add_argument("--boost-speed", type=float, default=-1.0)
    ap.add_argument("--brake-ms", type=int, default=-1)
    ap.add_argument("--brake-scale", type=float, default=-1.0)
    ap.add_argument("--scales", default="", help="4 nums FL FR RL RR (default = per-direction tuned)")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--settle", type=float, default=2.0, help="MCU 부트 대기(초)")
    ap.add_argument("--suggest", type=float, default=None,
                    help="측정 실측값. 주면 구동 안 하고 다음 duration만 계산/출력")
    ap.add_argument("--csv", default="scripts/drive_tests/control_retune_log.csv")
    ap.add_argument("--note", default="")
    args = ap.parse_args()

    d = args.direction
    kind = KINDS[d]
    unit = "deg" if kind == "angle" else "cm"
    target = args.target_deg if kind == "angle" and args.target_deg > 0 else args.target_cm
    if target <= 0:
        print("ERROR: give --target-cm (distance) or --target-deg (rotation)", file=sys.stderr)
        sys.exit(2)

    dur = args.duration_ms if args.duration_ms > 0 else duration_seed(d, target)
    dp = DEFAULT_PARAMS[d]
    speed = dp["speed"] if args.speed < 0 else args.speed
    boost_speed = dp["boost_speed"] if args.boost_speed < 0 else args.boost_speed
    boost_ms = dp["boost_ms"] if args.boost_ms < 0 else args.boost_ms
    brake_ms = dp["brake_ms"] if args.brake_ms < 0 else args.brake_ms
    brake_scale = dp["brake_scale"] if args.brake_scale < 0 else args.brake_scale
    scales = parse_scales(args.scales, d) if args.scales else list(DEFAULT_SCALES[d])

    # 측정값만 주면: 다음 duration 계산 후 종료(하드웨어 미구동).
    if args.suggest is not None:
        nxt = suggest_duration_ms(target, args.suggest, dur)
        err = args.suggest - target
        print(f"[SUGGEST] {d} {target:g}{unit}: used {dur}ms -> {args.suggest:g}{unit} "
              f"(err {err:+.1f}{unit}) -> next duration {nxt}ms")
        return

    steady = wheel_cmd(d, speed, scales)
    boost = wheel_cmd(d, boost_speed, scales)
    print(f"=== DRIVE {d} {target:g}{unit} ===")
    print(f"duration={dur}ms  speed={speed:.2f}  boost={boost_speed:.2f}/{boost_ms}ms  "
          f"brake={brake_scale:.2f}/{brake_ms}ms  scales=[{' '.join(f'{s:.2f}' for s in scales)}]")
    print(f"steady {format_cmd(steady)}")
    if boost_ms > 0:
        print(f"boost  {format_cmd(boost)}")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except (serial.SerialException, OSError) as e:
        print(f"port open failed: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"opened {args.port}; MCU reboot wait {args.settle}s...")
    time.sleep(args.settle)
    ser.reset_input_buffer()
    stop_base(ser)

    started = time.monotonic()
    try:
        run_continuous(
            ser, steady_cmd=steady, boost_cmd=boost, duration_ms=dur,
            boost_ms=int(boost_ms), brake_ms=int(brake_ms),
            brake_scale=float(brake_scale), hz=args.hz,
        )
    finally:
        for _ in range(5):
            stop_base(ser)
            time.sleep(0.02)
        ser.flush()
        ser.close()
    elapsed = time.monotonic() - started
    print(f"done in {elapsed:.3f}s. -> 바닥에서 실측 {unit} 재서 알려주세요.")

    append_row(Path(args.csv), {
        "direction": d, "target": target, "unit": unit, "used_duration_ms": dur,
        "speed": speed, "boost_speed": boost_speed, "boost_ms": boost_ms,
        "brake_ms": brake_ms, "brake_scale": brake_scale,
        "scales": " ".join(f"{s:.2f}" for s in scales),
        "elapsed_s": round(elapsed, 3), "actual": "", "note": args.note,
    })


if __name__ == "__main__":
    main()
