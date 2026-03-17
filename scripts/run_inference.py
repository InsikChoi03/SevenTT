"""
추론 실행 메인 스크립트.

사용법:
    python scripts/run_inference.py
    python scripts/run_inference.py --config config/model_config.yaml --test
"""

import argparse
import time

import cv2
import yaml

from src.camera import CameraCapture
from src.control import MotorController
from src.detection import TRTDetector
from src.utils import get_logger

logger = get_logger(__name__)


def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def run(model_cfg: dict, hw_cfg: dict, test_mode: bool = False):
    detector = TRTDetector(
        engine_path=model_cfg["model"]["engine_path"],
        input_shape=model_cfg["model"]["input_shape"],
    )

    cam_cfg = hw_cfg["camera"]
    motor_cfg = hw_cfg["motor"]

    frame_count = 0
    fps_start = time.time()

    with CameraCapture(source=cam_cfg["source"], device_id=cam_cfg["device_id"]) as cam, \
         MotorController(port=motor_cfg["port"], baudrate=motor_cfg["baudrate"]) as motor:

        logger.info("추론 루프 시작")

        for frame in cam:
            results = detector.infer(frame)

            # TODO: 탐지 결과에 따른 모터 제어 로직 추가
            # 예: if detected_object == "obstacle": motor.stop()

            frame_count += 1
            if frame_count % 30 == 0:
                elapsed = time.time() - fps_start
                fps = frame_count / elapsed
                logger.info(f"FPS: {fps:.1f}")

            if test_mode and frame_count >= 10:
                logger.info("테스트 모드: 10프레임 처리 완료")
                break

    logger.info(f"종료. 총 처리 프레임: {frame_count}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="비전 로봇 추론 실행")
    parser.add_argument("--config", default="config/model_config.yaml")
    parser.add_argument("--hw-config", default="config/hardware_config.yaml")
    parser.add_argument("--test", action="store_true", help="테스트 모드 (10프레임)")
    args = parser.parse_args()

    model_cfg = load_config(args.config)
    hw_cfg = load_config(args.hw_config)
    run(model_cfg, hw_cfg, test_mode=args.test)
