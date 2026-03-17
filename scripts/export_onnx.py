"""
PyTorch 모델을 ONNX 형식으로 변환하는 스크립트.

사용법:
    python scripts/export_onnx.py --weights models/yolov8n.pt --output models/yolov8n.onnx
"""

import argparse
from pathlib import Path

from src.utils import get_logger

logger = get_logger(__name__)


def export_onnx(weights_path: str, output_path: str, imgsz: int = 640):
    try:
        from ultralytics import YOLO
    except ImportError:
        raise ImportError("ultralytics 패키지가 필요합니다: pip install ultralytics")

    logger.info(f"모델 로드: {weights_path}")
    model = YOLO(weights_path)

    logger.info(f"ONNX 변환 시작 (imgsz={imgsz})...")
    model.export(format="onnx", imgsz=imgsz, simplify=True)

    # ultralytics는 같은 디렉토리에 저장하므로 필요 시 이동
    default_output = Path(weights_path).with_suffix(".onnx")
    if str(default_output) != output_path:
        default_output.rename(output_path)

    logger.info(f"ONNX 변환 완료: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PyTorch → ONNX 변환")
    parser.add_argument("--weights", default="models/yolov8n.pt", help=".pt 파일 경로")
    parser.add_argument("--output", default="models/yolov8n.onnx", help="출력 경로")
    parser.add_argument("--imgsz", type=int, default=640, help="입력 이미지 크기")
    args = parser.parse_args()

    export_onnx(args.weights, args.output, args.imgsz)
