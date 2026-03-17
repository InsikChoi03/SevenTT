"""
ONNX 모델을 TensorRT 엔진으로 변환하는 스크립트.
이 스크립트는 반드시 실제 젯슨 기기에서 실행해야 합니다.

사용법:
    python scripts/build_engine.py --onnx models/yolov8n.onnx --output models/yolov8n.engine
"""

import argparse

from src.utils import get_logger

logger = get_logger(__name__)


def build_engine(
    onnx_path: str,
    engine_path: str,
    fp16: bool = True,
    max_workspace_gb: int = 4,
):
    """
    ONNX → TensorRT 엔진 변환.

    Args:
        onnx_path: 입력 ONNX 파일 경로
        engine_path: 출력 .engine 파일 경로
        fp16: FP16 정밀도 활성화 여부 (젯슨 권장)
        max_workspace_gb: TensorRT 최대 작업 메모리 (GB)
    """
    try:
        import tensorrt as trt
    except ImportError:
        raise ImportError("TensorRT가 설치되지 않았습니다. 젯슨에서 실행하세요.")

    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser = trt.OnnxParser(network, logger)
    config = builder.create_builder_config()
    config.max_workspace_size = max_workspace_gb * (1 << 30)

    if fp16 and builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)
        logger.info("FP16 모드 활성화")

    logger.info(f"ONNX 파싱 중: {onnx_path}")
    with open(onnx_path, "rb") as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(parser.get_error(i))
            raise RuntimeError("ONNX 파싱 실패")

    logger.info("TensorRT 엔진 빌드 중... (수 분 소요될 수 있습니다)")
    serialized_engine = builder.build_serialized_network(network, config)

    with open(engine_path, "wb") as f:
        f.write(serialized_engine)

    logger.info(f"엔진 저장 완료: {engine_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ONNX → TensorRT 엔진 빌드")
    parser.add_argument("--onnx", default="models/yolov8n.onnx", help="ONNX 파일 경로")
    parser.add_argument("--output", default="models/yolov8n.engine", help="출력 경로")
    parser.add_argument("--fp16", action="store_true", default=True, help="FP16 활성화")
    parser.add_argument("--workspace", type=int, default=4, help="최대 작업 메모리 (GB)")
    args = parser.parse_args()

    build_engine(args.onnx, args.output, args.fp16, args.workspace)
