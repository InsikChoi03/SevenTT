"""
팀 공유 스토리지에서 모델 파일을 다운로드하는 스크립트.

Google Drive ID는 팀 내부에서 공유하세요. (.env 또는 팀 채널)

사용법:
    python scripts/download_models.py
"""

import os
from pathlib import Path

from src.utils import get_logger

logger = get_logger(__name__)

# Google Drive 파일 ID를 여기에 입력하세요.
# ID 확인 방법: Drive 공유 링크에서 /d/와 /view 사이의 문자열
MODEL_FILES = {
    "models/yolov8n.pt": os.getenv("GDRIVE_PT_ID", ""),
    "models/yolov8n.onnx": os.getenv("GDRIVE_ONNX_ID", ""),
}


def download_models():
    try:
        import gdown
    except ImportError:
        raise ImportError("gdown이 필요합니다: pip install gdown")

    Path("models").mkdir(exist_ok=True)

    for output_path, file_id in MODEL_FILES.items():
        if not file_id:
            logger.warning(
                f"파일 ID가 설정되지 않았습니다: {output_path}\n"
                f"  .env 파일에 GDRIVE_PT_ID 또는 GDRIVE_ONNX_ID를 설정하세요."
            )
            continue

        if Path(output_path).exists():
            logger.info(f"이미 존재함, 건너뜀: {output_path}")
            continue

        logger.info(f"다운로드 중: {output_path}")
        gdown.download(id=file_id, output=output_path, quiet=False)
        logger.info(f"완료: {output_path}")


if __name__ == "__main__":
    download_models()
