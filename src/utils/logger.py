"""
프로젝트 공통 로거 설정.
"""

import logging
import sys
from pathlib import Path

_LOG_DIR = Path(__file__).resolve().parents[3] / "logs"


def get_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """
    파일 + 콘솔 출력을 동시에 지원하는 로거를 반환합니다.

    Args:
        name: 로거 이름 (보통 __name__ 전달)
        level: 로그 레벨

    Returns:
        설정된 Logger 인스턴스
    """
    logger = logging.getLogger(name)

    if logger.handlers:
        return logger

    logger.setLevel(level)
    formatter = logging.Formatter(
        "[%(asctime)s] %(levelname)s [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    # 콘솔 핸들러
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # 파일 핸들러 (logs/ 디렉토리가 없으면 파일 로그 생략)
    if _LOG_DIR.exists():
        file_handler = logging.FileHandler(_LOG_DIR / "app.log", encoding="utf-8")
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger
