"""
TRTDetector 단위 테스트.

TensorRT가 없는 환경(노트북)에서도 전처리 로직을 검증할 수 있습니다.
"""

import numpy as np
import pytest


class TestPreprocess:
    """전처리 함수 테스트 (TRT 불필요, 노트북에서 실행 가능)"""

    def test_output_shape(self):
        from src.detection.preprocess import preprocess

        dummy_frame = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)
        result = preprocess(dummy_frame, target_size=(640, 640))

        assert result.shape == (1, 3, 640, 640), f"예상: (1,3,640,640), 실제: {result.shape}"

    def test_output_dtype(self):
        from src.detection.preprocess import preprocess

        dummy_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        result = preprocess(dummy_frame, target_size=(640, 640))

        assert result.dtype == np.float32

    def test_output_range(self):
        from src.detection.preprocess import preprocess

        dummy_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        result = preprocess(dummy_frame, target_size=(640, 640))

        assert result.min() >= 0.0, "최솟값이 0 미만"
        assert result.max() <= 1.0, "최댓값이 1 초과"


class TestTRTDetector:
    """TRTDetector 테스트 (TRT 환경에서만 실행)"""

    @pytest.mark.skipif(
        not __import__("importlib").util.find_spec("tensorrt"),
        reason="TensorRT가 설치되지 않은 환경"
    )
    def test_engine_load(self, tmp_path):
        """엔진 파일이 없을 때 적절한 예외가 발생하는지 확인"""
        from src.detection import TRTDetector

        with pytest.raises(FileNotFoundError):
            TRTDetector(str(tmp_path / "nonexistent.engine"))
