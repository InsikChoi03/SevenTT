"""
TensorRT 기반 객체 탐지 클래스.

사용 예시:
    detector = TRTDetector("models/model.engine")
    results = detector.infer(frame)
"""

import numpy as np

try:
    import tensorrt as trt
    import pycuda.driver as cuda
    import pycuda.autoinit  # noqa: F401
    TRT_AVAILABLE = True
except ImportError:
    TRT_AVAILABLE = False

from src.utils import get_logger

logger = get_logger(__name__)


class _HostDeviceMem:
    """호스트(CPU)와 디바이스(GPU) 메모리 쌍을 관리하는 내부 구조체."""

    def __init__(self, host_mem, device_mem):
        self.host = host_mem
        self.device = device_mem

    def __del__(self):
        self.device.free()


class TRTDetector:
    """
    TensorRT 엔진을 로드하고 추론을 실행하는 클래스.

    초기화 비용이 크므로 루프 밖에서 한 번만 생성하세요.
    엔진 파일은 현재 젯슨의 GPU 아키텍처에서 빌드된 것이어야 합니다.
    """

    def __init__(self, engine_path: str, input_shape: tuple = (1, 3, 640, 640)):
        """
        Args:
            engine_path: TensorRT .engine 파일 경로
            input_shape: 모델 입력 shape (N, C, H, W)
        """
        if not TRT_AVAILABLE:
            raise RuntimeError(
                "TensorRT 또는 pycuda가 설치되지 않았습니다. "
                "docs/jetson_setup.md 를 참고하세요."
            )

        self.input_shape = input_shape
        self.logger = trt.Logger(trt.Logger.WARNING)

        logger.info(f"TensorRT 엔진 로드 중: {engine_path}")
        self.engine = self._load_engine(engine_path)
        self.context = self.engine.create_execution_context()

        # GPU 버퍼는 초기화 시 한 번만 할당 (매 프레임 할당 금지)
        self.inputs, self.outputs, self.bindings, self.stream = \
            self._allocate_buffers()
        logger.info("TRTDetector 초기화 완료")

    def _load_engine(self, engine_path: str):
        with open(engine_path, "rb") as f, trt.Runtime(self.logger) as runtime:
            return runtime.deserialize_cuda_engine(f.read())

    def _allocate_buffers(self):
        inputs, outputs, bindings = [], [], []
        stream = cuda.Stream()

        for binding in self.engine:
            size = trt.volume(self.engine.get_binding_shape(binding))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            bindings.append(int(device_mem))

            if self.engine.binding_is_input(binding):
                inputs.append(_HostDeviceMem(host_mem, device_mem))
            else:
                outputs.append(_HostDeviceMem(host_mem, device_mem))

        return inputs, outputs, bindings, stream

    def infer(self, frame: np.ndarray) -> list:
        """
        단일 프레임에 대해 추론을 실행합니다.

        Args:
            frame: BGR 포맷의 numpy 배열 (H, W, 3)

        Returns:
            탐지 결과 리스트 (후처리된 bounding box, confidence, class)
        """
        from src.detection.preprocess import preprocess

        preprocessed = preprocess(frame, self.input_shape[2:])
        np.copyto(self.inputs[0].host, preprocessed.ravel())

        # CPU → GPU 전송
        for inp in self.inputs:
            cuda.memcpy_htod_async(inp.device, inp.host, self.stream)

        # 추론 실행
        self.context.execute_async_v2(
            bindings=self.bindings, stream_handle=self.stream.handle
        )

        # GPU → CPU 전송
        for out in self.outputs:
            cuda.memcpy_dtoh_async(out.host, out.device, self.stream)

        self.stream.synchronize()

        return self._postprocess([out.host for out in self.outputs])

    def _postprocess(self, outputs: list) -> list:
        """
        모델 출력을 bounding box 리스트로 변환합니다.
        모델 구조에 맞게 수정하세요.
        """
        # TODO: 모델별 후처리 로직 구현
        return outputs

    def __del__(self):
        if hasattr(self, "stream"):
            del self.stream
