"""
OpenCV CUDA를 활용한 전처리 모듈.

CPU → GPU 데이터 전송을 최소화하기 위해
리사이즈, 정규화 등을 GPU에서 수행합니다.
"""

import cv2
import numpy as np


def preprocess(frame: np.ndarray, target_size: tuple) -> np.ndarray:
    """
    프레임을 모델 입력 형식으로 전처리합니다.

    Args:
        frame: BGR 포맷 numpy 배열 (H, W, 3)
        target_size: (height, width) 목표 크기

    Returns:
        정규화된 float32 배열 (1, 3, H, W)
    """
    h, w = target_size

    # OpenCV CUDA 가속 리사이즈 (가능한 경우)
    if cv2.cuda.getCudaEnabledDeviceCount() > 0:
        gpu_frame = cv2.cuda_GpuMat()
        gpu_frame.upload(frame)
        gpu_resized = cv2.cuda.resize(gpu_frame, (w, h))
        resized = gpu_resized.download()
    else:
        resized = cv2.resize(frame, (w, h))

    # BGR → RGB, HWC → CHW, 정규화
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    normalized = rgb.astype(np.float32) / 255.0
    chw = np.transpose(normalized, (2, 0, 1))

    return np.expand_dims(chw, axis=0)  # (1, 3, H, W)
