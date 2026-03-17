"""
젯슨 카메라 입력 모듈.

CSI 카메라(GStreamer 파이프라인)와 USB 카메라를 모두 지원합니다.
"""

import cv2
from src.utils import get_logger

logger = get_logger(__name__)


def _gstreamer_pipeline(
    sensor_id: int = 0,
    capture_width: int = 1920,
    capture_height: int = 1080,
    display_width: int = 640,
    display_height: int = 640,
    framerate: int = 30,
    flip_method: int = 0,
) -> str:
    """CSI 카메라용 GStreamer 파이프라인 문자열을 생성합니다."""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, "
        f"height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, "
        f"format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! "
        f"appsink"
    )


class CameraCapture:
    """
    카메라 프레임을 캡처하는 클래스.

    사용 예시:
        with CameraCapture(source="csi") as cam:
            for frame in cam:
                process(frame)
    """

    def __init__(self, source: str = "csi", device_id: int = 0):
        """
        Args:
            source: "csi" (CSI 카메라) 또는 "usb" (USB 카메라)
            device_id: USB 카메라 디바이스 번호 (source="usb"일 때 사용)
        """
        self.source = source
        self.device_id = device_id
        self.cap = None

    def open(self):
        if self.source == "csi":
            pipeline = _gstreamer_pipeline()
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            logger.info("CSI 카메라 GStreamer 파이프라인 열기 완료")
        else:
            self.cap = cv2.VideoCapture(self.device_id)
            logger.info(f"USB 카메라 열기 완료: /dev/video{self.device_id}")

        if not self.cap.isOpened():
            raise RuntimeError(
                f"카메라를 열 수 없습니다. source={self.source}, "
                f"device_id={self.device_id}"
            )

    def read(self):
        """프레임 한 장을 읽어 반환합니다."""
        if self.cap is None:
            raise RuntimeError("카메라가 열려있지 않습니다. open()을 먼저 호출하세요.")
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.release()

    def __iter__(self):
        while True:
            frame = self.read()
            if frame is None:
                break
            yield frame
