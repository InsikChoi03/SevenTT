"""
모터 제어 모듈.

시리얼 통신 또는 GPIO를 통해 모터 드라이버를 제어합니다.
실제 하드웨어 인터페이스에 맞게 수정하세요.
"""

import serial
from src.utils import get_logger

logger = get_logger(__name__)


class MotorController:
    """
    시리얼 통신 기반 모터 컨트롤러.

    사용 예시:
        with MotorController(port="/dev/ttyUSB0") as motor:
            motor.forward(speed=50)
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 9600):
        self.port = port
        self.baudrate = baudrate
        self.serial = None

    def connect(self):
        self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
        logger.info(f"모터 컨트롤러 연결: {self.port} @ {self.baudrate}bps")

    def _send(self, command: str):
        if self.serial is None or not self.serial.is_open:
            raise RuntimeError("시리얼 포트가 열려있지 않습니다.")
        self.serial.write(f"{command}\n".encode())

    def forward(self, speed: int = 50):
        """전진. speed: 0~100"""
        self._send(f"F{speed:03d}")

    def backward(self, speed: int = 50):
        """후진. speed: 0~100"""
        self._send(f"B{speed:03d}")

    def turn_left(self, speed: int = 50):
        """좌회전. speed: 0~100"""
        self._send(f"L{speed:03d}")

    def turn_right(self, speed: int = 50):
        """우회전. speed: 0~100"""
        self._send(f"R{speed:03d}")

    def stop(self):
        """정지"""
        self._send("S000")

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.stop()
            self.serial.close()
            logger.info("모터 컨트롤러 연결 해제")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()
