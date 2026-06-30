# 하드웨어 인벤토리

## 컴퓨트
- **Jetson Orin Nano Developer Kit 8GB** (P3768-0000 + P3767-0005-super)
- JetPack R36.5 (2026-01 빌드) / Ubuntu 22.04.5 / CUDA 12.6 / TensorRT 10.3
- Python 3.10.12 / 512GB NVMe SSD

## 본체 (메카넘 베이스)
- **제품**: 쿠팡 메카넘휠 모바일 로봇 플랫폼 ([link.coupang.com/a/dO3vjYNDC8](https://link.coupang.com/a/dO3vjYNDC8))
- 4륜 메카넘 사륜구동
- **제어 보드**: Arduino UNO (**CH340 클론**) + 모터 드라이버 = **PCA9685(I²C 0x60) + MX1508 H-브리지 ×4** (※ L293D 아님 — 초기 오인)
- **배터리**: 12V (3.7V 리튬셀 × 3 직렬 팩, 베이스 동봉)
- **상태(2026-06-02)**: ✅ **Jetson 시리얼 제어 동작 확인** — 전진/후진/좌우 횡이동/제자리 회전 검증 완료
- **제어 경로**: Jetson `<BASE,fl,fr,rl,rr>` → USB(`/dev/ttyUSB0`, CH340) → UNO → I²C → PCA9685(0x60) → MX1508 ×4 → 모터
- **펌웨어**: [`firmware/base_mecanum_pca/`](../firmware/base_mecanum_pca/) (arduino-cli 빌드/플래시, 툴체인 `tools/arduino-cli`). PCA9685 채널맵 FL=8/9, FR=12/13, RL=10/11, RR=14/15 (정/역). 워치독 500ms.
- ⚠️ **CH340 드라이버**: Jetson 커널에 `ch341` 없음 → 소스 빌드함([`drivers/ch341/`](../drivers/ch341/)) + brltty 제거. 자세히: 메모리 `reference-jetson-ch340-driver`. **현재 모듈은 메모리에만 로드 → 재부팅 시 영구화 필요(미적용).**

### 상판 배치 (잠정)
| 위치 | 장착물 |
|---|---|
| Front | 로봇팔 Scipia A2T |
| Rear | 카메라 리프트 (광각 IMX219) |
| 중앙 | 객체 트레이 / 임시 보관함 (디자인 미정) |

**용어 주의**: "본체 트레이"는 픽업 객체를 잠시 들고 다니는 본체 위 임시 적재함. 룰북의 **"보관함(Storage Zone)"** (경기장 모서리 40×40cm)과 별개. 최종 점수는 룰북 Storage Zone에 들어간 객체만 인정.

## 로봇팔
- **Scipia A2T 6자유도 알루미늄** (아두이노 호환 키트)
- 액추에이터: MG996R × 6 — 정밀도 ±2° (비주얼 서보잉 필수 사유)
- 출처: [scipia.com](https://scipia.com/product/a2t-6%EC%9E%90%EC%9C%A0%EB%8F%84-%EC%95%8C%EB%A3%A8%EB%AF%B8%EB%8A%84-%EB%A1%9C%EB%B4%87%ED%8C%94%ED%82%A4%ED%8A%B8-%EC%95%84%EB%91%90%EC%9D%B4%EB%85%B8/592/)

## 저수준 제어 (팔)
- **MCU**: Arduino Uno (보유분 또는 신규)
- **서보 드라이버**: PCA9685 (I2C 16채널 하드웨어 PWM, ~₩3,000)
- **센서 실드**: 선 보호·정리

## 통신 토폴로지
```
Jetson Orin Nano (Master)
  ├─ USB serial ─→ Arduino_arm ──I²C──→ PCA9685 ──PWM──→ MG996R × 6
  └─ USB serial ─→ Arduino_base ─PWM/Driver─→ 메카넘 4모터
```

### 시리얼 프로토콜
- **팔**: `<θ1, θ2, θ3, θ4, θ5, θ6>` 텍스트 (각도 0~180°)
- **베이스**: `<BASE,fl,fr,rl,rr>` 휠 선속도 m/s (확정·구현됨, `firmware/base_mecanum_pca/`). 엔코더 없어 inbound은 `<HB,...>`(명령 echo), `<ODOM>`은 엔코더 추가 시
- 호스트: `pyserial` / 펌웨어: `Serial.read`

## 카메라

### 본체 카메라 (eye-in-hand, 픽업·서보잉·SigLIP 게이트)
- **Waveshare IMX219 B0183** 저왜곡 M12 마운트
- **장착 위치**: 로봇팔 **손목(wrist) 위쪽** — 그리퍼 직전 관절 상단
  - 이유: MG996R ±2° 정밀도라 eye-in-hand 서보잉 필수. 손목 장착은 그리퍼+객체 동시 시야, 끝단 흔들림 적음, CSI FFC 30cm 한계 내
- 인터페이스: CSI 22-pin, `sensor-id=1`
- 접근: `nvarguscamerasrc sensor-id=1`
- 좌표 변환: tf2 `arm_wrist → camera_body` static_transform 1개

### 상단 광각 카메라 (지속 사용, 위치 추정·인식·트래킹)
- **[SMG] IMX219 젯슨나노 160도 광각 8MP** ([devicemart 12538383](https://www.devicemart.co.kr/goods/view?no=12538383))
- 위치: 카메라 리프트로 본체 위 ~80cm
- 화각 160° → 가장자리 왜곡 큼, 캘리브레이션 + 크롭 처리
- **동작 모드**: 게임 내내 지속 캡처 5~10 FPS
  - 로봇 자기 위치 추정 (깃대·담장·보관함 랜드마크 ↔ 휠 오도메트리 fuse)
  - 객체 상태 실시간 추적 (픽업 직후 사라짐, 일시 occlusion 등)
  - 본체에 달려있어 본체 이동에 따라 시야 흔들림 → 매 프레임 self-localization ↔ world model 업데이트
- 리프트 프로파일로 한쪽 면 일부 가림 → world model occlusion mask 필요
- **슬롯/매핑**: `sensor-id=0` (확정)

### Device tree
- Overlay: `tegra234-p3767-camera-p3768-imx219-dual.dtbo` (적용됨)
- `/dev/video*`는 생성되지 않음 — **nvarguscamerasrc로만 접근**

## 카메라 리프트
- 2020 알루미늄 프로파일
- 측면 베어링 풀리 + DC모터로 줄 감김
- **보아 다이얼**: 단방향(상승)만 동작, 끝까지 올린 후 고정
- 본체 rear 장착
- 게임 중 재조정 불가 — 시작 시 한 번 올린 상태가 정상

## 전원

### 팔 / Jetson 계열
- **배터리**: Turnigy ZIPPY Flightmax 2S 7.4V 4000mAh 30C Hardcase ([falconshop 100054806](https://www.falconshop.co.kr/shop/goods/goods_view.php?goodsno=100054806&category=019003041))
  - 137×47×25mm / 256g / JST-XH 밸런스 / 4mm 불릿
- **UBEC**: HOBBYWING UBEC 10A-Car ([falconshop 100069127](https://www.falconshop.co.kr/shop/goods/goods_view.php?goodsno=100069127&category=019002003))
  - 입력 2-6S LiPo, 출력 6.0/7.4/8.4V 3단, 연속 10A · 최대 15A

### 메카넘 베이스 계열
- 자체 12V (3.7V × 3셀) 팩 (베이스 제품 동봉)

### 충전
- IMAX B6AC (당근 또는 신규)
- HXT 4.0mm × 2 ([Amazon B072JVQJ1R](https://www.amazon.com/OliYin-Charge-Connector-11-8inch-Silicone/dp/B072JVQJ1R))
  - 1개 충전 전용 / 1개 잘라서 회로 연결

## 미정 / 추후 결정
- 본체 트레이/임시 보관함 형상 + 룰북 Storage Zone 운반 전략
- 그리퍼 (A2T 기본 포함분 사용 여부)
- ~~메카넘 베이스 Arduino 펌웨어~~ ✅ 완료 (PCA9685+MX1508, `firmware/base_mecanum_pca/`). 남은 것: ch341 모듈 부팅 자동로드 영구화 + `/dev/base` udev 고정이름 (현재 미적용)
- 본체/광각 카메라 슬롯 CAM0/CAM1 매핑 확정
- 두 IMX219 동시 운용 시 nvargus 안정성·대역폭 검증

## 카메라 캡처 코드 예시 (CSI IMX219)

```python
import cv2

def make_pipeline(sensor_id, width=1920, height=1080, fps=30):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"framerate={fps}/1, format=NV12 ! "
        f"nvvidconv flip-method=0 ! "
        f"video/x-raw, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! "
        f"appsink drop=true max-buffers=1"
    )

# 광각 (탐지·localization)
top  = cv2.VideoCapture(make_pipeline(0), cv2.CAP_GSTREAMER)
# 본체 (분류·서보잉)
body = cv2.VideoCapture(make_pipeline(1), cv2.CAP_GSTREAMER)
```
