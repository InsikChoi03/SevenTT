# 젯슨 환경 세팅 가이드

## 사전 조건

- NVIDIA Jetson Orin Nano
- JetPack 6.x 이상 설치 완료
- 인터넷 연결

## 1. 레포 클론 및 기본 세팅

```bash
git clone <repo-url>
cd SevenTT
chmod +x setup.sh
./setup.sh
source venv/bin/activate
```

## 2. JetPack 내장 패키지 버전 확인

```bash
# PyTorch 버전 확인 (CUDA 지원 여부 포함)
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"

# OpenCV CUDA 빌드 확인
python3 -c "import cv2; print(cv2.__version__, cv2.cuda.getCudaEnabledDeviceCount())"

# TensorRT 버전 확인
python3 -c "import tensorrt as trt; print(trt.__version__)"
```

정상 출력 예시:
```
2.1.0a0+41361538.nv23.06 True     ← torch (CUDA=True 이어야 함)
4.8.1 1                            ← opencv (CUDA count=1 이어야 함)
8.6.1.6                            ← tensorrt
```

## 3. JetPack 버전별 패키지 정보

| JetPack | PyTorch | Python |
|---------|---------|--------|
| 6.0 | 2.1.0 | 3.10 |
| 5.1 | 2.0.0 | 3.8 |

## 4. Jetson Power Mode 설정

추론 성능 최대화를 위해 MAXN 모드로 설정하세요.

```bash
sudo nvpmodel -m 0     # MAXN (최대 성능)
sudo jetson_clocks     # 클럭 최대로 고정
```

현재 모드 확인:
```bash
sudo nvpmodel -q
```

## 5. Remote-SSH 설정 (VS Code)

1. VS Code에서 `Remote - SSH` 확장 설치
2. `Ctrl+Shift+P` → `Remote-SSH: Connect to Host`
3. `user@jetson-ip` 입력
4. 연결 후 이 레포 디렉토리를 열면 `.vscode/launch.json` 자동 인식

## 6. 자주 발생하는 문제

### `torch.cuda.is_available()` 이 False를 반환하는 경우

```bash
# venv를 --system-site-packages 없이 만든 경우
# venv를 삭제하고 setup.sh를 다시 실행하세요
rm -rf venv
./setup.sh
```

### GStreamer 카메라 파이프라인 오류

```bash
# GStreamer 플러그인 확인
gst-inspect-1.0 nvarguscamerasrc
# 없으면 JetPack 재설치 필요
```
