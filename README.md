# SevenTT - Vision Robot

젯슨 오린 나노를 활용하여 실시간으로 객체를 탐지하고 모터를 제어하는 비전 로봇 프로젝트입니다.

## 개발 환경

| 항목 | 버전 |
|------|------|
| 기기 | NVIDIA Jetson Orin Nano |
| JetPack | 6.x 이상 |
| Python | 3.x |
| OpenCV | CUDA 빌드 (JetPack 내장) |
| PyTorch | Jetson 전용 wheel |

## 빠른 시작

```bash
# 1. 레포 클론
git clone <repo-url>
cd SevenTT

# 2. 환경 세팅 (젯슨에서 최초 1회 실행)
chmod +x setup.sh
./setup.sh

# 3. 가상환경 활성화
source venv/bin/activate

# 4. 모델 다운로드
python scripts/download_models.py

# 5. 추론 실행
python scripts/run_inference.py
```

## 프로젝트 구조

```
SevenTT/
├── config/          # 모델 및 하드웨어 설정 (YAML)
├── src/             # 핵심 소스코드
│   ├── detection/   # TensorRT 추론 모듈
│   ├── control/     # 모터 제어 모듈
│   ├── camera/      # 카메라 입력 모듈
│   └── utils/       # 공통 유틸리티
├── models/          # 모델 파일 (Git 미추적 → README 참고)
├── data/            # 데이터셋 (Git 미추적)
├── scripts/         # 실행 및 변환 스크립트
├── tests/           # 단위 테스트
└── docs/            # 팀 문서
```

## 문서

- [젯슨 환경 세팅](docs/jetson_setup.md)
- [모델 변환 가이드](docs/model_conversion.md)
- [협업 워크플로우](docs/collaboration.md)
- [모델 파일 관리](models/README.md)
