# AI Robot Challenge

자율 객체 인식·픽업·적재 로봇 — Jetson Orin Nano 8GB 단일 보드 온디바이스 추론.

## 디렉토리

| 경로 | 용도 |
|---|---|
| `ros2_ws/src/` | ROS2 패키지들 (perception, planning, control, hardware, interfaces, bringup) |
| `docs/` | 아키텍처/하드웨어/룰북 문서 |
| `models/` | YOLO-World·SigLIP 등 가중치 (gitignore, 별도 다운로드) |
| `data/` | rosbag·이미지 녹화 (gitignore) |
| `scripts/` | 셋업·진단·유틸리티 스크립트 |
| `logs/` | 런타임 로그 (gitignore) |

## 빠른 시작

```bash
# 한 번만: 환경 셋업 검증
bash scripts/env_check.sh

# 워크스페이스 빌드
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 카메라 단독 테스트 (CSI IMX219)
bash scripts/camera_test.sh

# 전체 시스템 실행
ros2 launch robot_bringup full_robot.launch.py
```

## 핵심 결정 사항

- **프레임워크**: ROS2 Humble (Ubuntu 22.04 + JetPack 6.x)
- **인식**: YOLO-World (open-vocab detection) + SigLIP (Set2 zero-shot 분류 게이트)
- **플래닝**: FSM (VLA/LLM 미채택)
- **제어**: 분석적 IK + 비주얼 서보잉 (MoveIt2/Nav2 미채택)
- **하드웨어**: Scipia A2T 6DOF (MG996R×6) + Arduino + PCA9685, 메카넘 베이스(미정), Waveshare IMX219 B0183

자세한 결정 근거: [docs/architecture.md](docs/architecture.md), [docs/hardware.md](docs/hardware.md)

대회 룰북: [docs/rulebook.md](docs/rulebook.md)
