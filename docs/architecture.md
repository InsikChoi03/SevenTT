# 시스템 아키텍처

## 1. 제약 조건 (룰북 v2.0)

- 모든 연산: Jetson Orin Nano 8GB 온디바이스
- 인식: 카메라 비전 필수
- 완전 자율 (물리 개입 1회당 -2점)
- 3분 / 만점 100점 = Set1×4(40) + Set2×3(60)
- **최대 리스크: 오픽업 -2배 감점** (Set2 오픽업 = -40점)

## 2. 프레임워크: ROS2 Humble

- 살림: `tf2`, `ros2 bag`, 노드 분리 디버깅
- **미채택**: MoveIt2 (MG996R 정밀도 부족), Nav2 (필드 작음), IMU (광각이 벽 기준 자세 잡음)

## 3. Localization (자기 위치 추정)

**전략**: 광각 카메라 + 휠 오도메트리 fuse, **IMU 없음**.

1. **절대 자세**: 광각이 경기장 벽/코너/깃발 인식 → (x, y, θ) 재계산 → 오차 리셋
2. **상대 이동 (Visual Odometry)**: 벽/깃발 안 보일 때
   - 검출된 객체 중심점 (이미 있음, 비용 0)
   - + 이미지 특징점 ORB/GFTT + 옵티컬 플로우
   - 두 anchor를 fuse (객체 가림 시 일반 특징점이 대체)
3. **선행작업 필수**: 광각 160° 왜곡 보정 (OpenCV `calibrateCamera`)

정밀도 목표: ±5~10cm. 픽업 정밀도는 본체 cam 비주얼 서보잉이 보정.

## 4. Perception (인식)

### 카메라 역할 분리
| 카메라 | 역할 | 모델 |
|---|---|---|
| 광각 (상단 80cm) | **탐지만** — 객체 위치, 종류 구분 X | YOLO-World (단순 프롬프트) |
| 본체 (그리퍼 손목) | **정밀 박스 + 분류** | YOLO-World (타겟 1종) + SigLIP + classical CV |

두 카메라 통합: **영상 비교 X, 좌표만 매칭** (tf2 + 호모그래피).

### YOLO-World 공유 인스턴스
- 광각 5~10 FPS, 단순 클래스 (예: `["object on floor"]`)
- 본체 20+ FPS, 단일 클래스 (예: `["apple"]`)
- **본체에도 YOLO-World 필수**: SigLIP은 분류만, 박스가 비주얼 서보잉에 필요
- 프롬프트 임베딩 캐시 → 전환 무비용

### SigLIP (본체 cam, 픽업 게이트)
- **과일 4종**: 사과/오렌지/바나나/파인애플 → **잘 작동**
- **도형 4종**: zero-shot 약함 → classical CV로 보조

### 도형 분류 보조 (classical CV)
```python
contour = cv2.approxPolyDP(largest_face, eps, True)
n_edges = len(contour)
# 4 → cube / 5 → dodecahedron / 3 → octahedron 또는 icosahedron
# 8면체 vs 20면체는 보이는 꼭짓점 수로 추가 분별
```
신뢰도 낮으면 **패스** (오픽업 -20점 vs 패스 0점).

### Set1 vs Set2 정육면체 구분
외형 동일 → **과일 그림 면 검출 여부**로 분기:
```python
discriminator = ["a white cube with fruit picture",
                 "a plain white cube without any image"]
```

### 메모리 예산
| 항목 | 메모리 |
|---|---|
| OS + ROS2 | ~2.0 GB |
| YOLO-World-S TRT FP16 (공유 1개) | ~1.2 GB |
| SigLIP-base TRT FP16 | ~0.4 GB |
| 카메라 버퍼 × 2 + 처리 | ~0.7 GB |
| FSM / 제어 / 통신 / VO | ~0.4 GB |
| **합계** | **~4.7 GB / 8 GB** (여유) |

## 5. Planning (FSM)

```
SCAN(광각) → SELECT_TARGET → APPROACH(베이스)
  → ALIGN(본체 cam, 비주얼서보) → CLASSIFY(SigLIP + 도형보조)
    ├─ 목표 → PICK → STORE_IN_TRAY → SELECT_TARGET
    └─ 비목표 → 좌표 기록(블랙리스트) → SELECT_TARGET
7개(도형 4 + 과일 3) 충족 → DRIVE_TO_STORAGE → ALIGN_OVER_BIN → DUMP_ALL → END
```

### 결정 규칙
- 타겟 우선순위: Set2(20점) > Set1(10점), 거리·confidence 가중
- **오픽업 회피**: SigLIP 임계값(예: 0.7) 미만 → 패스
- 패스 좌표 = 블랙리스트, 재방문 안 함
- 만점 100점 = 도형 1종 4개 + 과일 1종 3개

### 본체 트레이 운반 (확정)
- **7개 다 모은 후 1회 운반 + 한 번에 쏟기**
- 트레이는 객체 이탈 방지용 벽 필요
- 쏟기 메커니즘 미정 (트레이 기울임 / 하부 개폐 등 — 팔로 1개씩은 배제)
- 트레이 크기 제약: 8cm 객체 7개 수용 + 본체 40×40×40cm 한계 (룰북 4.1)
- 보관함 정렬 정확도 결정적 — 밖으로 튀어나가면 점수 X (룰북 3.2.4)

### 명시적 미채택
- **VLA**: 사전학습 모델 Franka/UR 기반, MG996R fine-tuning 비현실적
- **소형 LLM 플래너**: 결정당 2~5초 지연 누적 → 픽업 손실
- **생성형 VLM 분류**: SigLIP이 빠르고 결정적

## 6. Control

### 베이스 (메카넘 4WD)
- 역기구학: `(vx, vy, ω) → 4휠 속도`
- 위치: localizer 출력 + 휠 오도메트리 fuse
- Nav2 안 씀, 경량 waypoint follower

### 로봇팔 (Scipia A2T 6DOF, MG996R×6)
- 분석적 IK (Robotics Toolbox for Python 또는 GitHub 6DOF IK)
- **비주얼 서보잉 필수**: 본체 cam(eye-in-hand, 손목) 피드백으로 마지막 5~10cm 보정
- tf2: `arm_wrist → camera_body` static_transform 1개

## 7. ROS2 노드 구성

> ⚠️ §4 인식 스택 서술(YOLO-World + classical-CV 도형보조)은 **구버전**. 현재는 커스텀
> YOLOv8n(`models/cube.pt`, 4도형 검출+분류) + SigLIP(과일 제로샷)으로 대체됨.
> 실제 노드·토픽 배선 구상도는 **[node_graph.md](node_graph.md)** 참조(이 절은 요약).

```
[hardware]   (robot_hardware)
  camera_csi_node ×2     (camera_top 광각 sensor-id=1 / camera_body 본체 sensor-id=0)
  mcu_bridge_arm_node    (serial ↔ Arduino_arm+PCA9685)
  mcu_bridge_base_node   (serial ↔ Arduino_base, /base/wheel_odom 발행)

[perception] (robot_perception)
  yolo_detector_node     (커스텀 YOLOv8n cube.pt, 두 스트림 + /classification/shape 재발행)
  siglip_gate_node       (본체 cam 과일 제로샷 게이트)
  world_model_node       (2D 시맨틱 월드모델)
  localizer_node         (광각 VO + 휠오도 → field 포즈)
  # 폐기: yolo_world_node, shape_heuristic_node (디스크엔 남김, 미등록)

[planning]   (robot_planning)
  mission_fsm_node       (11-state FSM)
  target_selector_node

[control]    (robot_control)
  go_to_goal_node        (홀로노믹 P제어: /base/goal_pose → /base_command)
  base_controller_node   (mecanum 역기구학 → MCU)
  pick_sequencer_node    (/arm/pick_trigger → 픽 시퀀스)
  arm_controller_node    (분석 IK → MCU, /arm/gripper 동적 개폐)
```
