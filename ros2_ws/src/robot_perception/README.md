# robot_perception

인식 + localization 노드. `ament_python` 빌드.

## 노드 (계획)

| 노드 | 역할 |
|---|---|
| `yolo_world_node` | YOLO-World 단일 인스턴스, 두 스트림 처리. 프롬프트 토글로 광각(탐지만)/본체(타겟 분류) 분리 |
| `siglip_gate_node` | 본체 cam crop → 4-class zero-shot 분류 게이트 (Set2 -40점 방지) |
| `shape_heuristic_node` | classical CV로 도형 4종 보조 분류 (`approxPolyDP` 면 변 수) |
| `localizer_node` | 광각 → 벽/코너/깃대 absolute pose + 객체·특징점 VO + 휠 오도메트리 fuse |
| `world_model_node` | 객체·로봇·블랙리스트 통합 상태 유지 |

## 분담
- **광각**: 위치만 (탐지) — 프롬프트 단순
- **본체**: 정밀 박스 + 분류 — 프롬프트 좁고 SigLIP 추가

## 좌표
모든 검출은 `tf2`로 `field` 프레임으로 변환. 두 카메라 영상 비교 X, 좌표만 매칭.
