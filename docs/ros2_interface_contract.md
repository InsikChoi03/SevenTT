# ROS2 Interface Contract — Localization / Perception / Judgment

이 문서는 위치추정·인식·판단 노드들이 **반드시** 따라야 하는 토픽/메시지/프레임/파라미터 계약이다.
모든 노드는 이 계약의 토픽 이름·메시지 타입을 글자 그대로 사용한다. 임의로 토픽명을 바꾸지 않는다.

## 좌표 프레임 (REP-103: x forward, y left, z up)

- `field` — 경기장 월드 프레임. 원점 = 보관함이 있는 **좌측 하단 코너**.
  x축 = 하단 벽을 따라 우측(+x, 출발구역 방향), y축 = 좌측 벽을 따라 위쪽(+y).
  경기장 범위 x∈[0,4], y∈[0,4] (m). 출발구역 ≈ (x 3.6–4.0, y 0–0.4), 보관함 ≈ (x 0–0.4, y 0–0.4).
- `base_link` — 로봇 중심. x 전방, y 좌측, z 상방.
- `camera_top` — 광각(sensor-id=0), 리프트로 base 위 ~0.8m, 아래로 기울어 바닥을 봄.
- `camera_body` — 본체 eye-in-hand(sensor-id=1), 팔 손목.
- `odom` — 오도메트리 누적 프레임.

tf 체인(생산 환경): `field → odom → base_link`, `base_link → camera_top`, `base_link → camera_body`.
**단, 노드들은 tf2 런타임에 의존하지 않고 동작 가능해야 한다.** 카메라 외부 파라미터는 노드 파라미터로 받는다
(아래 각 노드 파라미터 참조). `localizer_node`만 tf를 브로드캐스트한다(선택). `world_model_node`는 로봇 자세를
`/localization/pose`(PoseStamped, field)에서 받고, 광각 카메라 외부 파라미터는 파라미터로 받아 직접 투영한다.

## 토픽 (정확히 이 이름/타입 사용)

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/camera_top/image_raw` | `sensor_msgs/Image` (bgr8) | camera_top | yolo_world, localizer |
| `/camera_body/image_raw` | `sensor_msgs/Image` (bgr8) | camera_body | yolo_world, siglip_gate, shape_heuristic |
| `/camera_top/detections` | `robot_interfaces/DetectionArray` | yolo_world | world_model |
| `/camera_body/detections` | `robot_interfaces/DetectionArray` | yolo_world | siglip_gate, shape_heuristic |
| `/base/wheel_odom` | `std_msgs/Float32MultiArray` `[fl,fr,rl,rr]` (m/s) | mcu_bridge_base | localizer |
| `/localization/pose` | `geometry_msgs/PoseStamped` (frame=`field`) | localizer | world_model, mission_fsm |
| `/world_model` | `robot_interfaces/WorldModel` | world_model | target_selector, mission_fsm |
| `/world_model/blacklist_add` | `std_msgs/UInt64` (object id) | mission_fsm | world_model |
| `/selected_target` | `robot_interfaces/Object` | target_selector | mission_fsm |
| `/classification/siglip` | `robot_interfaces/Classification` | siglip_gate | mission_fsm |
| `/classification/shape` | `robot_interfaces/Classification` | shape_heuristic | mission_fsm |
| `/mission_state` | `robot_interfaces/MissionState` | mission_fsm | (모니터) |
| `/state_advance` | `std_msgs/Empty` | (수동/테스트) | mission_fsm |
| `/base/goal_pose` | `geometry_msgs/PoseStamped` (frame=`field`) | mission_fsm | base_planner(미래) |
| `/arm/pick_trigger` | `std_msgs/Bool` | mission_fsm | arm_planner(미래) |

## 메시지 필드 (robot_interfaces)

```
# Detection.msg
string label; float32 confidence
float32 x_center; float32 y_center; float32 width; float32 height   # 픽셀
string source_frame      # "camera_top" | "camera_body"

# DetectionArray.msg
std_msgs/Header header; Detection[] detections

# Object.msg  (world model 객체, field 프레임)
uint64 id; string class_label
uint8 set_type           # 0=unknown,1=set1(shape),2=set2(fruit),3=storage_flag
float32 x; float32 y     # m, field
float32 confidence
builtin_interfaces/Time last_seen
bool blacklisted

# WorldModel.msg
std_msgs/Header header; Object[] objects
float32 robot_x; float32 robot_y; float32 robot_theta   # m, m, rad (field)

# MissionState.msg
string state
uint8 tray_shape_count; uint8 tray_fruit_count
uint64 current_target_id
builtin_interfaces/Time stamp

# Classification.msg
std_msgs/Header header
string label; uint8 set_type; float32 confidence
bool is_target; bool image_face_visible; string source   # "siglip"|"shape_heuristic"
```

## 룰북 제약 (판단에 직접 반영)

- 오픽업 = 해당 점수 2배 감점 (Set2 = -40점). **분류 신뢰도 < 임계값 → 패스(블랙리스트)**. 패스는 0점.
- Set1 정육면체 ↔ Set2 정육면체: 외형 동일. **과일 그림 면 검출 여부(image_face_visible)로 분기.**
  - Set2 타겟(과일): siglip이 그 과일 + image_face_visible=true + conf≥thresh → PICK.
  - Set1 타겟(cube): shape=cube + siglip image_face_visible=false → PICK.
  - 그 외 → PASS.
- 만점 100점 = Set1 타겟 4개 + Set2 타겟 3개 = 7개. 7개 모이면 보관함 운반 후 한 번에 쏟기.
- 완전 자율. 텔레옵/원격연산 금지. 모든 추론은 Orin Nano 온디바이스.

## 코드 스타일 (기존 노드와 일치)

- 첫 줄 모듈 docstring, 그 다음 `from __future__ import annotations`.
- 표준 라이브러리 → 서드파티 → ros 메시지 순 import (기존 노드 참고).
- 모든 설정은 `self.declare_parameter(...)` + `self.get_parameter(...).value`.
- 타입힌트 사용. 노드 클래스명 `XxxNode(Node)`, `super().__init__("xxx_node")`.
- 표준 `main(args=None)`:
  ```python
  def main(args=None) -> None:
      rclpy.init(args=args)
      node = XxxNode()
      try:
          rclpy.spin(node)
      except KeyboardInterrupt:
          pass
      finally:
          node.destroy_node()
          if rclpy.ok():
              rclpy.shutdown()

  if __name__ == "__main__":
      main()
  ```
- 로그: `self.get_logger().info/warn(...)`. 반복 경고는 `throttle_duration_sec=`.

## Dry-run 요구 (하드웨어/가중치 없이 기동 가능해야 함)

- 무거운 라이브러리(torch, ultralytics, transformers)는 **try/except로 임포트**하고, 실패 시
  `get_logger().error(...)`로 경고 후 노드는 살아있되 추론은 스킵(콜백에서 일찍 return). import 실패로 프로세스가 죽지 않아야 한다.
- 모델 가중치 경로가 없거나 로드 실패 시에도 노드는 기동(생성자에서 예외로 죽지 않음). 추론만 비활성화하고 주기적으로 1회 경고.
- 카메라/시리얼이 없어도 노드 생성·토픽 광고는 성공해야 한다(콜백이 안 불릴 뿐).
- GPU 추론은 가능하면 CUDA, 없으면 CPU로 폴백.

## 패키지/엔트리포인트 (작성자는 파일만, 등록은 통합 단계에서 처리)

- 위치추정·인식 노드 → `robot_perception/robot_perception/nodes/<name>.py`, 패키지 `robot_perception`.
- 판단 노드 → `robot_planning/robot_planning/nodes/<name>.py`, 패키지 `robot_planning`.
- 각 파일은 `def main(args=None)`를 노출(entry point = `<module>:main`).
