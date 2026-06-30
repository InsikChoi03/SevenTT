# robot_interfaces

대회 로봇의 커스텀 ROS2 메시지·서비스 정의. `ament_cmake` 빌드.

## 메시지 (계획)

| 메시지 | 용도 |
|---|---|
| `Detection.msg` | YOLO-World 단일 검출 결과 (클래스, 박스, 신뢰도) |
| `DetectionArray.msg` | 검출 결과 배열 + 헤더 |
| `Object.msg` | world model의 트래킹된 객체 (필드 좌표, 클래스, 블랙리스트 여부) |
| `WorldModel.msg` | `Object[]` + 로봇 pose |
| `ArmCommand.msg` | 6관절 목표각 `<θ1...θ6>` |
| `BaseCommand.msg` | `(vx, vy, ω)` 또는 휠 속도 |
| `MissionState.msg` | FSM 현재 상태 + 트레이 적재 카운트 |

## 서비스 (계획)

| 서비스 | 용도 |
|---|---|
| `SetTargets.srv` | 당일 오전 공지된 목표(도형 1종, 과일 1종) 설정 |
| `CalibrateCamera.srv` | 광각 캘리브레이션 트리거 |
