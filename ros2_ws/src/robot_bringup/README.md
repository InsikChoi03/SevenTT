# robot_bringup

전체 시스템 launch 파일 모음. `ament_python` 빌드.

## launch 파일 (계획)

| 파일 | 용도 |
|---|---|
| `full_robot.launch.py` | 모든 노드 한 번에 (실전 경기용) |
| `camera_only.launch.py` | 두 카메라만 → 토픽 확인/녹화용 |
| `perception_only.launch.py` | 카메라 + 인식 노드 (디버깅) |
| `arm_test.launch.py` | 팔 단독 (서보잉 캘리브레이션) |
| `base_test.launch.py` | 베이스 단독 (메카넘 동작 확인) |

## config 파일

| 파일 | 용도 |
|---|---|
| `targets.yaml` | 당일 공지된 목표 (도형 1종, 과일 1종) |
| `field.yaml` | 4×4m 경기장 좌표계, 출발/보관함 영역 |
| `cameras.yaml` | 광각/본체 카메라 캘리브레이션 + tf2 정적 변환 |
