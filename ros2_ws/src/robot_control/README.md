# robot_control

저수준 모션 제어. `ament_python` 빌드.

## 노드 (계획)

| 노드 | 역할 |
|---|---|
| `base_controller_node` | 메카넘 역기구학 `(vx, vy, ω) → 4휠 속도` → `/base_command` 발행 |
| `arm_controller_node` | 관절각 인터폴레이션 + 안전 제한 → `/arm_command` 발행 |

## 라이브러리 사용
- **분석적 IK**: Robotics Toolbox for Python 또는 GitHub 공개 6DOF IK 가져오기. 링크 길이는 실측 → `config/arm_dimensions.yaml`
- **메카넘 IK**: 직접 구현 (4×4 행렬)

## MG996R 한계 대응
- 정밀도 ±2° → 비주얼 서보잉(robot_perception → arm_planner)에 의존
- 관절 속도 제한 / soft limit / 통신 끊김 시 안전 정지
