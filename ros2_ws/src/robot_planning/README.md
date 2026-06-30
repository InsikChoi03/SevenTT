# robot_planning

미션 플래닝(FSM) + 타겟 선택 + 경로 생성. `ament_python` 빌드.

## 노드 (계획)

| 노드 | 역할 |
|---|---|
| `mission_fsm_node` | 전체 상태 머신 (SCAN → APPROACH → ALIGN → CLASSIFY → PICK → ... → DUMP_ALL) |
| `target_selector_node` | 점수·거리·신뢰도 가중으로 다음 타겟 선정. Set2(20) > Set1(10) 우선 |
| `base_planner_node` | world model → 베이스 waypoint 생성, 메카넘 경로 계산 |
| `arm_planner_node` | 분석적 IK + 비주얼 서보잉 합성. 본체 cam 피드백으로 마지막 5~10cm 보정 |

## 규칙
- **오픽업 회피**: SigLIP 임계값 미만 → 패스, 좌표 블랙리스트
- **재방문 금지**: 블랙리스트는 영구
- **운반**: 도형 4 + 과일 3 = 7개 충족 시 보관함 이동 → 한 번에 쏟기
- **만점 100점**: 도형 1종 4개 + 과일 1종 3개
