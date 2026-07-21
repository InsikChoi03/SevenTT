# PROJECT LOG (변경 이력 · 작업자 명단 · 다음 할 일)

> 이 문서는 여러 명이 함께 작업하는 이 저장소의 공동 작업 기록입니다.
> 사람은 코드를 직접 작성하지 않으며, 모든 작성/수정은 각자의 AI가 수행합니다.
> AI는 작업 전 이 문서를 읽고, 작업 후 아래 규칙에 따라 이 문서를 갱신합니다.
> 대상 브랜치: `new`

---

## 1. 버전 관리 규칙 (Naming)

**버전 형식:** `vMAJOR.MINOR.PATCH` (예: `v1.3.2`)

| 자리 | 이름 | 올리는 경우 | 예시 |
|------|------|-------------|------|
| MAJOR | 대규모/호환성 파괴 | 구조 전체 변경, 이전 결과물과 호환 안 됨 | `v1.0.0 → v2.0.0` |
| MINOR | 기능 추가 | 새 기능·새 파일·새 모듈 추가 (기존은 유지) | `v1.2.0 → v1.3.0` |
| PATCH | 소규모 수정 | 버그 수정, 오타, 리팩터링, 문서 보완 | `v1.3.1 → v1.3.2` |

**엔트리 ID:** `YYYY-MM-DD-NN` (날짜 + 그날의 순번 2자리). 예: `2026-07-06-01`

**산출물 파일 네이밍(필요 시):** `<프로젝트약칭>_<모듈>_v<버전>.<확장자>`
→ 예: `4WD_encoder_v0.2.0.txt`. 이전 버전 파일은 지우지 않고 그대로 둔다.

---

## 2. 현재 상태 (Snapshot — 이 섹션만 최신값으로 덮어쓰기)

- **현재 버전:** `v4.1.0` (`new` 브랜치 릴리스 커밋 `cc3bb25`, Git 태그 없음)
- **최종 업데이트:** 2026-07-21 17:05 (KST)
- **최종 작업자:** `@AI`
- **한 줄 요약:** 메인 경기를 Z1~Z4 dodecahedron 전용 흐름으로 전환하고, 직교 Lane heading lock·stand-off 접근과 MCU/실행 종료 안전을 추가한 v4.1.0을 확정했다.

---

## 3. 작업자 명단 (Collaborators)

> 새 작업자가 합류하면 이 표에 **추가**한다(기존 행 삭제 금지). 핸들(`@`)은 변경 이력에서 작업자 식별용.

| 핸들 | 이름 | 역할 | 비고 |
|------|------|------|------|
| `@junsu` | 김준수 | (역할 미정) | |
| `@jeongha` | 신정하 | (역할 미정) | |
| `@yuchang` | 이유창 | (역할 미정) | |
| `@inho` | 최인호 | (역할 미정) | |
| `@insik` | 최인식 | (역할 미정) | repo 생성자 |
| `@junyoung` | 허준영 | (역할 미정) | |
| `@seungmin` | 양승민 | (역할 미정) | |
| `@AI` | AI 어시스턴트 | 실제 작성/수정 담당 | 사람 대신 코드·문서 작성 |

*(역할 칸은 팀에서 정해지는 대로 채워 넣으세요.)*

---

## 4. 다음 할 일 (TODO / Next Actions)

> 완료 시 `[x]` 체크(삭제하지 말 것). 담당자는 `@핸들`로 지정.

- [x] 공동 작업 기록 문서(`PROJECT_LOG.md`) 최초 생성 및 작업자 7명 등록 — `@AI` (2026-07-06 완료)
- [x] 이 문서를 `new` 브랜치 루트에 커밋·push 하고, 팀원 전원 pull 받기 — `@insik` (2026-07-13 완료)
- [ ] 각 작업자 역할(제어/인식/하드웨어/문서/통합 등) 확정해 명단 표에 채우기 — 팀 전체
- [x] 첫 실제 코딩 작업 시 이 로그에 엔트리가 규칙대로 잘 추가되는지 검증 — `@AI` (2026-07-06 완료)
- [ ] 프로젝트 약칭/모듈 명명 규칙 팀 합의 (산출물 파일명용) — `@seungmin`
- [x] 아두이노 베이스 펌웨어에 엔코더 ODOM/ENC 출력과 ENCZERO 처리 추가 — `@AI` (2026-07-06 완료)
- [x] `scripts/drive_tests/encoder_distance_test.py` 작성 및 직진/후진/회전 serial 테스트 경로 정리 — `@AI` (2026-07-06 완료)
- [x] 직진 60cm 보정값 확보 (`forward target-m 0.133`, `backward target-m 0.146`) — `@AI` (2026-07-06 완료)
- [x] 시계/반시계 90도 회전 1차 보정값 확보 (`cw target-deg 69.3`, `ccw target-deg 101.0`) — `@AI` (2026-07-06 완료)
- [x] 횡이동 최소 구동 속도와 대각선 드리프트 보정값 재측정 — `@insik` (2026-07-07 1차 완료)
- [ ] `lateral-kick` 모드로 오른쪽/왼쪽 횡이동 시작성, 실제 이동 거리, 드리프트를 실측 기록 — `@insik`
- [ ] `forward-then-lateral` 모드로 전진 10cm 후 오른쪽/왼쪽 10cm 횡이동 성공 여부와 최종 위치 오차 기록 — `@insik`
- [ ] `forward_right`/`forward_left` 단독 대각선 이동의 실제 각도, 거리, 회전 틀어짐 기록 — `@insik`
- [ ] 60cm 정사각형 시계방향 둘레 주행 후 시작점 오차 기록 — `@insik`
- [x] 직진/회전 보정값을 프리셋 또는 별도 실행 스크립트로 정리 — `@AI` (2026-07-07 통합 튜닝 스크립트로 1차 정리)
- [ ] IMU 결합 시 회전 제어 로직 및 보정 절차 재설계 — `@AI`
- [ ] 실제 주행에서 `<HB>`가 `/base/wheel_odom`으로 들어오지 않고 `<ODOM>`만 pose에 반영되는지 확인 — `@insik`
- [ ] `/localization/is_stationary`를 정지/주행 상태에서 echo해 정지 판정 지연과 오검출 여부 확인 — `@insik`
- [ ] FL 엔코더 배선/핀 충돌을 복구한 뒤 `wheel_odom_enabled_wheels`를 `[true, true, true, true]`로 되돌리고 4륜 odom 재튜닝 — `@insik`
- [x] ROS 경로(`/base_command`→`base_controller_node`→`mcu_bridge_base_node`)에서 open-loop 엔코더 거리 비교 테스트 파일 작성 — `@AI` (2026-07-09 완료)
- [x] 상위 엔코더 제어 검증용 반복성/목표거리 정지 테스트 파일 작성 — `@AI` (2026-07-09 완료)
- [ ] `encoder_control_validation.py`의 `repeat-open-loop`로 같은 조건 3~5회 반복해 엔코더거리/실제거리/드리프트 기록 — `@insik`
- [ ] 반복성 보정비가 ±15% 안에 들어오는 조건을 찾은 뒤 `odom_scale` 확정 — `@insik`
- [ ] `encoder_control_validation.py`의 `closed-loop-distance`로 엔코더 목표거리 30cm 정지 테스트 수행 — `@insik`
- [ ] 엔코더 목표거리 정지 테스트가 안정화되면 `mission_fsm_node` opening 이동을 시간 기반에서 거리 기반으로 전환 검토 — `@AI`
- [ ] `motion_tune.py`로 전진/후진/횡이동/회전 최종 튜닝값 확정 — `@insik`
- [ ] 최종 튜닝값 확정 후 실제 경기 주행용 함수/모듈 작성 — `@AI`
- [x] 오늘 경기장 run 광각 사진 기준 벽/바닥 접선 기반 위치 보정 노드 실행 연결 — `@AI` (2026-07-08 완료)
- [ ] 실제 경기장 주행 중 `wall_localizer_node` 로그의 `wall fix dx/dy/dth`를 확인하고 `wall_gate_m`, `axis_tol_deg`, `correction_conf` 최종 튜닝 — `@insik`
- [ ] X-AnyLabeling으로 `data/wallseg_export/images` wide 이미지의 `wall_floor_boundary` polygon 라벨링 완료 — `@insik`
- [x] 라벨링 완료 후 `scripts/colab_train_wallseg_v1.ipynb`로 `wall_floor_boundary_seg_v1.pt` 학습 — `@AI` (2026-07-09 1차 완료)
- [x] 학습된 `wall_floor_boundary_seg_v1.pt`를 `models/`에 복사하고 기존 `wide.pt`/`cube.pt`와 분리 보관 — `@insik` (2026-07-09 로봇 로컬 복사 완료, 모델 파일은 git 제외)
- [x] `wall_localizer_node.py`에 segmentation mask 후보 생성 hook과 edge fallback 연결 — `@AI` (2026-07-09 완료)
- [x] 추가 촬영한 벽 데이터 61장을 라벨링해 v2 dataset으로 합치고 재학습 여부 판단 — `@insik` (2026-07-10 v2 학습·로봇 복사 완료)
- [x] segmentation mask 내부 중심선/최종 벽 선분 선택 로직을 실제 화면에서 더 튜닝 — `@AI` (2026-07-10 mask median/PCA 선분 기준으로 전환)
- [ ] 저속 메인 경기 실행에서 벽 보정이 들어갈 때 파란 로봇 pose, 주황 raw 선분, 노란 snapped 선분이 함께 안정적으로 움직이는지 확인 — `@insik`
- [ ] 1/3 주행속도 설정에서 로봇이 스톨하면 `wheel_min`, `wheel_min_rot`, `wheel_boost`를 최소한으로 재상향 튜닝 — `@AI`
- [ ] 팔 제어 보드 연결 후 `/dev/ttyUSB*` 또는 `/dev/ttyACM*` 포트 인식 확인하고 집기-보관 1회 재실행 — `@insik`
- [ ] CH340 장치에 대해 `udev`/`devtmpfs` 상태를 점검해 `/dev/ttyUSB0` 노드가 생성되지 않는 원인을 해결 — `@insik`
- [ ] `arm_controller_node`가 참조하는 `arm_ik.WRIST_ROLL_HOME` 누락을 정리해 메인 경기 런치 크래시를 제거 — `@AI`
- [x] 종료지점 태극기 깃발 데이터 수집용 WASD 촬영·pre-label JSON export 도구 작성 — `@AI` (2026-07-11 완료)
- [ ] `flag_capture_export.py`로 실제 경기장 종료지점 태극기 깃발 body/wide 이미지 촬영 후 X-AnyLabeling에서 `taeguk_flag` 박스 교정 — `@insik`
- [ ] 교정된 깃발 데이터와 기존 body/wide 누적 학습셋을 6클래스 순서로 병합하고 Colab에서 `cube_v7_flag`/`wide_v6_flag` 재학습 — `@AI`
- [ ] 새 `models/cube.pt`, `models/wide.pt` 배포 후 `/world_model`에서 `taeguk_flag`가 `set_type=3`으로 들어오는지 실제 카메라에서 확인 — `@insik`
- [x] 종료지점 클래스명을 `taeguk_flag`에서 `arrival`로 변경하고 런타임/학습 문서/노트북 클래스 순서를 동기화 — `@AI` (2026-07-11 완료)
- [x] `flag_capture_export.py` 촬영 중 MJPEG 라이브 화면 송출, 제자리 회전 키, 10cm 이동 기본값을 추가 — `@AI` (2026-07-11 완료)
- [ ] 촬영·교정 완료한 arrival JSON zip을 `labelme_to_yolo.py`로 body/wide YOLO 데이터셋으로 변환 — `@insik`
- [ ] 변환한 arrival 데이터와 `dataset_body_cube_v6.zip`/`dataset_wide_v5.zip`을 병합해 `dataset_body_flag_v7.zip`/`dataset_wide_flag_v6.zip` 생성 — `@AI`
- [ ] Colab에서 6클래스 `cube_v7_flag`/`wide_v6_flag`를 새로 학습하고 best.pt를 버전 파일로 백업 후 실전 모델 교체 검증 — `@insik`
- [x] 월드모델에 7x6 격자 기반 물체 위치 soft prior 보정과 런타임 on/off 파라미터 추가 — `@AI` (2026-07-11 완료)
- [x] 로봇 정지 상태에서 격자 보정 모델과 라이브 맵 송출을 함께 실행하는 테스트 스크립트 작성 — `@AI` (2026-07-11 완료)
- [ ] 실제 경기장 기준으로 `grid_origin_x_m`, `grid_origin_y_m`, field 축 방향을 보정하고 격자점 42개가 실물 점과 겹치는지 확인 — `@insik`
- [ ] `scripts/grid_prior_live_test.py`로 정지 테스트를 실행해 raw 검출 위치와 격자 보정 후 track 위치가 안정적인지 비교 — `@insik`
- [ ] 격자 보정 안정성이 확인되면 실전 실행 config에서 `grid_prior_enabled` 기본값을 켤지 결정 — `@AI`
- [ ] body 카메라 각도 변경 후 `body_ground.npz`를 재캘리브하고 body 검출 기반 물체 거리/맵 표시가 맞는지 확인 — `@insik`
- [ ] 구역 미션 실주행에서 zone center 도착 후 6초 타이머와 구역 전환 로그가 의도대로 작동하는지 확인 — `@insik`
- [ ] 정이십면체 단독 목표 설정으로 실제 픽업 1회 성공 여부와 ALIGN 소요 시간을 확인 — `@insik`
- [ ] 각 zone 안에서 Set1/Set2를 모두 처리한 뒤 다음 zone으로 넘어가는 FSM 로그와 실제 동작을 확인 — `@insik`
- [ ] Set1 목표와 fruit cube 후보를 섞어 다음 zone 방향으로 훑는 one-stroke inspection route 점수식과 YAML 파라미터를 구현·튜닝 — `@AI`
- [ ] `motion_tuning.yaml` 수정 후 `scripts/apply_motion_tuning.py`로 `perception.yaml`/`test_field.yaml` 동기화가 의도대로 되는지 dry-run과 실제 적용을 비교 — `@AI`
- [ ] v1.0.0 기준선 복구 상태에서 실제 경기장 맵/자기위치 추정이 이전 안정 수준으로 돌아오는지 재검증 — `@insik`
- [ ] Set2/HSV/one-stroke inspection 기능은 v1.0.0 안정성 확인 후 별도 브랜치 또는 새 버전에서 단계적으로 재도입 여부 결정 — `@AI`
- [ ] `motion_tuning.yaml` 값 수정 후 메인 런치 재시작만으로 perception/planning/control 파라미터 override가 적용되는지 실제 경기 실행에서 확인 — `@insik`
- [x] 개막 타임라인(3초 정지→개막 이동→18초 release→SCAN)이 실제 경기 실행 로그와 로봇 움직임에서 맞는지 확인 — `@insik` (2026-07-19 완료)
- [x] 오프닝 이후 SCAN/anchor 이동에서 `/base_command`가 `vy=0` 회전/전진 명령으로 나오는지 실제 메인 런치에서 확인 — `@insik` (2026-07-19 완료)
- [x] FSM 단일 `/base_command` 경로로 전환한 뒤 SCAN/APPROACH 메카넘 실주행 정상 동작 확인 — `@insik` (2026-07-19 완료)
- [ ] v2.0.0 기준 전체 경기 반복 실행 3회 이상에서 단일 베이스 제어와 wide/body 카메라 자동 복구 회귀 확인 — `@insik`
- [ ] `lane_simplify_enabled`/`direct_fallback_enabled` 조합을 바꿔 레인-only와 레인 우선+직선 fallback 주행을 실주행 비교 — `@insik`
- [ ] `/localization/imu_motion_state`와 SigLIP square crop/prompt 보강이 pose 흔들림과 과일 분류 안정성에 주는 영향 확인 — `@insik`
- [ ] ALIGN adaptive step의 `align_mid_error_m`, `align_step_fwd_mid_sec`, `align_step_strafe_mid_sec`를 실제 물체 접근 거리별로 튜닝 — `@insik`
- [ ] Set2 FruitSlot 생성 기준(`set2_slot_birth_min_obs_wide`, `set2_slot_grid_lock_radius_m`)을 실제 경기장 오인식/누락 기준으로 재튜닝 — `@insik`
- [ ] `/planning/obstacles` 공유 후 물체 사이 APPROACH에서 좌우 떨림과 멈춤이 줄었는지 실주행으로 확인 — `@insik`
- [ ] body lost 후 같은 슬롯 재접근과 `align_retry_heading_timeout_sec`가 PICK 성공률을 높이는지 decisions 로그로 확인 — `@insik`
- [ ] checkpoint/anchor waypoint 테스트 경로와 실제 메인 경기 zone 순회 경로를 비교해 유지할 테스트 route를 확정 — `@AI`
- [ ] 새 wall segmentation 라벨은 벽면 전체 또는 floor-side edge 기준으로 다시 정하고, raw 주황선과 파란 base_link 상대거리가 맞는지 먼저 검증 — `@insik`
- [ ] 벽 보정 gain을 다시 켜기 전 `/localization/wall_map_transform`이 0 적용 상태에서 pose drift가 사라지는지 경기장 정지/주행 테스트로 확인 — `@insik`
- [ ] 엔코더 motion constraint 적용 후 `/localization/motion_mode`, `/localization/is_stationary`, `/base/wheel_odom`을 정지/스톨/전진/횡이동 상황에서 비교 기록 — `@insik`
- [ ] body 카메라 저조도 원인을 `scripts/lighting_compare.py`로 exposure/gain/조명 조건별 비교 후 카메라 파라미터를 확정 — `@insik`
- [ ] parking 테스트 패키지(`robot_parking_test`)를 실제 arrival 표식 촬영 데이터와 연결해 독립 검증할지 결정 — `@AI`
- [x] 오프닝을 전진→우측 횡이동→시계 45도 상대 회전으로 복원하고, 종료 후 벽 heading 및 x/y 수렴으로 초기 pose를 확정 — `@AI` (2026-07-20 완료, 사용자 실주행 확인)
- [ ] 벽 초기 pose 확정 이후 K1 앵커 주행에서 남아 있는 좌우 흔들림의 `/base_command`, Object flow pose, wall x/y 보정 기여도를 분리 측정하고 단순 LanePlanner 경로 주행을 안정화 — `@AI`, `@insik`
- [x] 독립 로컬 앵커 시험에 50cm 후보 inventory, IMU 누적 360도 제한, body 중앙 시각 heading 보정, 3Hz 중앙 우선 SigLIP 판정을 구현 — `@AI` (2026-07-20 완료)
- [x] 첫 타겟 과일 확정 시 남은 후보 조사를 취소하고 motion tuning 기반 ALIGN 후 실제 2R PICK_PLACE 1회를 실행해 종료하도록 연결 — `@AI` (2026-07-20 완료)
- [ ] `v4.0.0` 로컬 앵커 실주행에서 50cm inventory 중복 수, 시각 heading lock, body 좌표 ALIGN 및 실제 과일 1회 집기 성공 여부를 반복 검증 — `@insik`
- [x] 메인 경기 설정을 anchor/Set2 슬롯 대신 Z1~Z4 순회와 dodecahedron 4개 수집 전용 흐름으로 전환 — `@AI` (2026-07-21 완료)
- [x] zone별 복수 진입 anchor 중 현재 pose에서 가장 가까운 후보를 고정하고 라이브 맵에 선택 후보를 표시 — `@AI` (2026-07-21 완료)
- [x] 직교 Lane heading lock, 물체 stand-off 경로, 정지 후 heading 안정화와 IMU 폐루프 body 재탐색 제어를 구현 — `@AI` (2026-07-21 완료)
- [x] MCU PCA9685 FULL_OFF·명시적 STOP·프로파일 강제 OFF와 메인 실행기 프로세스/카메라 preflight 종료 안전을 강화 — `@AI` (2026-07-21 완료)
- [ ] `v4.1.0` 메인 실주행에서 Z1~Z4 진입 anchor 선택, 2초 fresh 관측, dodecahedron stand-off·ALIGN·PICK과 zone 전환을 반복 검증 — `@insik`
- [ ] `opening_enabled=false`와 RUNNING 버튼 즉시 D13 2초 ON 조합이 최종 경기 시작 의도와 일치하는지 확인 — `@insik`, `@AI`
- [ ] 직진 wheel scale `[1.0, 0.90, 0.75, 1.0]`과 Lane heading lock이 실제 주행의 우측 편향·좌우 흔들림을 줄이는지 측정 — `@insik`

---

## 5. 변경 이력 (Changelog) — ⛔ 삭제 금지 / 최신 항목이 맨 위

<!-- 새 엔트리는 바로 이 줄 아래에 추가하세요. 기존 엔트리는 건드리지 마세요. -->

### `v4.1.0` — 2026-07-21 17:05 (KST) · 작업자: `@AI` · ID: `2026-07-21-01`
- **변경 요약:** 메인 경기를 Z1~Z4 dodecahedron 전용 미션으로 전환하고 직교 Lane heading·stand-off 접근, 로컬 앵커 회전 안정화와 MCU/실행 종료 안전을 통합했다.
- **상세:**
  - `anchor_mission`과 기존 Set2/FruitSlot 경로를 끄고 `zone_mission`, zone filter, zone anchor 이동과 2초 fresh 관측을 켜 Z1→Z4 구역에서 dodecahedron 최대 4개만 처리하도록 경기 설정을 변경했다.
  - 슬롯이 없는 zone 모드에서도 dodecahedron track이 생성되도록 world model의 slot 필수 birth gate를 해제하고, zone별 복수 anchor 중 진입 시 현재 pose에서 가장 가까운 후보 하나를 고정하도록 했다.
  - 라이브 맵에 모든 zone anchor 후보와 현재 선택된 후보를 구분해 표시하고, Set2 슬롯 오버레이는 숨겨 현재 구역·경로 판단을 직관적으로 확인할 수 있게 했다.
  - LanePlanner에 직교 endpoint connector와 물체 앞 stand-off 계획을 추가하고, 메인 FSM에 field heading 고정, 원형 heading 필터, 지속 이탈 재정렬, 방향 반전 settle과 직교 구간 주행을 연결했다.
  - APPROACH 목표·heading을 latch하고 stand-off 도착 후 brake/settle 및 연속 heading 수렴을 거쳐 ALIGN으로 진입하며, body target 재탐색 회전도 시간 기반이 아닌 IMU heading 폐루프로 변경했다.
  - 로컬 앵커 회전은 목표에서 멀 때 안정 출력으로 연속 회전하고 목표 7.5도 이내에서 정지한 뒤 IMU/body를 검증하며, 실패할 때만 안정 출력의 짧은 보정 펄스를 사용하도록 바꿨다.
  - 순수 시계/반시계 회전에 별도 바퀴 scale을 적용하고 낮은 회전 명령이 일반 deadband에서 제거되지 않도록 base controller를 보완했으며, 직진 실측 편향 보정 scale을 설정했다.
  - Arduino PCA9685 정지를 `FULL_OFF`로 전환하고 전 채널 반복 정지와 `<STOP>`을 추가했으며, Jetson bridge가 비-RUNNING에서 모터 frame을 반복 전송하지 않고 종료 시 zero·STOP·프로파일 OFF를 flush하도록 강화했다.
  - 두 번째 시작 버튼의 RUNNING 전환에서 D13 프로파일을 2초 구동하고 비-RUNNING·부팅·종료 시 LOW를 반복 보장하도록 펌웨어와 bridge의 profile 소유권을 동기화했다.
  - 메인 실행기가 이전 launch process group과 stale MCU bridge를 먼저 종료하고 Argus 해제 뒤 wide camera preflight를 최대 3회 재시도하며, Ctrl-C에서 actuator·rosbag·launch 자식을 단계적으로 정리하도록 개선했다.
  - 단독 팔 점검에 `--force-running`과 `stow`를 추가하고 PWM/servo 진단 스크립트를 신설해 통합 펌웨어의 RUNNING gate 안에서 개별 채널을 검사할 수 있게 했다.
  - 로컬 앵커·LanePlanner·heading lock·stand-off·zone anchor·base deadband 관련 테스트 73개, Python/Bash 문법 검사와 `git diff --check`를 통과했다.
  - `new` 브랜치에 코드 릴리스 커밋 `cc3bb25`(`release: add zone lane navigation and actuator safety v4.1.0`)을 생성했다.
- **변경 파일:**
  - `firmware/base_arm_combined/base_arm_combined.ino` / 수정
  - `ros2_ws/src/robot_bringup/config/local_anchor_test.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/motion_tuning.yaml` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/base_controller_node.py` / 수정
  - `ros2_ws/src/robot_control/test/test_base_controller_deadband.py` / 수정
  - `ros2_ws/src/robot_hardware/robot_hardware/nodes/mcu_bridge_base_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/recognition_viz_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/world_model_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/lane_planner.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/local_anchor_fsm.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/local_anchor_test_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/local_anchor_web_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `ros2_ws/src/robot_planning/test/test_align_heading_control.py` / 신규
  - `ros2_ws/src/robot_planning/test/test_approach_standoff_control.py` / 신규
  - `ros2_ws/src/robot_planning/test/test_lane_heading_lock.py` / 신규
  - `ros2_ws/src/robot_planning/test/test_lane_planner.py` / 수정
  - `ros2_ws/src/robot_planning/test/test_local_anchor_fsm.py` / 수정
  - `ros2_ws/src/robot_planning/test/test_zone_anchor_candidates.py` / 신규
  - `scripts/arm_pick2r.py` / 수정
  - `scripts/arm_pwm_diag.py` / 신규
  - `scripts/arm_servo_diag.py` / 신규
  - `scripts/run_test_field.sh` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** zone 전용 미션·복수 anchor·Lane heading/stand-off·MCU 종료 안전 구현을 완료 처리하고, 실제 zone 주행 검증, opening/D13 시작 정책 확인과 wheel scale 편향 측정 항목을 추가했다.
- **버전 근거:** 독립 로컬 앵커 v4.0.0을 유지하면서 메인 미션에 새 zone 전용 실행 모드, 복수 진입 anchor, 직교 Lane heading 및 stand-off 접근, 새 진단·테스트와 하드웨어 종료 안전 기능을 추가했으므로 하위 호환을 유지하는 기능 릴리스인 `MINOR` 버전 `v4.1.0`으로 올렸다. 저장소의 기존 방식에 따라 별도 Git 태그는 생성하지 않았다.

### `v4.0.0` — 2026-07-20 21:27 (KST) · 작업자: `@AI` · ID: `2026-07-20-03`
- **변경 요약:** 독립 로컬 앵커 시험을 50cm 단일 회전 탐색부터 첫 타겟 과일의 즉시 ALIGN·실제 집기·종료까지 수행하는 새 end-to-end 흐름으로 확장했다.
- **상세:**
  - 후보 탐색 반경을 40cm에서 50cm로 확장하고 웹 로컬 맵의 범위·눈금도 50cm 기준으로 동기화했다.
  - 물리 IMU 누적 회전량과 body 카메라 기반 global heading 보정을 분리해, 시각 보정이 후보 방향에는 반영되지만 실제 360도 한 바퀴 종료량을 줄이지 않도록 했다.
  - `fruit_photo_cube` confidence 0.60 이상이 body 영상 중앙 ±40px에 3프레임 연속 들어오고 IMU 목표와 20도 이내일 때만 후보 heading을 시각 lock하도록 제한했다.
  - 로컬 시험에서 SigLIP을 1.5Hz에서 3Hz로 높이고 중앙 crop을 우선해 한 개만 추론하며, 정면 안정화 0.75초와 동일 결과 2회 조건으로 판정 지연과 다른 과일 crop 혼선을 줄였다.
  - 첫 타겟 과일이 확정되면 남은 후보 route를 즉시 폐기하고 body 관측 기반 `ALIGN_SETTLE_INITIAL → ALIGN_MEASURE/PULSE`로 전환하도록 순서를 변경했다.
  - `motion_tuning.yaml`의 `grab_x=0.19m`, 전후·좌우 허용오차 0.02m, 전후/횡이동 duty 0.27/0.315, adaptive 중간 펄스 0.25/0.60초와 안정화 1.0초를 로컬 ALIGN에 반영했다.
  - ALIGN 완료 시 `/arm/pick_trigger=True`를 한 번 발행하고 기존 2R `PICK_PLACE` 시퀀스를 실행하며, 9초 동안 base를 잠근 뒤 첫 과일 하나만 `PICKED`로 확정하고 전체 시험을 종료하도록 했다.
  - 안전 실행에서는 팔과 실제 pick을 끄고, `bash scripts/run_local_anchor_test.sh live`에서만 `with_arm=true`, `pick_enabled=true`를 주입하도록 분리했다.
  - 로컬 웹에 visual hit/lock, heading correction, ALIGNING/PICKING/PICKED 상태를 표시하고 rosbag에 `/arm/pick_trigger`, `/arm2r/target`을 추가했다.
  - 핵심 FSM 23개 테스트, Python·Bash 문법, YAML/diff 검사와 ROS launch 인자 로딩을 통과했다.
  - `new` 브랜치에 코드 릴리스 커밋 `a7b5b77`(`release: add one-pick local anchor workflow v4.0.0`)을 생성했다.
- **변경 파일:**
  - `ros2_ws/src/robot_bringup/config/local_anchor_test.yaml` / 수정
  - `ros2_ws/src/robot_bringup/launch/local_anchor_test.launch.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/siglip_gate_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/local_anchor_fsm.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/local_anchor_test_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/local_anchor_web_node.py` / 수정
  - `ros2_ws/src/robot_planning/test/test_local_anchor_fsm.py` / 수정
  - `scripts/run_local_anchor_test.sh` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 50cm inventory·시각 heading lock·body ALIGN·실제 과일 1회 집기의 실주행 반복 검증 항목을 추가하고, 구현 완료 항목 2개를 `[x]` 처리했다.
- **버전 근거:** 기존 메인 경기 FSM의 앵커 이동을 단순 보완한 수준이 아니라 독립 상태기계 중심의 단일 회전 inventory, 복합 heading 추정, 중앙 우선 분류, 즉시 ALIGN 및 실제 pick 종료로 실행 구조와 시험 목적이 크게 바뀌었다. 사용자가 지정한 메이저 릴리스 번호에 따라 `v4.0.0`으로 확정했으며, 저장소의 기존 릴리스 방식대로 별도 Git 태그는 생성하지 않았다.

### `v2.1.0` — 2026-07-20 18:27 (KST) · 작업자: `@AI` · ID: `2026-07-20-02`
- **변경 요약:** 벽 기반 오프닝 초기화와 3단계 경기 시작 기능을 포함한 현재 작업 25개 파일을 `v2.1.0` 릴리스 커밋으로 확정했다.
- **상세:**
  - `new` 브랜치에 릴리스 커밋 `faddd69`(`release: add wall-based opening initialization v2.1.0`)을 생성했다.
  - 앞선 `2026-07-20-01` 엔트리의 벽 heading→x/y 초기 pose 확정, 오프닝 동작, Object flow 활성화 순서와 디버그/테스트 변경을 포함했다.
  - Arduino/Jetson 3단계 경기 시작 상태, 안전 명령 게이트, 프로파일 출력, preflight 및 실행 스크립트 변경도 같은 릴리스 범위에 포함했다.
  - 앵커 슬롯 inventory와 상대 격자 모듈 및 테스트, 인식/시각화/월드모델 게이트 변경을 함께 반영했다.
  - shell/Python 문법 검사, diff 검사, ROS 5개 패키지 build, launch 파싱과 기능 테스트 66개를 통과했다.
  - 사용자가 벽 기반 초기 위치 보정 정상 동작을 확인했지만 이후 앵커 주행의 좌우 흔들림은 미해결이므로 관련 TODO를 유지했다.
- **변경 파일:** `PROJECT_LOG.md` 포함 총 25개 파일 / 신규 9개·수정 16개 (`faddd69` 기준)
- **다음 할 일 반영:** 기존 K1 앵커 주행 좌우 흔들림 원인 분리 및 LanePlanner 안정화 TODO를 유지했다.
- **버전 근거:** `v2.0.0` 이후 경기 시작 상태기계, 오프닝 벽 기반 초기 pose 확정, 안전 게이트, 새 앵커 자료구조와 진단·테스트가 추가된 기능 릴리스이므로 `MINOR` 버전 `v2.1.0`으로 확정했다. 저장소의 기존 릴리스 방식에 따라 별도 Git 태그는 생성하지 않았다.

### `v2.1.0` — 2026-07-20 18:24 (KST) · 작업자: `@AI` · ID: `2026-07-20-01`
- **변경 요약:** 오프닝 종료의 고정 pose 주입을 없애고 벽 관측 수렴으로 초기 위치·방향을 확정하는 절차를 추가했다.
- **상세:**
  - 경기 오프닝을 기존 의도인 `전진 → 우측 횡이동 → 현재 heading 기준 시계방향 45도 회전` 순서로 복원했다.
  - `opening_end_x`, `opening_end_y`, `opening_end_theta`와 오프닝 종료 강제 pose reset을 제거해 오프닝 후 위치·방향을 미리 지정하지 않도록 했다.
  - 정지 상태에서 벽 heading raw 오차, 유효 선분 수, 각도 분산을 검증하고 `HEADING_ONLY` 보정을 먼저 수렴시킨 뒤에만 `TRANSLATION_ONLY` x/y 보정을 허용하도록 초기화 단계를 분리했다.
  - 벽 heading은 유효 선분 2개 이상, 약 2도 이내, 표준편차 약 5도 이내 조건을 연속 3프레임 만족해야 확정하며, x/y는 양축 관측 confidence 0.9 이상과 각 축 잔차 4cm 이내를 연속 3프레임 만족해야 확정한다.
  - 상대 격자 회전에는 물리 IMU 증분만 사용해 global wall heading correction이 로봇의 실제 회전으로 중복 반영되지 않도록 분리했다.
  - 초기 pose 확정 전에는 물체 map과 Object flow를 차단하고, 벽 기반 heading 및 x/y가 모두 수렴한 뒤 새 물체 프레임부터 활성화하도록 했다.
  - 벽 heading 부호 검증 도우미와 `관측 벽 +10도 → 적용 보정 -10도` 단위 테스트를 추가하고 `/localization/wall_heading_debug`에 raw/filtered/applied 값, 선분 수, 각도 분산, confidence, 적용 모드를 발행하도록 했다.
  - 초기 위치 보정은 사용자가 실제 실행에서 정상 동작한다고 확인했다.
  - 오프닝 이후 앵커 주행은 상대 격자 추종을 끄고 고정 LanePlanner 경로를 한 번 계산해 유지하도록 변경했으나, 실제 주행의 좌우 흔들림은 여전히 남아 있어 완료로 판단하지 않았다.
  - Python compile, 관련 ROS 패키지 build, launch 파싱과 관련 기능 테스트 63개를 통과했다.
- **변경 파일:**
  - `ros2_ws/src/robot_bringup/config/motion_tuning.yaml` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/localizer_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/wall_localizer_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/wall_heading.py` / 신규
  - `ros2_ws/src/robot_perception/test/test_wall_heading.py` / 신규
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 오프닝 후 벽 기반 초기 pose 확정 항목을 완료 처리하고, K1 앵커 주행의 좌우 흔들림에서 명령·Object flow·wall correction 기여도를 분리해 LanePlanner 주행을 안정화하는 항목을 추가했다.
- **버전 근거:** 고정 오프닝 종료 pose를 벽 관측 기반 단계적 초기 위치·방향 확정 구조로 교체하고 새 진단 토픽·부호 테스트를 추가한 기능 확장이므로 `MINOR` 버전을 올려 `v2.1.0`으로 기록했다. 이 버전은 현재 문서/작업 트리 기준이며 Git 커밋·태그는 아직 없다.

### `v2.0.0` — 2026-07-19 17:28 (KST) · 작업자: `@AI` · ID: `2026-07-19-02`
- **변경 요약:** 경기 메카넘 제어를 FSM 단일 명령 경로로 재구성하고 source/install 및 카메라 복구 구조를 안정화했다.
- **상세:**
  - 메인 경기와 field-test launch에서 `go_to_goal_node`를 제거하고, `mission_fsm_node`가 모든 상태의 `/base_command`를 단독 발행하도록 변경했다.
  - FSM의 `/base/goal_pose` publisher와 우회 분기를 제거하고 OPENING, SCAN, APPROACH, 저장소 이동을 동일한 직접 회전·전진 명령 경로로 통일했다.
  - 호출되지 않던 구형 `_step_approach()`와 body-stop 보조 함수, 관련 미사용 파라미터를 제거해 실제 APPROACH 경로를 하나로 정리했다.
  - launch가 source YAML을 조건부로 직접 읽던 구조를 제거하고 설치된 `motion_tuning.yaml`만 사용하도록 해 Python 설치본과 설정 파일의 혼합 실행을 차단했다.
  - `setuptools 81` 환경에서 문제가 된 symlink build 대신 일반 colcon build로 `robot_planning`과 `robot_bringup` 설치본을 갱신하고 source/install 해시 일치를 확인했다.
  - `base_controller_node`에 런타임 파라미터 callback을 추가해 wheel minimum/boost/scale 변경이 캐시된 제어값에 즉시 적용되도록 했다.
  - 일반 제자리 회전 최소 출력과 부스트를 오프닝에서 검증된 안전 범위로 맞추고, standalone `go_to_goal_node`에는 상태별 yield 및 횡이동 제한 설정을 유지했다.
  - top/body CSI 카메라 노드에 timeout 기반 pipeline 재연결과 launch respawn을 추가해 간헐적인 wide no-frame 상태의 자동 복구 경로를 마련했다.
  - Python compile, YAML parse, 일반 colcon build, 관련 planning 테스트 21개, 런타임 publisher introspection을 통과했으며 사용자가 오프닝 이후 SCAN/APPROACH 실주행이 문제없이 동작함을 확인했다.
- **변경 파일:**
  - `ros2_ws/src/robot_bringup/config/motion_tuning.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_bringup/launch/bringup.launch.py` / 수정
  - `ros2_ws/src/robot_bringup/launch/cameras.launch.py` / 수정
  - `ros2_ws/src/robot_bringup/launch/perception.launch.py` / 수정
  - `ros2_ws/src/robot_bringup/launch/test_field.launch.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/base_controller_node.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/go_to_goal_node.py` / 수정
  - `ros2_ws/src/robot_hardware/robot_hardware/nodes/camera_csi_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 개막 타임라인, 오프닝 이후 회전·전진 명령, SCAN/APPROACH 메카넘 실주행 확인 항목을 완료 처리하고 v2.0.0 전체 경기 3회 반복 회귀 확인을 추가했다.
- **버전 근거:** 메인 경기의 `/base/goal_pose`·`go_to_goal_node` 제어 계약을 제거하고 FSM 단일 `/base_command` 구조로 전환해 기존 실행·파라미터 구성과 호환되지 않는 구조 변경이므로 `MAJOR` 버전을 `v2.0.0`으로 올렸다.

### `v1.7.1` — 2026-07-19 15:52 (KST) · 작업자: `@AI` · ID: `2026-07-19-01`
- **변경 요약:** 오프닝 이후 goal 이동에서 메카넘 순수 횡이동을 막고 회전 후 전진 주행으로 제한했다.
- **상세:**
  - 오프닝 동작은 `mission_fsm_node`가 `/base_command`를 직접 발행하고, 오프닝 이후 SCAN/anchor 이동은 `go_to_goal_node`가 `/base/goal_pose`를 `/base_command`로 변환한다는 제어 경로 차이를 확인했다.
  - `go_to_goal_node`에 `lateral_motion_enabled` 파라미터를 추가해 `false`일 때 waypoint 추종 중 `vy` 명령을 발행하지 않도록 했다.
  - lateral motion 비활성 시 목표가 옆/뒤에 있으면 먼저 `omega` 회전만 수행하고, 목표 방향을 충분히 바라본 뒤 `vx` 전진만 수행하도록 했다.
  - 경기 실행 튜닝에서 `lateral_motion_enabled: false`를 적용해 오프닝 이후 SCAN/goal 이동의 기본값을 회전/전진 기반으로 고정했다.
  - 일반 주행 stiction 보정을 상향하고 direct fallback을 켜서 레인 경로 실패 시 정지 hold 대신 목표 방향으로 주행을 시도하도록 유지했다.
- **변경 파일:**
  - `ros2_ws/src/robot_control/robot_control/nodes/go_to_goal_node.py` / 수정
  - `ros2_ws/src/robot_bringup/config/motion_tuning.yaml` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 오프닝 이후 SCAN/anchor 이동에서 `/base_command`가 `vy=0` 회전/전진 명령으로 나오는지 실제 메인 런치에서 확인하는 TODO를 추가했다.
- **버전 근거:** 오프닝 이후 주행 제어 방식의 버그성 동작을 막는 소규모 제어 수정과 튜닝 변경이므로 `PATCH` 버전 증가로 `v1.7.1`로 올렸다.

### `v1.7.0` — 2026-07-18 17:43 (KST) · 작업자: `@AI` · ID: `2026-07-18-01`
- **변경 요약:** 경기 실행용 구역/주행/개막 타이밍을 단순화하고, IMU 기반 보정 감쇠와 SigLIP crop 안정화를 추가했다.
- **상세:**
  - 4개 zone 경계를 격자 기준으로 재정의해 Z1/Z2는 각 9개 격자점, Z3/Z4는 각 12개 격자점을 포함하도록 하고 zone 간 중첩을 제거했다.
  - target selector, mission FSM, world model, live map 표시의 zone bounds/anchor 기본값을 동일하게 맞춰 웹 맵과 실제 선택 로직이 같은 구역을 보도록 했다.
  - Zone 1에서는 opening 이후 anchor 위치로 이동하지 않고, opening 후 정지한 위치에서 바로 목표를 선택하도록 FSM 흐름을 바꿨다.
  - LanePlanner에 `lane_simplify_enabled`를 추가해 직선 shortcut/string-pull을 끄면 A*가 만든 상하좌우 4-connected 레인 waypoint를 그대로 따르도록 했다.
  - `direct_fallback_enabled`를 추가해 레인 경로 실패 시 목표 직행을 끄고 현재 위치 hold를 선택할 수 있게 했다.
  - 실험 baseline으로 front escape와 go-to-goal 반응형 회피를 끄고, 레인 차단/clearance 비용을 느슨하게 조정했다.
  - opening 시작 전 대기를 18초에서 3초로 줄이고, 개막 이동 후에는 경기 시작 후 18초가 될 때까지 정지한 뒤 mapping/SCAN을 시작하도록 `opening_release_at_sec`를 추가했다.
  - localizer에 IMU motion state 기반 translation gain 감쇠와 `/localization/imu_motion_state` 진단 토픽을 추가해 정지/회전/충격 상황에서 vision/wall 위치 보정 흔들림을 줄일 수 있게 했다.
  - SigLIP fruit prompt를 보강하고 crop을 흰 배경 정사각형으로 padding하는 옵션을 추가해 길쭉하거나 비스듬한 과일 사진 crop 분류 안정성을 높였다.
  - LanePlanner 테스트에 simplify 비활성 시 레인 waypoint가 상하좌우 인접 노드로 유지되는지 검증을 추가했다.
- **변경 파일:**
  - `ros2_ws/src/robot_bringup/config/motion_tuning.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/localizer_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/recognition_viz_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/siglip_gate_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/world_model_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/lane_planner.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/target_selector_node.py` / 수정
  - `ros2_ws/src/robot_planning/test/test_lane_planner.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** opening 18초 release 실주행 확인, 레인-only/direct fallback 비교 주행, IMU motion/SigLIP crop 안정화 영향 확인 TODO를 추가했다.
- **버전 근거:** zone 분할, LanePlanner 동작 모드, opening release gate, IMU motion smoothing, SigLIP crop 처리 등 경기 실행 흐름과 기능 파라미터가 함께 추가·변경되었으므로 `MINOR` 버전 증가로 `v1.7.0`으로 올렸다.

### `v1.6.0` — 2026-07-17 22:42 (KST) · 작업자: `@AI` · ID: `2026-07-17-02`
- **변경 요약:** 통합 wall segmentation 모델을 기준으로 벽-바닥 경계 추출/검증을 재구성하고, 실주행 중 위치보정·회피·진단 튜닝을 보강했다.
- **상세:**
  - `wall_seg_v1.pt` 단일 모델에서 `wall_surface`와 `wall_floor_boundary` class를 함께 읽도록 벽 localizer를 변경했다.
  - 두꺼운 wall surface mask에서 로봇 중심 방향의 floor-side edge를 여러 선분으로 추출하고, boundary mask와 겹치는 선분만 벽-바닥 경계로 쓰도록 gate를 추가했다.
  - boundary gate가 너무 빡세게 작동해 벽 후보가 줄어드는 문제를 완화하기 위해 overlap/confidence/dilation/sample 간격을 실주행 테스트 기준으로 넓혔다.
  - 주황 raw wall overlay가 `/localization/wall_map_transform`을 중복 적용해 과장되어 보이는 문제를 막고, wall localizer가 계산한 field-frame raw 관측을 그대로 표시하도록 시각화를 정리했다.
  - 정지/settle/ALIGN/APPROACH standoff 등 실제로 멈춘 상태에서만 fast wall correction이 적용되도록 localizer와 wall localizer의 fast correction 조건을 보강했다.
  - opening warmup을 늘리고, opening/settle 후 mapping gate와 wall correction 흐름을 실제 경기 초기 인식 안정화에 맞췄다.
  - APPROACH 큰 전방 장애물 상황에서 direct fallback 대신 lateral escape waypoint를 만들도록 FSM planning fallback을 보강했다.
  - 일반 전진용 최소 바퀴 출력과 일반 횡이동용 최소 바퀴 출력을 분리해, 횡이동 명령이 모터 삐소리만 내고 실제로 움직이지 않는 상황을 완화했다.
  - wall raw segment를 base_link 기준으로 기록·비교하는 독립/메인 파이프라인 진단 스크립트를 추가했다.
  - body 카메라 밝기를 초기 설정에 가깝게 되돌리기 위해 고정 exposure/gain lock을 제거했다.
- **변경 파일:**
  - `.gitignore` / 수정
  - `ros2_ws/src/robot_bringup/config/motion_tuning.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_bringup/launch/cameras.launch.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/base_controller_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/localizer_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/recognition_viz_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/wall_localizer_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `scripts/run_wall_boundary_base_debug.sh` / 신규
  - `scripts/wall_boundary_base_debug.py` / 신규
  - `scripts/wall_distance_pause_diagnose.py` / 신규
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 통합 wall 모델의 boundary gate 통과율과 raw 주황선/base_link 상대거리, 횡이동 최소 출력, front escape waypoint 동작을 실주행에서 계속 검증한다.
- **버전 근거:** 벽 segmentation 입력/추출 방식 변경, boundary 검증 gate 추가, 위치보정 조건 보강, 회피 waypoint 및 진단 도구 추가가 포함된 기능 단위 변경이므로 `MINOR` 버전 증가로 `v1.6.0`으로 올렸다.

### `v1.5.0` — 2026-07-17 06:55 (KST) · 작업자: `@AI` · ID: `2026-07-17-01`
- **변경 요약:** 경기 실주행 중 pose drift와 벽 raw 오차 문제를 격리하고, 엔코더 기반 motion constraint와 메인 경기 안정화 튜닝을 통합했다.
- **상세:**
  - localizer에 엔코더 실제 움직임 기반 motion mode 분류를 추가해, command만 나가고 로봇이 물리적으로 멈춘 상황에서 object-flow/visual 보정이 pose를 계속 미는 문제를 줄였다.
  - `/localization/motion_mode`, `/localization/is_stationary`, `/localization/wall_map_transform` 등 위치추정 진단 토픽을 유지해 정지/스톨/전진/횡이동 상태를 분리해서 볼 수 있게 했다.
  - wall field correction은 현재 raw 주황선과 base_link 상대거리가 틀어지는 현상이 확인되어, 새 wall 모델/라벨 검증 전까지 pose 적용 gain과 step을 0으로 두는 안전 모드로 전환했다.
  - wall segmentation은 계속 실행·표시하되 pose를 끌지 못하게 해, 경기 진행 중 벽 raw 오류가 파란 로봇 pose와 물체/슬롯 맵 전체를 밀어내는 현상을 차단했다.
  - opening 이동 후 mapping gate를 유지해, 개막 전진/횡이동/회전과 4초 settle 전에는 world_model 물체/slot birth가 바로 생기지 않도록 했다.
  - 초음파 기반 align/hard stop 의존을 제거하고, body camera/월드모델 기반 접근·정렬 흐름으로 정리했다.
  - APPROACH 중 `/base_command` 주도권을 `go_to_goal_node`로 정리하고, 회피 상황에서는 회전 대신 횡이동 중심으로 피하도록 `go_to_goal_node` 회피 로직과 튜닝을 조정했다.
  - Set1/Set2 ALIGN에서 body lost 시 backoff 대신 10도 내외 yaw scan으로 목표를 다시 찾도록 하고, slot retry/track 없는 slot 처리 기준을 보수적으로 다듬었다.
  - base controller에 일반/개막/ALIGN 구동별 wheel minimum과 boost 튜닝을 분리해, opening 속도와 일반 주행/회피/정렬 속도를 독립적으로 조정할 수 있게 했다.
  - SigLIP fruit prompt pooling, body 카메라 조명 비교 스크립트, camera CSI 캡처 튜닝을 추가해 과일 큐브 오인식과 body 화면 저조도 문제를 진단할 수 있게 했다.
  - 독립 arrival parking 검증용 `robot_parking_test` 패키지와 웹/테스트 스캐폴딩을 추가했다.
  - YAML 파싱, `git diff --check`, 관련 Python 모듈 compile/build 테스트로 주요 정적 검증을 수행했다.
- **변경 파일:**
  - `firmware/base_arm_combined/base_arm_combined.ino` / 수정
  - `ros2_ws/src/robot_bringup/config/motion_tuning.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_bringup/launch/cameras.launch.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/base_controller_node.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/go_to_goal_node.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/pick_sequencer_node.py` / 수정
  - `ros2_ws/src/robot_hardware/robot_hardware/nodes/camera_csi_node.py` / 수정
  - `ros2_ws/src/robot_hardware/robot_hardware/nodes/mcu_bridge_base_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/localizer_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/siglip_gate_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/wall_localizer_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/world_model_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `scripts/csi_capture.py` / 수정
  - `scripts/lighting_compare.py` / 신규
  - `ros2_ws/scripts/lighting_compare.py` / 신규
  - `ros2_ws/src/robot_parking_test/` / 신규
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** wall segmentation 라벨/상대거리 재검증, wall gain 0 상태 pose drift 확인, 엔코더 motion mode 실측, body 카메라 저조도 비교, parking 테스트 패키지 활용 여부 확인 TODO를 추가했다.
- **버전 근거:** 위치추정 motion constraint, 벽 보정 안전 모드, 회피/정렬/구동 튜닝, 조명 진단 스크립트, parking 테스트 패키지 등 메인 경기 안정화를 위한 기능 추가와 구조적 튜닝이 포함되어 `MINOR` 버전 증가로 `v1.5.0`으로 올렸다.

### `v1.4.0` — 2026-07-15 23:43 (KST) · 작업자: `@AI` · ID: `2026-07-15-02`
- **변경 요약:** Set2 과일 큐브를 FruitSlot/grid 기반으로 안정화하고, 회피·재접근·시각화·테스트 경로를 경기 실행 흐름에 통합했다.
- **상세:**
  - Set2 과일 큐브 후보를 short-lived track ID가 아니라 고정 grid slot으로 관리하는 `SlotInventory`를 추가했다.
  - `fruit_photo_cube` 근거가 cube/shape vote에 눌리지 않도록 sticky 판단을 보강하고, wide-only 1회 오인식은 슬롯으로 바로 고정되지 않도록 누적 기준을 적용했다.
  - 슬롯 좌표를 7x6 경기장 격자에 lock하고, 현재 zone 안의 슬롯만 생성/선택하도록 FSM과 target selector를 맞췄다.
  - zone 순서는 1→2→3→4로 유지하되 anchor 강제 이동/정지 안정화는 끄고, 현재 위치에서 zone-local 후보를 처리하도록 튜닝했다.
  - body lost 후 같은 슬롯으로 재접근하고, 재접근 시 heading을 더 정확히 맞추되 timeout 후 ALIGN으로 빠져나오도록 해 APPROACH 무한 대기를 줄였다.
  - LanePlanner가 실제로 사용한 장애물 목록을 `/planning/obstacles`로 발행하고 `go_to_goal_node`가 이를 우선 사용하게 해 planner와 local avoidance의 판단 기준을 통일했다.
  - 라이브 맵에 Set2 slot, 간단 라벨 모드, checkpoint route overlay를 추가하고, anchor/checkpoint waypoint 테스트 스크립트를 작성했다.
  - 광각 마스트 lift MOSFET 핀을 D13 기준으로 정리하고, MCU bridge에서 startup lift command를 보낼 수 있게 했다.
  - `py_compile`, YAML 파싱, `git diff --check`, 관련 ROS 패키지 빌드로 정적 검증했다.
- **변경 파일:**
  - `docs/arm_pick_place_handoff.md` / 수정
  - `firmware/base_arm_combined/base_arm_combined.ino` / 수정
  - `ros2_ws/src/robot_bringup/config/motion_tuning.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_bringup/launch/bringup.launch.py` / 수정
  - `ros2_ws/src/robot_bringup/launch/test_field.launch.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/go_to_goal_node.py` / 수정
  - `ros2_ws/src/robot_hardware/robot_hardware/nodes/mcu_bridge_base_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/recognition_viz_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/world_model_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/target_selector_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/object_slot_inventory.py` / 신규
  - `ros2_ws/src/robot_planning/test/test_object_slot_inventory.py` / 신규
  - `scripts/anchor_waypoint_test.py` / 신규
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** FruitSlot 생성 기준 튜닝, `/planning/obstacles` 기반 회피 검증, body-lost 재접근 timeout 검증, waypoint 테스트 route 확정 TODO를 추가했다.
- **버전 근거:** Set2 slot inventory, planner/local avoidance 공유 토픽, 재접근 복구 로직, waypoint 테스트/시각화가 새 기능으로 추가되었으므로 `MINOR` 버전 증가로 `v1.4.0`으로 올렸다.

### `v1.3.0` — 2026-07-15 11:07 (KST) · 작업자: `@AI` · ID: `2026-07-15-01`
- **변경 요약:** v1.0.0 경기 기준선 위에 `motion_tuning.yaml` override 파일을 복구하고, ALIGN 큰 오차 구간에서 더 길게 이동하는 adaptive step을 추가했다.
- **상세:**
  - `motion_tuning.yaml`을 ROS parameter override 형식으로 새로 만들어, `perception.yaml` 기본값 위에 경기 튜닝값만 덮어쓸 수 있게 했다.
  - 메인 `bringup.launch.py`와 `perception.launch.py`가 `perception.yaml` 뒤에 `motion_tuning.yaml`을 함께 로드하도록 연결했다.
  - 기본 override 경로는 source tree의 `ros2_ws/src/robot_bringup/config/motion_tuning.yaml`을 우선 사용하므로, 튜닝값 수정 후 저장하고 런치를 다시 시작하면 재빌드 없이 숫자 변경이 반영된다.
  - 정지 중 맵 흔들림을 줄이기 위해 `suppress_object_flow_when_stationary`를 튜닝 파일에서 직접 조절할 수 있게 했다.
  - ALIGN 단계에서 목표 물체와의 전후/좌우 오차가 `align_mid_error_m` 이상이면 긴 펄스(`align_step_fwd_mid_sec`, `align_step_strafe_mid_sec`)를 쓰고, 가까워지면 기존 짧은 펄스로 정밀 조정하도록 FSM을 확장했다.
  - 로봇 실행 없이 `py_compile`과 YAML 파싱으로 정적 검증했다.
- **변경 파일:**
  - `ros2_ws/src/robot_bringup/config/motion_tuning.yaml` / 신규
  - `ros2_ws/src/robot_bringup/launch/bringup.launch.py` / 수정
  - `ros2_ws/src/robot_bringup/launch/perception.launch.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** motion tuning override 실제 적용 확인 TODO와 ALIGN adaptive step 실주행 튜닝 TODO를 추가했다.
- **버전 근거:** 새 튜닝 override 파일과 런치 연결, ALIGN adaptive step이라는 런타임 기능 확장이 추가되었으므로 `MINOR` 버전 증가로 `v1.3.0`으로 올렸다.

### `v1.2.1` — 2026-07-14 23:00 (KST) · 작업자: `@AI` · ID: `2026-07-14-03`
- **변경 요약:** 맵/자기위치 추정 안정성 비교를 위해 tracked 코드와 설정을 `ee7dec0` v1.0.0 기준선으로 복구했다.
- **상세:**
  - 최근 Set2 통합, motion tuning 프리셋, lane planner 확장, pick sequencer 파라미터 확장 이후 실제 주행에서 map drift가 커졌다는 피드백에 따라 안정 기준선으로 되돌렸다.
  - 원격 최신 기록은 보존한 채, `PROJECT_LOG.md`를 제외한 tracked 코드/설정 파일을 `ee7dec0 Release match baseline v1.0.0` 내용과 맞췄다.
  - `motion_tuning.yaml`과 `scripts/apply_motion_tuning.py`는 v1.0.0 기준선에 없던 실험/동기화 파일이므로 이번 복구 커밋에서 제거했다.
  - 복구 전 작업 내용은 로컬 `stash@{0}`에 `backup before restore ee7dec0` 이름으로 백업했다.
- **변경 파일:**
  - `ros2_ws/src/robot_bringup/config/motion_tuning.yaml` / 삭제
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/pick_sequencer_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/lane_planner.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `scripts/apply_motion_tuning.py` / 삭제
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** v1.0.0 기준선 복구 상태에서 실제 경기장 map 안정성 재검증 TODO와 Set2/HSV/one-stroke 기능의 단계적 재도입 검토 TODO를 추가했다.
- **버전 근거:** 새 기능 추가가 아니라 최근 통합 변경을 안정 기준선으로 되돌리는 복구성 수정이므로 `PATCH` 버전 증가로 `v1.2.1`로 올렸다.

### `v1.2.0` — 2026-07-14 17:04 (KST) · 작업자: `@AI` · ID: `2026-07-14-02`
- **변경 요약:** 경기용 `motion_tuning.yaml` 프리셋과 적용된 ROS YAML 설정을 커밋 대상으로 정리했다.
- **상세:**
  - `motion_tuning.yaml`을 경기 주행/인식/정렬/팔/플래너 튜닝의 단일 프리셋 파일로 추가했다.
  - `perception.yaml`과 `test_field.yaml`을 프리셋 값에 맞춰 동기화해 Set1/Set2 라벨, grid prior, opening, approach, ALIGN, planner, arm, live view 설정을 반영했다.
  - 프리셋을 ROS parameter YAML에 반영하는 `scripts/apply_motion_tuning.py`를 추가했다.
  - YAML에서 쓰는 adaptive ALIGN, taxi-style planner 옵션, reach 중 gripper 선개방 파라미터가 런타임에서 선언/동작하도록 관련 노드를 함께 맞췄다.
- **변경 파일:**
  - `ros2_ws/src/robot_bringup/config/motion_tuning.yaml` / 신규
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `scripts/apply_motion_tuning.py` / 신규
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/lane_planner.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/pick_sequencer_node.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** `motion_tuning.yaml`과 적용 대상 YAML의 동기화 결과를 dry-run/실제 적용으로 비교하는 TODO를 추가했다.
- **버전 근거:** 신규 튜닝 프리셋 파일과 적용 스크립트가 추가되고, 설정에서 참조하는 새 런타임 파라미터 지원이 포함된 기능 확장이므로 `MINOR` 버전 증가로 `v1.2.0`으로 올렸다.

### `v1.1.0` — 2026-07-14 16:54 (KST) · 작업자: `@AI` · ID: `2026-07-14-01`
- **변경 요약:** 구역 미션 phase 전환을 전역 Set1→Set2 방식에서 zone-local Set1→Set2 방식으로 바꿨다.
- **상세:**
  - 기존 FSM은 전체 field에서 Set1 phase를 먼저 끝낸 뒤 Set2 phase로 다시 zone 1부터 순회했다.
  - 실제 대회 운용 의도에 맞춰 각 zone에서 Set1 목표를 확인하고, 같은 zone 안에서 Set2 fruit cube까지 확인한 뒤 다음 zone으로 넘어가도록 phase 전환을 조정했다.
  - `shape_target_total`이 0이면 처음부터 Set2 phase로 시작할 수 있게 해, 시작 모드가 반드시 phase 1일 필요 없도록 했다.
  - quota 충족으로 phase가 바뀌는 경우도 zone-local 규칙을 따르도록 정리했다.
  - 향후 Set1 목표와 fruit cube 후보를 한 후보 큐로 섞어 다음 zone 방향으로 훑는 one-stroke inspection route는 별도 TODO로 남겼다.
- **변경 파일:**
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** zone-local Set1/Set2 FSM 실주행 검증 TODO와 one-stroke inspection route 점수식/YAML 구현 TODO를 추가했다.
- **버전 근거:** 구역 미션의 phase 진행 방식이 실제 대회 전략에 맞게 새 동작으로 확장된 기능 변경이므로 `MINOR` 버전 증가로 `v1.1.0`으로 올렸다.

### `v1.0.0` — 2026-07-13 18:17 (KST) · 작업자: `@AI` · ID: `2026-07-13-01`
- **변경 요약:** 정십이면체 4개 실전 경기 흐름을 기준으로 인식/구역/주행/ALIGN 튜닝과 모델 교체 상태를 1.0.0 기준선으로 정리했다.
- **상세:**
  - Set1 목표를 정십이면체 4개로 맞추고, Set2 과일 목표는 비활성화한 실전 설정을 유지했다.
  - 중앙 십자가 경계에서 각 구역이 10cm씩 겹치도록 `zone_bounds_m`을 조정해 맵 오차로 경계 물체를 놓칠 가능성을 줄였다.
  - 경기 시작 동작을 전진 2초, 오른쪽 횡이동 1.5초, 45도 회전 흐름으로 정리하고, 회전/횡이동 시작 시 부스트 펄스가 적용되도록 base controller를 유지했다.
  - APPROACH/ALIGN 기준 거리, 집기 위치, 미세조정 전후/횡 이동값을 현장 테스트 기준으로 정리했다.
  - ALIGN 단계에서 wide/world에는 목표가 남아 있는데 body에서만 놓치는 경우 약 10cm 후진 후 1초 대기하고 재탐지하도록 fallback을 추가했다.
  - body 카메라 지면 보정 기준을 앞 5.5cm, 높이 15.5cm, ArUco 38mm 기준으로 맞추고 관련 PnP 스크립트 설명과 기본값을 동기화했다.
  - 현재 실전 사용 모델 파일 `models/cube.pt`, `models/wide.pt`를 최신 교체 상태로 반영했다.
  - `robot_planning`, `robot_bringup` 빌드를 실행해 `install` 쪽 실행 파일/설정이 `src` 기준과 일치함을 확인했다.
- **변경 파일:**
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/base_controller_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/world_model_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `scripts/body_ground_pnp.py` / 수정
  - `models/cube.pt` / 수정
  - `models/wide.pt` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 공동 작업 로그 커밋·push 항목을 완료 처리했다. 실제 경기장에서 wide CSI 카메라 간헐 연결 문제, 구역 전환 타이머, 정십이면체 픽업 성공률은 계속 확인한다.
- **버전 근거:** 실전 경기 기준의 인식 모델, 구역 미션, 주행/ALIGN 복구 동작, body 지면 보정, 실행 설정을 하나의 기준선으로 확정하라는 사용자 요청에 따라 `MAJOR` 버전인 `v1.0.0`으로 승격했다.

### `v0.16.0` — 2026-07-11 23:04 (KST) · 작업자: `@AI` · ID: `2026-07-11-04`
- **변경 요약:** 실전 경기 실행 흐름에 구역 미션, 격자/라이브맵 표시, wall v3, 팔·주행·ALIGN 튜닝을 통합 반영했다.
- **상세:**
  - 메인 경기 시작 흐름을 15초 대기 후 `1초 전진 → 시계방향 45도 회전 → 1초 대기 → 자율 미션` 순서로 맞췄다.
  - 경기장을 2x2 구역 4개로 나누고, 1→2→3→4 반시계 순서로 구역 중심에 도착한 뒤 목표가 6초 동안 없을 때만 다음 구역으로 넘어가도록 FSM을 조정했다.
  - 구역 상태를 `/planning/zone`으로 발행하고, 라이브 맵에 42개 격자점과 4개 구역 사각형을 함께 표시하도록 했다.
  - 격자 기반 물체 위치 soft prior는 런타임 옵션으로 유지하되 실전 기본값은 꺼둔 상태(`grid_prior_enabled: false`)로 두었다.
  - 벽 인식 모델 경로를 `wall_floor_boundary_seg_v3.pt`로 맞추고, wide 화면 가장자리 5% 영역에 걸친 노란 벽 선분은 벽 기반 보정에서 제외하도록 했다.
  - 로봇 주행/회전/ALIGN 파라미터를 현장 피드백에 맞춰 재조정하고, 물체 선택 거리를 늘리며 미세조정 전후/좌우 이동량과 브레이크 펄스를 튜닝했다.
  - 경기 중 정이십면체만 목표로 잡도록 target selector/FSM 설정을 조정하고, Set2 오렌지 큐브 목표는 비활성화했다.
  - 주행 중 팔은 보관함 쪽 stow 자세로 올리고 그리퍼는 닫힌 상태를 유지하며, 팔을 내리거나 집기 전 필요한 순간에만 열도록 그리퍼 각도와 시퀀스를 정리했다.
  - 종료지점 arrival 데이터 촬영/학습 도구와 Colab 노트북, 문서 파일을 현재 작업 상태에 맞춰 커밋 대상으로 정리했다.
- **변경 파일:**
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_bringup/launch/bringup.launch.py` / 수정
  - `ros2_ws/src/robot_bringup/launch/test_field.launch.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/base_controller_node.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/go_to_goal_node.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/pick_sequencer_node.py` / 수정
  - `ros2_ws/src/robot_perception/package.xml` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/recognition_viz_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/wall_localizer_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/world_model_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/yolo_detector_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/target_selector_node.py` / 수정
  - `scripts/arm_pick2r.py` / 수정
  - `scripts/grid_prior_live_test.py` / 신규
  - `scripts/flag_capture_export.py` / 신규
  - `scripts/colab_train_body_flag_v7.ipynb` / 신규
  - `scripts/colab_train_wide_flag_v6.ipynb` / 신규
  - `docs/flag_capture_training.md` / 신규
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** body 카메라 각도 변경 후 재캘리브, 구역 중심 도착 후 6초 타이머 검증, 정이십면체 단독 픽업 검증 TODO를 추가했다.
- **버전 근거:** 실전 FSM 구역 미션, 라이브맵 구역/격자 표시, wall v3 반영, 주행/팔/ALIGN 동작 튜닝과 신규 테스트·학습 도구가 함께 추가된 기능 확장이므로 `MINOR` 버전 증가로 `v0.16.0`으로 올렸다.

### `v0.15.0` — 2026-07-11 18:36 (KST) · 작업자: `@AI` · ID: `2026-07-11-03`
- **변경 요약:** 월드모델에 7x6 격자 기반 물체 위치 soft prior를 추가하고, 정지 상태 라이브 테스트 경로를 만들었다.
- **상세:**
  - `world_model_node.py`에 6행 x 7열, 0.5m 간격의 격자 prior를 추가하고, 새 track 확정 시에만 nearest grid로 soft snap하도록 했다.
  - 기존 track 매칭은 항상 우선 처리하고, 이미 움직인 물체는 관측 위치를 유지하며 spawn grid에서 일정 거리 이상 벗어나면 `moved` 상태로 표시하도록 했다.
  - `grid_prior_enabled`, `grid_prior_debug`, snap 반경/alpha, moved threshold 등 파라미터를 추가하고, ROS parameter callback으로 실행 중 on/off 가능하게 했다.
  - `recognition_viz_node.py` 맵에 같은 격자 파라미터를 쓰는 작은 회색 점 42개를 표시해 실제 경기장 점과 인식 track 위치를 함께 볼 수 있게 했다.
  - `test_field.launch.py`에 `grid_prior_enabled`와 `params_file` launch 인자를 추가해 정지 테스트에서 source YAML과 격자 보정 옵션을 바로 주입할 수 있게 했다.
  - `scripts/grid_prior_live_test.py`를 추가해 `with_base:=false`로 로봇 구동 노드를 띄우지 않고, 격자 보정 perception stack과 브라우저용 live stream을 같이 실행하도록 했다.
  - `py_compile`, YAML 파싱, source launch 인자 확인, `git diff --check`로 정적 검증했다. 전체 ROS build는 기존 setuptools/colcon 설치 환경 문제로 별도 해결이 필요하다.
- **변경 파일:**
  - `ros2_ws/src/robot_perception/robot_perception/nodes/world_model_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/recognition_viz_node.py` / 수정
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_bringup/launch/test_field.launch.py` / 수정
  - `ros2_ws/src/robot_perception/package.xml` / 수정
  - `scripts/grid_prior_live_test.py` / 신규
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 격자 prior 구현과 정지 라이브 테스트 스크립트 작성을 완료 처리하고, 실제 경기장 기준 격자 원점/축 보정, 정지 테스트 비교, 실전 config 기본값 전환 판단 TODO를 추가했다.
- **버전 근거:** 월드모델 보정 알고리즘, 시각화 레이어, 테스트 실행 스크립트가 추가된 기능 확장이므로 `MINOR` 버전 증가로 `v0.15.0`으로 올렸다.

### `v0.14.0` — 2026-07-11 15:13 (KST) · 작업자: `@AI` · ID: `2026-07-11-02`
- **변경 요약:** 종료지점 클래스를 `arrival`로 확정하고, WASD 촬영 도구에 라이브 화면·10cm 이동·제자리 회전·6클래스 병합 학습 절차를 정리했다.
- **상세:**
  - 신규 종료지점/깃발 클래스명을 `taeguk_flag`에서 `arrival`로 변경하고, `flag_capture_export.py`, Colab 학습 노트북, 문서, perception 런타임의 클래스명을 맞췄다.
  - `world_model_node.py`에서 `arrival` 검출을 `set_type=3`으로 분류하고, Set1 shape vote 및 `fruit_photo_cube` 판단과 섞이지 않도록 했다.
  - `yolo_detector_node.py`에서 `arrival`이 `/classification/shape`로 재발행되지 않도록 제외했다.
  - `recognition_viz_node.py`의 training dump 클래스 목록과 시각화 색상에 `arrival`을 반영했다.
  - `flag_capture_export.py`에 MJPEG 라이브 화면 송출(`--mjpeg-port`, 기본 8090), 제자리 좌/우 회전 키(`j/l`), 10cm 이동 기본값(`speed=0.375`, `move_dur=0.29`, `strafe_dur=0.56`)을 추가했다.
  - `docs/flag_capture_training.md`에 라이브 화면 접속, `arrival` 라벨링, JSON→YOLO 변환, 기존 최신 학습셋(`dataset_body_cube_v6.zip`, `dataset_wide_v5.zip`)과 병합해 Colab용 zip을 만드는 흐름을 정리했다.
  - Colab 학습은 기존 모델 이어학습이 아니라 기존 누적 데이터와 `arrival` 데이터를 합친 6클래스 zip으로 `yolov8n.pt`에서 새로 학습하는 방향으로 정리했다.
- **변경 파일:**
  - `scripts/flag_capture_export.py` / 신규·수정
  - `scripts/colab_train_body_flag_v7.ipynb` / 신규·수정
  - `scripts/colab_train_wide_flag_v6.ipynb` / 신규·수정
  - `docs/flag_capture_training.md` / 신규·수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/yolo_detector_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/world_model_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/recognition_viz_node.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** `arrival` 클래스명 전환과 촬영 도구 라이브/제자리회전/10cm 이동 기본값 반영을 완료 처리하고, arrival JSON zip 변환, 기존 최종 학습셋과 병합, Colab 재학습 및 모델 교체 검증 TODO를 추가했다.
- **버전 근거:** 종료지점 클래스 계약을 `arrival`로 확정하고 촬영 도구 기능과 학습 데이터 병합 절차를 확장한 기능 추가이므로 `MINOR` 버전 증가로 `v0.14.0`으로 올렸다.

### `v0.13.0` — 2026-07-11 13:46 (KST) · 작업자: `@AI` · ID: `2026-07-11-01`
- **변경 요약:** 종료지점 태극기 깃발 학습을 위한 WASD 촬영·pre-label JSON export 도구와 6클래스 학습/런타임 연결을 추가했다.
- **상세:**
  - `scripts/flag_capture_export.py`를 새로 만들어 터미널 WASD 입력마다 0.5초 오픈루프 이동 후 body/wide 카메라 이미지를 저장하도록 했다.
  - 촬영 종료 시 기존 `models/cube.pt`, `models/wide.pt`로 각 카메라 이미지를 pre-label하고, X-AnyLabeling/LabelMe JSON과 이미지를 카메라별 폴더에 묶어 zip으로 내보내도록 했다.
  - 새 클래스명은 `taeguk_flag`로 고정하고, 기존 5클래스 뒤에 6번째 클래스로 추가하는 클래스 순서를 문서화했다.
  - `labelme_to_yolo.py`로 교정 JSON을 다시 YOLO 학습셋으로 변환한 뒤 Colab에서 학습할 수 있도록 body/wide 6클래스 학습 노트북을 추가했다.
  - `yolo_detector_node.py`에서 `taeguk_flag`가 Set1 shape classification으로 재발행되지 않도록 제외했다.
  - `world_model_node.py`에서 `taeguk_flag`를 `set_type=3` 도착점/깃발 표식으로 분류하고, fruit/shape identity vote와 섞이지 않도록 했다.
  - `recognition_viz_node.py`의 training dump 클래스 목록과 시각화 색상에 `taeguk_flag`를 추가했다.
- **변경 파일:**
  - `scripts/flag_capture_export.py` / 신규
  - `scripts/colab_train_body_flag_v7.ipynb` / 신규
  - `scripts/colab_train_wide_flag_v6.ipynb` / 신규
  - `docs/flag_capture_training.md` / 신규
  - `ros2_ws/src/robot_perception/robot_perception/nodes/yolo_detector_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/world_model_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/recognition_viz_node.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 깃발 데이터 수집/export 도구 작성 TODO를 완료 처리하고, 실제 촬영·라벨 교정, 6클래스 병합/재학습, 새 모델 배포 후 `set_type=3` 확인 TODO를 추가했다.
- **버전 근거:** 새 데이터 수집/export 스크립트, 6클래스 학습 노트북, 런타임 깃발 타입 처리가 추가된 기능 확장이므로 `MINOR` 버전 증가로 `v0.13.0`으로 올렸다.

### `v0.12.0` — 2026-07-10 22:32 (KST) · 작업자: `@AI` · ID: `2026-07-10-01`
- **변경 요약:** YOLO 벽 segmentation mask 기반 벽 보정/맵 생성 로직을 메인 경기 실행에 연결하고 저속 주행 설정으로 안정화했다.
- **상세:**
  - `wall_localizer_node.py`에서 기존 Canny/Hough/기하 게이트 중심 벽 후보 대신, 학습된 wall-floor-boundary segmentation mask의 connected component 중심과 주 방향(PCA/median line)을 최종 벽 관측으로 사용하도록 바꿨다.
  - L자형 mask가 대각선 한 줄로 이어지는 문제를 줄이기 위해 mask component 기반 선분 생성과 분할 로직을 정리하고, 보정용 field 선분(`/localization/wall_segments`), 원본 투영 선분(`/localization/wall_raw_segments`), 카메라 overlay용 mask 선분(`/localization/wall_mask_segments_image`)을 분리했다.
  - `localizer_node.py`에 wall anchor/field correction 입력과 `/localization/wall_map_transform` 발행을 추가해, 벽 보정이 들어갈 때 로봇 pose만 따로 움직이는 것이 아니라 전체 맵 좌표계 변환을 전달하도록 했다.
  - `world_model_node.py`와 `recognition_viz_node.py`가 `/localization/wall_map_transform`을 받아 로봇 pose, 물체 track/anchor/candidate, 경로, raw wall projection을 같은 강체 변환으로 이동하도록 연결했다.
  - `recognition_viz_node.py`에서 WIDE 카메라에는 mask 기반 노란 선분을, 맵에는 보정용 노란 선분과 raw 주황 선분을 표시해 실제 측정과 snapped wall의 차이를 확인할 수 있게 했다.
  - `perception.yaml`을 live mapping/test-field 설정과 동기화해 메인 `bringup.launch.py`에서도 wall segmentation v2, object-flow, object landmark, recognition visualization이 같은 방식으로 작동하도록 했다.
  - `perception.launch.py`에 `recognition_viz_node`를 추가하고, `bringup.launch.py`에는 IMU 노드를 추가해 메인 경기 실행에서도 현재 맵 생성 과정이 함께 뜨도록 했다.
  - `mission_fsm_node.py`의 opening 동작을 기존 전진+우측 횡이동에서 `1초 전진 → 45도 회전 → 1초 대기 → SCAN`으로 변경하고, 회전은 world heading 도달 기준과 timeout을 함께 쓰도록 했다.
  - 맵 생성 중 흔들림을 줄이기 위해 메인/테스트 config의 주행 속도와 회전 속도, base wheel floor/boost를 기존 대비 약 1/3 수준으로 낮췄다.
  - `py_compile`, YAML 파싱, 설치본 config 값 확인, `ros2 launch robot_bringup bringup.launch.py --show-args`로 launch 해석을 확인했다.
- **변경 파일:**
  - `ros2_ws/src/robot_perception/robot_perception/nodes/wall_localizer_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/localizer_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/world_model_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/recognition_viz_node.py` / 수정
  - `ros2_ws/src/robot_planning/robot_planning/nodes/mission_fsm_node.py` / 수정
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_bringup/launch/bringup.launch.py` / 수정
  - `ros2_ws/src/robot_bringup/launch/perception.launch.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** v2 벽 segmentation 학습/로봇 복사와 mask 중심선 튜닝 TODO를 완료 처리하고, 저속 메인 경기 실행 검증 및 스톨 시 최소 구동값 재상향 튜닝 TODO를 추가했다.
- **버전 근거:** 벽 mask 기반 위치보정 파이프라인을 재구성하고 메인 경기 launch/config/FSM 동작까지 연결한 기능 추가이므로 `MINOR` 버전 증가로 `v0.12.0`으로 올렸다.

### `v0.11.0` — 2026-07-09 22:40 (KST) · 작업자: `@AI` · ID: `2026-07-09-11`
- **변경 요약:** 벽 segmentation 기반 위치보정 디버깅과 엔코더 odom 검증 작업을 하루 마무리 기준으로 정리하고 GitHub 업로드 준비를 완료했다.
- **상세:**
  - LabelMe/X-AnyLabeling JSON과 이미지를 YOLO segmentation dataset으로 변환하는 스크립트와 Colab 학습 노트북을 준비하고, 사용자가 만든 `wall_floor_boundary_seg_v1.pt`를 로봇 로컬 `models/`에 복사해 테스트했다.
  - `wall_localizer_node.py`에 YOLO segmentation mask 입력을 연결하고, mask component 기반 후보 생성, edge fallback, 최종 field/image segment 발행을 추가했다.
  - `recognition_viz_node.py`에서 오른쪽 WIDE 카메라 화면에 segmentation mask와 최종 벽 선분을 표시해, mask는 잘 잡히지만 최종 선분이 mask와 떨어지는 현상을 직접 확인할 수 있게 했다.
  - 디버깅 결과를 반영해 최종 벽 선분 선택에서 mask 내부 후보를 더 중점적으로 쓰는 방향으로 수정했고, `test_field.yaml`은 현재 실험 기준으로 segmentation fallback을 잠시 끈 상태로 남겼다.
  - 오늘 수행한 엔코더 odom ROS 경로 검증, open-loop calibration, closed-loop 검증 스크립트와 로그를 함께 커밋 대상으로 정리했다.
  - 로컬 빌드 산출물(`build/`, `install/`, `log/`, package `build/egg-info`)과 개인 설정(`.claude/settings.local.json`), 학습 데이터/모델 바이너리는 커밋 대상에서 제외한다.
- **변경 파일:**
  - `ros2_ws/src/robot_perception/robot_perception/nodes/wall_localizer_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/recognition_viz_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/localizer_node.py` / 수정
  - `ros2_ws/src/robot_hardware/robot_hardware/nodes/mcu_bridge_base_node.py` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/launch/bringup.launch.py` / 수정
  - `docs/wall_segmentation_training.md` / 신규
  - `scripts/colab_train_wallseg_v1.ipynb` / 신규
  - `scripts/labelme_seg_to_yolo.py` / 신규
  - `scripts/live_field_view.py` / 수정
  - `scripts/drive_tests/encoder_distance_test.py` / 수정
  - `scripts/drive_tests/ros_open_loop_calib.py` / 신규
  - `scripts/drive_tests/encoder_control_validation.py` / 신규
  - `scripts/drive_tests/encoder_open_loop_calib_log.csv` / 신규
  - `scripts/drive_tests/ros_open_loop_calib_log.csv` / 신규
  - `scripts/drive_tests/encoder_control_validation_log.csv` / 신규
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** v1 segmentation 학습, 모델 로봇 복사, `wall_localizer_node.py` segmentation hook 연결을 완료 처리하고, 추가 촬영 61장 라벨링/v2 재학습 판단과 mask 기반 최종 선분 튜닝 TODO를 추가했다.
- **버전 근거:** segmentation 위치보정 연결, 시각화, 변환/학습 스크립트, 엔코더 검증 스크립트까지 하루 단위 기능 추가가 포함되어 `MINOR` 버전 증가로 `v0.11.0`으로 올렸다.

### `v0.10.0` — 2026-07-09 22:35 (KST) · 작업자: `@AI` · ID: `2026-07-09-10`
- **변경 요약:** 엔코더 odom을 실제 상위 주행 제어에 쓰기 위한 ROS/시리얼 검증 스크립트와 보정 절차를 추가했다.
- **상세:**
  - `encoder_distance_test.py`에 `open-loop-calib` 모드를 추가해, 엔코더 목표거리로 멈추는 방식이 아니라 정해진 시간 동안 MCU에 직접 `<BASE,...>`를 보내고 엔코더 적분거리와 실제 측정거리를 비교할 수 있게 했다.
  - `ros_open_loop_calib.py`를 추가해 실제 경기 주행 경로와 같은 `/base_command`→`base_controller_node`→`/base/wheel_speeds`→`mcu_bridge_base_node` 경로로 open-loop 테스트를 수행하고, 엔코더 odom과 실제 측정거리를 비교할 수 있게 했다.
  - `ros_open_loop_calib.py`는 방향/이동시간/속도를 실행 후 터미널에서 입력받도록 했고, 실제 측정값 입력 시 `test_field.yaml`의 `odom_scale`과 실행 중인 `/mcu_bridge_base_node` 파라미터를 갱신할 수 있게 했다.
  - ROS Humble 환경에서 `rclpy.parameter_client`가 없는 문제를 피하기 위해 `/mcu_bridge_base_node/set_parameters` 서비스를 직접 호출하는 방식으로 수정했다.
  - `mcu_bridge_base_node.py`에 `odom_scale`, `odom_wheel_scales`, 동적 파라미터 갱신 콜백을 추가해 엔코더 odom 스케일을 실행 중에도 바꿀 수 있게 했다.
  - `encoder_control_validation.py`를 추가해 상위 제어 적용 전 필요한 두 단계, 즉 같은 명령 반복성 테스트(`repeat-open-loop`)와 엔코더 목표거리 정지 테스트(`closed-loop-distance`)를 ROS 경로에서 수행할 수 있게 했다.
  - 실측 중 400ms/900ms 테스트의 실제거리 대비 엔코더거리 보정비가 크게 달라져, 짧은 시간/짧은 거리 데이터로 `odom_scale`을 바로 적용하면 위험하다는 판단을 남겼다.
- **변경 파일:**
  - `scripts/drive_tests/encoder_distance_test.py` / 수정
  - `scripts/drive_tests/ros_open_loop_calib.py` / 신규
  - `scripts/drive_tests/encoder_control_validation.py` / 신규
  - `scripts/drive_tests/encoder_open_loop_calib_log.csv` / 신규
  - `scripts/drive_tests/ros_open_loop_calib_log.csv` / 신규
  - `scripts/drive_tests/encoder_control_validation_log.csv` / 신규
  - `ros2_ws/src/robot_hardware/robot_hardware/nodes/mcu_bridge_base_node.py` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** ROS open-loop/상위 제어 검증 스크립트 작성 TODO를 완료 처리하고, 반복성 데이터 수집, `odom_scale` 확정, 엔코더 목표거리 정지 테스트, opening 이동 거리 기반 전환 검토 TODO를 추가했다.
- **버전 근거:** 새 검증 스크립트 2개와 엔코더 odom 동적 보정 기능을 추가해 상위 제어 적용 준비 범위가 넓어진 기능 추가이므로 `MINOR` 버전 증가로 `v0.10.0`으로 올렸다.

### `v0.9.0` — 2026-07-09 19:51 (KST) · 작업자: `@AI` · ID: `2026-07-09-09`
- **변경 요약:** FL 엔코더 불량 상태에서도 3륜 엔코더 odom 기반 위치 보정을 사용할 수 있게 했다.
- **상세:**
  - `mcu_bridge_base_node.py`에서 `<ODOM,...>`만 `/base/wheel_odom`으로 발행하도록 기본값을 바꾸고, `<HB,...>` 명령 echo는 `publish_heartbeat_as_odom` 옵션을 켤 때만 odom으로 쓰게 했다.
  - bridge와 localizer 양쪽에 작은 wheel odom deadband를 넣어 정지 중 잔진동/노이즈가 pose 적분으로 누적되는 것을 줄였다.
  - `localizer_node.py`에 `wheel_odom_enabled_wheels` 파라미터를 추가해 고장난 FL 엔코더를 제외하고 FR/RL/RR 3개 바퀴만으로 mecanum forward kinematics를 least-squares로 풀도록 했다.
  - `/base/wheel_speeds`와 encoder odom을 함께 보고 `/localization/is_stationary`를 발행하며, 정지 중에는 object-flow odom의 작은 흔들림을 pose에 반영하지 않도록 했다.
  - `test_field.yaml`과 `perception.yaml`에 현재 하드웨어 상태 기준으로 `wheel_odom_enabled_wheels: [false, true, true, true]`를 설정했다.
  - `bringup.launch.py`의 control/hardware 노드도 `perception.yaml`을 읽게 해 실전 bringup에서 bridge 파라미터가 적용되도록 했다.
- **변경 파일:**
  - `ros2_ws/src/robot_hardware/robot_hardware/nodes/mcu_bridge_base_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/localizer_node.py` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `ros2_ws/src/robot_bringup/launch/bringup.launch.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** `<HB>` 차단 확인, `/localization/is_stationary` 정지 판정 확인, FL 엔코더 복구 후 4륜 odom 재튜닝 TODO를 추가했다.
- **버전 근거:** 엔코더 odom 처리 정책, 고장 휠 제외 기구학, 정지 상태 발행/억제 기능을 추가한 위치추정 기능 개선이므로 `MINOR` 버전 증가로 `v0.9.0`으로 올렸다.

### `v0.8.0` — 2026-07-09 19:27 (KST) · 작업자: `@AI` · ID: `2026-07-09-08`
- **변경 요약:** 벽-바닥 경계 segmentation 라벨링/Colab 학습 준비 파일을 추가했다.
- **상세:**
  - 이전 대화 내용을 바탕으로 `wall_floor_boundary` 1-class segmentation 학습 흐름을 정리했다.
  - `data/wallseg_export`를 X-AnyLabeling 작업 폴더로 문서화하고 `classes.txt`, `labels/.gitkeep`, README를 추가했다.
  - Colab에서 zip 업로드, dataset root 자동 탐색, 라벨 누락/빈 라벨 검사, train/val 분할, `yolov8n-seg.pt` 학습, `wall_floor_boundary_seg_v1.pt` 다운로드까지 실행하는 노트북을 추가했다.
  - `docs/wall_segmentation_training.md`에 라벨링 규칙, zip 생성, Colab 실행, 로봇 복사, 이후 `wall_localizer_node.py` hook 방향을 정리했다.
  - 현재 `data/wallseg_export/images`에는 wide 이미지 184장이 있고, segmentation label `.txt`는 아직 0개임을 확인했다.
- **변경 파일:**
  - `docs/wall_segmentation_training.md` / 신규
  - `scripts/colab_train_wallseg_v1.ipynb` / 신규
  - `data/wallseg_export/classes.txt` / 신규
  - `data/wallseg_export/README.md` / 신규
  - `data/wallseg_export/labels/.gitkeep` / 신규
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** `wall_floor_boundary` polygon 라벨링, Colab segmentation 학습, 모델 복사, `wall_localizer_node.py` segmentation hook 구현 TODO를 추가했다.
- **버전 근거:** 새 학습 노트북과 문서/데이터셋 작업 폴더를 추가한 기능성 준비 작업이므로 `MINOR` 버전 증가로 `v0.8.0`으로 올렸다.

### `v0.7.19` — 2026-07-09 14:47 (KST) · 작업자: `@AI` · ID: `2026-07-09-07`
- **변경 요약:** 라이브 송출 기본 페이지를 canvas 렌더링 방식으로 바꿔 브라우저 탭 로딩중 표시를 줄였다.
- **상세:**
  - 기존 `/frame.png`를 `<img>`에서 0.2초마다 재요청하는 방식도 브라우저 탭 로딩 표시를 계속 유발할 수 있어, 기본 페이지를 `<canvas>` 기반으로 변경했다.
  - JavaScript가 `/frame.png`를 `fetch()`로 가져와 `createImageBitmap()` 후 canvas에 그리도록 했다. 페이지 문서는 한 번 로드되고, 이후 내부 bitmap만 갱신된다.
  - 8766 포트를 잡고 있던 이전 `python3` 송출 서버 프로세스(pid 16615)를 종료하고 새 서버를 재시작했다.
  - `/latest`, `/frame.png`, `/` 응답을 확인했고, 캐시를 피하기 위해 `http://127.0.0.1:8766/?v=canvas`를 열었다.
- **변경 파일:**
  - `scripts/live_field_view.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 화면 상단 latest age가 증가하면 메인 런/`recognition_viz_node`가 새 프레임을 쓰지 않는 상태로 판단한다.
- **버전 근거:** 라이브 뷰어 브라우저 표시 방식만 조정한 소규모 수정이므로 `PATCH` 버전 증가로 `v0.7.19`로 올렸다.

### `v0.7.18` — 2026-07-09 14:44 (KST) · 작업자: `@AI` · ID: `2026-07-09-06`
- **변경 요약:** 라이브 송출 기본 페이지를 MJPEG 직접 표시에서 PNG 프레임 주기 갱신 방식으로 바꿔 로딩중 고착을 줄였다.
- **상세:**
  - `http://127.0.0.1:8766/`가 계속 로딩중처럼 보이는 원인을 확인했다: 기존 서버가 8766 포트를 잡고 있었고, 기본 화면이 끝나지 않는 MJPEG 스트림을 직접 `<img>`로 열어 브라우저 로딩 표시가 지속될 수 있었다.
  - `scripts/live_field_view.py`에 `/frame.png` 엔드포인트를 추가해 최신 `live.png` 한 장을 일반 PNG 응답으로 제공하도록 했다.
  - 기본 HTML은 즉시 로드되고, JavaScript가 `/frame.png?t=...`를 0.2초마다 갱신하도록 변경했다. 기존 `/stream.mjpg` 엔드포인트는 보조 스트림으로 유지했다.
  - 이전 8766 점유 프로세스(`python3`, pid 12935)를 종료하고 새 서버를 띄운 뒤 `/latest`, `/frame.png`, `/` 응답을 확인했다.
- **변경 파일:**
  - `scripts/live_field_view.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 라이브 페이지가 안 보이면 먼저 `/latest`와 `/frame.png` 응답을 확인한다.
- **버전 근거:** 라이브 뷰어 표시/갱신 방식만 조정한 소규모 수정이므로 `PATCH` 버전 증가로 `v0.7.18`로 올렸다.

### `v0.7.17` — 2026-07-09 14:35 (KST) · 작업자: `@AI` · ID: `2026-07-09-05`
- **변경 요약:** 라이브 송출 화면을 별도 맵 탭 없이 전체 합성 화면 축소 표시 방식으로 되돌렸다.
- **상세:**
  - 사용자가 앞으로 직접 실행 대신 터미널 명령으로 안내해 달라고 요청했으므로, 이후 로봇/ROS/브라우저 실행은 명령만 제공하는 방식으로 전환한다.
  - `scripts/live_field_view.py`에서 `Map`/`Full` 링크와 `?view=...` 분기, 왼쪽 맵 crop 송출 로직을 제거했다.
  - 브라우저 화면은 `recognition_viz_node`가 생성한 원본 합성 이미지(왼쪽 맵 + 오른쪽 카메라 패널)를 그대로 송출하고, CSS `object-fit: contain`과 `max-width/max-height`로 창 안에 전체가 축소 표시되도록 했다.
  - 4x4 경기장 전체 표시는 `test_field.yaml`의 `field_extent_m: [-2.0, 2.0, -2.0, 2.0]`, `auto_extent: false` 설정을 그대로 사용한다.
- **변경 파일:**
  - `scripts/live_field_view.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 라이브 확인 시에는 먼저 송출 서버와 메인 런을 사용자가 직접 터미널에서 실행한다.
- **버전 근거:** 라이브 뷰어 표시 방식만 조정한 소규모 수정이므로 `PATCH` 버전 증가로 `v0.7.17`로 올렸다.

### `v0.7.16` — 2026-07-09 14:28 (KST) · 작업자: `@AI` · ID: `2026-07-09-04`
- **변경 요약:** 브라우저 라이브 송출 서버를 재기동하고 기본 화면을 맵 전체 보기로 변경했다.
- **상세:**
  - 라이브 화면이 보이지 않는 원인을 확인했다: `scripts/live_field_view.py` 서버 프로세스가 떠 있지 않았고, 최신 `live.png`도 `run_20260709_142340/20260709_142345/live.png`에서 갱신이 멈춘 상태였다.
  - 외부 라이브 뷰어의 기본 `/` 화면을 합성 패널 전체가 아니라 왼쪽 정사각형 맵 영역만 crop해 크게 송출하도록 변경했다.
  - 원본 합성 화면(맵 + 카메라 패널)은 `http://127.0.0.1:8766/?view=full`에서 계속 볼 수 있게 유지했다.
  - `python3 -m py_compile scripts/live_field_view.py`로 문법을 확인하고, `python3 scripts/live_field_view.py --port 8766 --fps 5`로 서버를 재기동한 뒤 `http://127.0.0.1:8766/` 브라우저를 열었다.
- **변경 파일:**
  - `scripts/live_field_view.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 상단 latest age가 계속 증가하면 송출 서버 문제가 아니라 `recognition_viz_node` 또는 메인 런이 새 `live.png`를 쓰지 않는 문제로 본다.
- **버전 근거:** 라이브 뷰어 표시 방식과 실행 상태 점검을 보완한 소규모 수정이므로 `PATCH` 버전 증가로 `v0.7.16`으로 올렸다.

### `v0.7.15` — 2026-07-09 14:20 (KST) · 작업자: `@AI` · ID: `2026-07-09-03`
- **변경 요약:** 메인 경기 런치 `bringup.launch.py`를 실행해 현재 실전 기동 불가 원인을 확인했다.
- **상세:**
  - `source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash && ros2 launch robot_bringup bringup.launch.py`로 메인 경기 런치를 실행했다.
  - 카메라, TF, localizer, world_model, mission_fsm, base controller, pick sequencer, MCU bridge 등 다수 노드는 기동됐다.
  - `arm_controller_node`는 `robot_control.arm_ik`에 없는 `WRIST_ROLL_HOME`를 참조하다 `AttributeError`로 즉시 종료됐다.
  - `mcu_bridge_base_node`는 `/dev/ttyUSB0`, `mcu_bridge_arm_node`는 `/dev/ttyUSB1` 포트를 열지 못해 계속 재시도했다.
  - 두 CSI 카메라는 `nvargus-daemon`/EGL/NvRm 접근 제한으로 프레임을 받지 못했고, `siglip_gate_node`는 네트워크 제한 때문에 Hugging Face 모델 파일 HEAD 요청을 재시도했다.
  - 따라서 메인 경기 런치는 “프로세스 다수 기동” 수준까지는 확인됐지만, 실제 경기 수행 가능한 정상 런으로 보기는 어렵다.
- **변경 파일:**
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** `arm_controller_node`의 `WRIST_ROLL_HOME` 참조 누락을 고치는 TODO를 추가했다. 기존 포트/udev TODO는 유지했다.
- **버전 근거:** 코드 변경 없이 메인 경기 런 실행 결과와 주요 장애 원인을 문서화한 작업 기록 보완이므로 `PATCH` 버전 증가로 `v0.7.15`로 올렸다.

### `v0.7.14` — 2026-07-09 14:17 (KST) · 작업자: `@AI` · ID: `2026-07-09-02`
- **변경 요약:** 포트 미인식 원인을 CH340 인식 후 `/dev/ttyUSB0` 미생성 상태로 진단했다.
- **상세:**
  - 권한 확장 `lsusb` 결과에서 `1a86:7523 QinHeng Electronics CH340 serial converter`가 보여 USB 장치 자체는 연결된 것을 확인했다.
  - `lsmod`에서 `ch341`, `usbserial` 모듈이 로드되어 있고, `/sys/class/tty/ttyUSB0`와 major/minor `188:0`도 존재해 커널 내부 TTY 등록까지는 완료된 상태를 확인했다.
  - 반면 `/dev/ttyUSB0`는 존재하지 않아, 이번 실패는 그리퍼 코드나 시리얼 권한 문제가 아니라 사용자 공간 장치 노드가 생성되지 않은 문제로 판단했다.
  - 현재 세션에서 `dmesg`는 계속 권한 제한으로 읽지 못해, 최종 원인이 `udev`, `devtmpfs`, 컨테이너/네임스페이스 노출 문제 중 무엇인지는 추가 점검이 필요하다.
- **변경 파일:**
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** CH340 장치에 대해 `udev`/`devtmpfs` 상태를 점검해 `/dev/ttyUSB0` 노드가 생성되지 않는 원인을 해결하는 TODO를 추가했다.
- **버전 근거:** 코드 변경 없이 포트 미인식 원인 분석과 후속 점검 항목을 문서화한 작업 기록 보완이므로 `PATCH` 버전 증가로 `v0.7.14`로 올렸다.

### `v0.7.13` — 2026-07-09 14:13 (KST) · 작업자: `@AI` · ID: `2026-07-09-01`
- **변경 요약:** 2R 팔 집기-보관 한 사이클 실행을 시도했으나 시리얼 포트가 없어 중단됐다.
- **상세:**
  - `scripts/arm_pick2r.py`의 현재 포즈 상수를 확인했다: `INIT [110,10,100]`, `PICK [25,150]`, `GRIP_OPEN=95`, `GRIP_CLOSED=40`, `PLACE [110,30]`.
  - `/dev/ttyUSB0`, `/dev/ttyACM0` 존재 여부와 `/dev/ttyUSB*`, `/dev/ttyACM*`, `/dev/ttyAMA*`, `/dev/ttyS*` 후보 포트를 확인했으나 사용 가능한 포트가 없었다.
  - `python3 scripts/arm_pick2r.py --port /dev/ttyUSB0 --loop 1 --move-delay 1.2 --grip-delay 0.8 --cycle-pause 0.5`를 실행했지만, `/dev/ttyUSB0`을 열 수 없어 실제 서보 명령은 전송되지 않았다.
  - 따라서 이번에는 그리퍼 각도 또는 보관함 투입 동작을 실제 하드웨어로 확인하지 못했다.
- **변경 파일:**
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 팔 제어 보드 연결 후 `/dev/ttyUSB*` 또는 `/dev/ttyACM*` 포트 인식 확인하고 집기-보관 1회 재실행 항목을 추가했다.
- **버전 근거:** 코드 변경 없이 하드웨어 실행 시도 결과와 후속 확인 항목을 문서화한 작업 기록 보완이므로 `PATCH` 버전 증가로 `v0.7.13`으로 올렸다.

### `v0.7.12` — 2026-07-08 22:43 (KST) · 작업자: `@AI` · ID: `2026-07-08-13`
- **변경 요약:** 오늘 작업된 경기장 인식·위치보정·주행·팔 동작 변경분을 git push 준비 상태로 정리했다.
- **상세:**
  - `new` 브랜치의 변경 상태를 확인하고, 로컬 빌드 산출물(`build/`, `install/`, `log/`)과 개인 설정(`.claude/settings.local.json`)은 커밋 대상에서 제외하기로 정리했다.
  - 커밋 대상은 ROS 소스/launch/config, 펌웨어, 주행 튜닝 스크립트와 로그, 라이브 뷰어, lane planner, wall localizer, IMU 노드, 학습 보조 노트북, `PROJECT_LOG.md`로 정했다.
  - 직전 하드웨어 확인 기준으로 팔 집기-보관 동작은 `open=95`, `closed=40`, `PLACE=[110,30]` 흐름을 유지한다.
- **변경 파일:**
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 이번 커밋 push 후 팀원들은 `new` 브랜치를 pull 받아 최신 경기장/팔/주행 통합 상태를 맞춘다.
- **버전 근거:** 코드 변경 없이 커밋/push 준비 상태와 제외 대상을 문서화한 작업 기록 보완이므로 `PATCH` 버전 증가로 `v0.7.12`로 올렸다.

### `v0.7.11` — 2026-07-08 22:41 (KST) · 작업자: `@AI` · ID: `2026-07-08-12`
- **변경 요약:** 2R 팔 단독 집기-보관 한 사이클을 기존 보정값으로 실행 확인했다.
- **상세:**
  - `python3 scripts/arm_pick2r.py --port /dev/ttyUSB0 --loop 1 --move-delay 1.2 --grip-delay 0.8 --cycle-pause 0.5`를 실행했다.
  - 동작 흐름은 `INIT [110,10,100] → PICK [25,150,95] → GRASP [25,150,40] → PLACE [110,30,40] → RELEASE [110,30,95]`이다.
  - 스크립트 출력의 놓기 안내 문구에는 `손목10`이라고 표시되지만, 실제 코드 상수 `PLACE`는 `손목30`으로 유지되어 보관 위치 동작은 `[110,30]`이다.
  - 코드 변경 없이 기존 보정값과 동작 시퀀스로 하드웨어 실행만 확인했다.
- **변경 파일:**
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 직접 스크립트의 출력 문구(`손목10`)는 혼동 방지를 위해 추후 `손목30`으로 정정한다.
- **버전 근거:** 코드 변경 없이 하드웨어 실행 확인 기록만 추가한 문서성 보완이므로 `PATCH` 버전 증가로 `v0.7.11`로 올렸다.

### `v0.7.10` — 2026-07-08 22:30 (KST) · 작업자: `@AI` · ID: `2026-07-08-11`
- **변경 요약:** 메인 경기 팔 시퀀스에 단계별 램프 발행을 추가하고 집기-놓기 이동 시간을 늘려 그리퍼 내림 속도를 완화했다.
- **상세:**
  - ROS `pick_sequencer_node`가 각 단계의 최종 각도만 반복 발행하던 방식을 바꿔, 현재 각도에서 목표 각도까지 중간 각도를 계속 발행하도록 했다.
  - 메인 집기 시퀀스는 기존 합의대로 `REACH(open) → GRASP(close 40) → LIFT(close 유지) → TO_PLACE(close 유지) → PLACE(open 95)`를 유지했다.
  - `test_field.yaml`의 `pick_sequencer_node` 설정에 `move_sec=4.0`, `grasp_sec=1.0`을 명시해 `test_field.launch.py` 실행 시 팔 내림/이동/그리퍼 단계가 더 천천히 보이도록 했다.
  - 집기 전체 시간이 약 `14s`로 늘어남에 따라 `mission_fsm_node.pick_duration_sec`를 `15.0`으로 늘렸다.
  - `python3 -m py_compile`로 문법을 확인했고, 임시 clean build-base(`/tmp/robot_pick_ramped_build`)로 `robot_control`, `robot_bringup`을 재빌드해 설치본 반영을 확인했다.
- **변경 파일:**
  - `ros2_ws/src/robot_control/robot_control/nodes/pick_sequencer_node.py` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 실제 경기장에서 `step REACH/GRASP/LIFT/TO_PLACE/PLACE` 로그와 함께 팔이 중간 각도로 천천히 내려가는지 확인한다.
- **버전 근거:** 메인 팔 제어 노드의 발행 방식과 실행 파라미터를 조정한 소규모 동작 수정이므로 `PATCH` 버전 증가로 `v0.7.10`으로 올렸다.

### `v0.7.9` — 2026-07-08 22:22 (KST) · 작업자: `@AI` · ID: `2026-07-08-10`
- **변경 요약:** 2R 팔 그리퍼 닫힘을 40도로 복구하고, 닫은 채 놓기 위치로 이동한 뒤 열도록 시퀀스를 조정했다.
- **상세:**
  - 그리퍼 닫힘값을 `30`에서 `40`으로 되돌렸다.
  - 직접 테스트용 `scripts/arm_pick2r.py`의 놓기 동작을 `PLACE [110,30]`까지 그리퍼 닫힘 상태로 이동한 뒤, 도착해서 `grip_open=95`로 열도록 변경했다.
  - ROS `pick_sequencer_node`의 True-trigger 시퀀스를 `REACH → GRASP → LIFT → TO_PLACE → PLACE`로 변경했다.
  - `test_field.yaml`의 `place_shoulder_wrist`를 `[110,30]`, `grip_closed`를 `40.0`으로 맞췄다.
  - `python3 -m py_compile`로 수정한 Python 파일 문법을 확인했고, 임시 clean build-base(`/tmp/robot_pick_place40_build`)로 `robot_control`, `robot_bringup`을 재빌드해 설치본을 갱신했다.
- **변경 파일:**
  - `scripts/arm_pick2r.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/pick_sequencer_node.py` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 실제 물체로 `GRASP 40 → LIFT 닫힘 유지 → TO_PLACE [110,30] 닫힘 유지 → PLACE open 95` 흐름을 확인한다.
- **버전 근거:** 팔 동작 시퀀스와 그리퍼 닫힘 보정값을 조정한 소규모 동작 수정이므로 `PATCH` 버전 증가로 `v0.7.9`로 올렸다.

### `v0.7.8` — 2026-07-08 22:18 (KST) · 작업자: `@AI` · ID: `2026-07-08-09`
- **변경 요약:** 2R 팔 그리퍼 닫힘 각도를 40도에서 30도로 낮춰 더 강하게 잡도록 조정했다.
- **상세:**
  - 직접 테스트용 `scripts/arm_pick2r.py`의 `GRIP_CLOSED`를 `30`으로 낮췄다.
  - ROS `pick_sequencer_node.py` 기본 `grip_closed`와 `test_field.yaml` launch override도 `30.0`으로 맞춰, 직접 테스트와 메인 경기 동작이 같은 그리퍼 닫힘값을 쓰도록 했다.
  - `python3 -m py_compile`로 수정한 Python 파일 문법을 확인했다.
  - 기존 `ros2_ws/build` 빌드는 setuptools `--uninstall` 옵션 오류가 있어 실패했으나, 임시 clean build-base(`/tmp/robot_gripper30_build`)로 `robot_control`, `robot_bringup`을 재빌드해 `ros2_ws/install` 설치본을 갱신했다.
- **변경 파일:**
  - `scripts/arm_pick2r.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/pick_sequencer_node.py` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 실제 물체로 `grip_closed=30` 집기-들기-놓기 1회 테스트 후, 물체가 찌그러지거나 서보 떨림이 심하면 `35` 근처로 되돌려 비교한다.
- **버전 근거:** 그리퍼 닫힘 보정값만 낮춘 하드웨어 튜닝성 소규모 수정이므로 `PATCH` 버전 증가로 `v0.7.8`로 올렸다.

### `v0.7.7` — 2026-07-08 20:13 (KST) · 작업자: `@AI` · ID: `2026-07-08-08`
- **변경 요약:** 브라우저 라이브 스트림 서버가 NUL-prefixed 요청과 HEAD 요청을 안정적으로 처리하도록 보완했다.
- **상세:**
  - 브라우저 라이브 송출 페이지에서 `Unsupported method ('\\x00...GET')` 501 에러가 발생해, HTTP 요청줄 앞에 붙은 NUL 바이트를 제거한 뒤 파싱하도록 `parse_request()`를 보완했다.
  - 브라우저/프록시가 보낼 수 있는 `HEAD` 요청에 200 응답을 주도록 처리했다.
  - 스트림 서버를 `HTTP/1.0` 응답으로 고정해 keep-alive 재사용으로 인한 요청 섞임 가능성을 줄였다.
  - `python3 -m py_compile scripts/live_field_view.py`로 문법 검사를 통과했고, `http://127.0.0.1:8766/?v=2`로 새 브라우저 창을 열었다.
- **변경 파일:**
  - `scripts/live_field_view.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 라이브 페이지가 멈추면 먼저 상단 latest age가 증가하는지 확인하고, 증가하면 로봇/인식 런타임의 `recognition_viz_node` 출력 갱신 여부를 점검한다.
- **버전 근거:** 브라우저 라이브 확인용 보조 서버의 오류 처리 보완이므로 `PATCH` 버전 증가로 `v0.7.7`로 올렸다.

### `v0.7.6` — 2026-07-08 20:05 (KST) · 작업자: `@AI` · ID: `2026-07-08-07`
- **변경 요약:** 최신 경기장 `live.png`를 자동 추적해 브라우저 MJPEG 스트림으로 보여주는 라이브 뷰어 스크립트를 추가했다.
- **상세:**
  - 기존 `live_view.html`은 특정 run 경로에 고정되어 새 경기장 run이 시작되면 예전 이미지만 보는 문제가 있었다.
  - `scripts/live_field_view.py`를 추가해 `data/mock_field_test/run_*/*/live.png` 중 가장 최근 파일을 매 프레임 자동으로 찾아 MJPEG 스트림(`/stream.mjpg`)으로 제공하게 했다.
  - 브라우저에서는 `http://127.0.0.1:8766/`만 열면 최신 run을 따라가며 지도/카메라 인식 패널을 라이브처럼 볼 수 있다.
  - `python3 -m py_compile scripts/live_field_view.py`로 문법 검사를 통과했다.
- **변경 파일:**
  - `scripts/live_field_view.py` / 신규
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 경기장 테스트 중 화면이 멈추면 페이지 상단의 최신 파일 age를 확인해 `recognition_viz_node`가 `live.png`를 계속 쓰는지 점검한다.
- **버전 근거:** 실험 확인용 보조 스트리밍 스크립트 추가로 기존 로봇 동작에는 영향이 없는 소규모 기능 보완이므로 `PATCH` 버전 증가로 `v0.7.6`로 올렸다.

### `v0.7.5` — 2026-07-08 20:00 (KST) · 작업자: `@AI` · ID: `2026-07-08-06`
- **변경 요약:** 경기장 인식/매핑 `live.png`를 브라우저에서 자동 갱신해 보는 라이브 뷰어 HTML을 추가했다.
- **상세:**
  - 단순 `live.png` 직접 열기는 정지 이미지처럼 보여 혼동되므로, 0.5초마다 cache-busting 쿼리로 이미지를 다시 불러오는 `live_view.html`을 추가했다.
  - 현재 최신 경기장 run(`run_20260708_195629/20260708_195634/live.png`)을 대상으로 지도와 카메라 인식 패널이 브라우저에서 계속 갱신되도록 했다.
  - 로컬 HTTP 서버(`python3 -m http.server 8765 --directory data/mock_field_test`)에서 `http://127.0.0.1:8765/live_view.html`로 열어 확인하는 흐름을 정리했다.
- **변경 파일:**
  - `data/mock_field_test/live_view.html` / 신규
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 새 경기장 run을 시작하면 `live_view.html`의 `imagePath`를 최신 run 경로로 갱신하거나, 추후 최신 run 자동 탐색 서버로 개선한다.
- **버전 근거:** 실험 확인용 보조 HTML 파일 추가로 기존 로봇 동작에는 영향이 없는 소규모 기능 보완이므로 `PATCH` 버전 증가로 `v0.7.5`로 올렸다.

### `v0.7.4` — 2026-07-08 18:30 (KST) · 작업자: `@AI` · ID: `2026-07-08-05`
- **변경 요약:** 2R 팔 집기 후 별도 놓기 위치/복귀 없이 올린 자세에서 바로 열도록 시퀀스를 단축했다.
- **상세:**
  - ROS `pick_sequencer_node`의 True-trigger 집기 시퀀스에서 `TO_PLACE [110,30,40]`와 마지막 `RETURN [110,10,100]` 단계를 제거했다.
  - 새 시퀀스는 `REACH [25,150,95] → GRASP [25,150,40] → LIFT [110,10,40] → PLACE [110,10,95]` 네 단계로 끝난다.
  - False-trigger release도 별도 place 이동 없이 INIT 어깨/손목 위치에서 그리퍼만 열도록 정리했다.
  - 직접 테스트용 `scripts/arm_pick2r.py`도 같은 흐름으로 맞춰, `release()`가 `[110,30]`으로 가지 않고 `[110,10]`에서 바로 열도록 했다.
  - `test_field.yaml`의 `place_shoulder_wrist`도 혼동 방지를 위해 `[110,10]`으로 갱신했다.
  - 임시 clean build-base(`/tmp/robot_pick_sequence_build`)로 `robot_control`, `robot_bringup`을 재빌드했고, 설치본에서 4단계 시퀀스와 startup 로그 `place=(110.0, 10.0)`을 확인했다.
- **변경 파일:**
  - `ros2_ws/src/robot_control/robot_control/nodes/pick_sequencer_node.py` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `scripts/arm_pick2r.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 실제 물체로 새 4단계 집기-놓기 흐름을 확인하고, 필요하면 LIFT 위치 또는 `grasp_sec`을 추가 조정한다.
- **버전 근거:** 기존 팔 동작 시퀀스에서 불필요한 단계 제거 및 흐름 단축을 한 소규모 동작 수정이므로 `PATCH` 버전 증가로 `v0.7.4`로 올렸다.

### `v0.7.3` — 2026-07-08 18:16 (KST) · 작업자: `@AI` · ID: `2026-07-08-04`
- **변경 요약:** 2R 팔 그리퍼 보정값만 open 95, close 40으로 갱신하고 설치본 반영을 확인했다.
- **상세:**
  - 사용자가 실측 확인한 그리퍼 값 `open=95`, `close=40`을 반영했다.
  - 어깨/손목 보정값은 유지했다: `init=[110,10,100]`, `pick=[25,150]`, `place=[110,30]`.
  - 직접 실행용 `scripts/arm_pick2r.py`의 상수와 안내 문구를 새 그리퍼 값으로 맞췄다.
  - ROS 기본값(`pick_sequencer_node.py`)과 `test_field.yaml` launch override의 `grip_open`, `grip_closed`를 동일하게 갱신했다.
  - 임시 clean build-base(`/tmp/robot_gripper_values_build`)로 `robot_control`, `robot_bringup`을 재빌드했고, 설치본과 `pick_sequencer_node` startup 로그에서 `grip(open=95.0/closed=40.0)`을 확인했다.
- **변경 파일:**
  - `scripts/arm_pick2r.py` / 수정
  - `ros2_ws/src/robot_control/robot_control/nodes/pick_sequencer_node.py` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 실제 물체로 집기-놓기 1회 테스트 후 필요하면 그리퍼 지연시간(`grasp_sec`, `grip-delay`)을 조정한다.
- **버전 근거:** 하드웨어 보정값 중 그리퍼 각도만 수정한 소규모 변경이므로 `PATCH` 버전 증가로 `v0.7.3`로 올렸다.

### `v0.7.2` — 2026-07-08 18:03 (KST) · 작업자: `@AI` · ID: `2026-07-08-03`
- **변경 요약:** 오늘 재보정된 2R 팔 집기·놓기 서보값을 test_field launch config와 직접 실행 스크립트에 명시 반영했다.
- **상세:**
  - 오늘 입력된 2R 팔 보정값을 재확인했다: `init=[110,10,100]`, `pick=[25,150]`, `place=[110,30]`, `grip_open=85`, `grip_closed=130`.
  - `test_field.yaml`의 `pick_sequencer_node` 파라미터에 `pick_shoulder_wrist`와 `place_shoulder_wrist`를 명시 추가해, `test_field.launch.py with_arm:=true` 실행 시 기본값 의존 없이 같은 값이 적용되도록 했다.
  - `scripts/arm_pick2r.py`의 상단 설명과 CLI 출력 문구에 남아 있던 이전 보정값(`88/10/140`, `10/170`, `150`, `80`)을 실제 코드 상수와 같은 최신값으로 정정했다.
  - 임시 clean build-base(`/tmp/robot_pick_values_build`)로 `robot_control`, `robot_bringup`을 재빌드했고, 설치본의 config와 `pick_sequencer_node` startup 로그에서 최신값 반영을 확인했다.
- **변경 파일:**
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `scripts/arm_pick2r.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 TODO는 유지했다. 실제 하드웨어에서 `/arm2r/target` echo와 서보 동작을 함께 보며 집기·놓기 반복 확인이 필요하다.
- **버전 근거:** 오늘 입력된 서보 보정값을 실행 config와 표시 문구에 반영한 소규모 보정이므로 `PATCH` 버전 증가로 `v0.7.2`로 올렸다.

### `v0.7.1` — 2026-07-08 03:32 (KST) · 작업자: `@AI` · ID: `2026-07-08-02`
- **변경 요약:** 벽/바닥 위치 보정에 adaptive Lab 색상 마스크와 robust residual trimming을 추가해 정확도를 높였다.
- **상세:**
  - 오늘 경기장 run 광각 프레임의 색 분포를 확인해, 고정 RGB 임계값 대신 프레임별 Lab median 기반 wood-tone 마스크를 생성하도록 했다.
  - HSV의 saturation/value 조건과 fisheye 중심 타원 ROI, 중앙 로봇 마스크를 함께 사용해 흰 물체, 사람, 로봇, 케이블 후보를 색 단계에서 줄였다.
  - 색상 마스크의 morphology gradient에서 Hough 선분을 추가로 추출해 벽/바닥 접선 후보를 보강했다.
  - 최종 벽 매칭에서 선분 midpoint뿐 아니라 양 끝점이 같은 경기장 벽(`x/y = ±2.0m`) 근처에 있는지 검사해 가짜 선분 통과를 줄였다.
  - 매칭된 residual은 MAD 기반 trimming 후 median을 다시 계산해 튄 선분 하나가 위치 보정을 흔들지 않도록 했다.
  - 오늘 run 샘플 기준 color mask 비율은 약 0.55~0.66, color line 후보는 약 140~199개로 확인했고, 실제 pose/호모그래피 조건에서도 벽 매치가 생성되는 것을 확인했다.
- **변경 파일:**
  - `ros2_ws/src/robot_perception/robot_perception/nodes/wall_localizer_node.py` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 기존 `wall_localizer_node` 실주행 로그 확인 및 파라미터 최종 튜닝 TODO를 유지했다.
- **버전 근거:** 기존 벽/바닥 위치 보정 기능의 정확도 개선과 파라미터 보강이므로 `PATCH` 버전 증가로 `v0.7.1`로 올렸다.

### `v0.7.0` — 2026-07-08 03:28 (KST) · 작업자: `@AI` · ID: `2026-07-08-01`
- **변경 요약:** 오늘 경기장 run 광각 사진 기준으로 벽/바닥 접선 기반 절대 위치 보정 기능을 실행 연결하고 검출을 보강했다.
- **상세:**
  - `wall_localizer_node`를 ROS2 console entry에 등록해 실제 launch에서 실행 가능하게 했다.
  - `test_field.launch.py`에 `with_wall_localizer` launch argument를 추가하고 기본값을 `true`로 설정했다.
  - `perception.launch.py`에도 `wall_localizer_node`를 포함해 일반 perception stack에서도 벽 기반 위치 보정을 사용할 수 있게 했다.
  - 오늘 경기장 run(`data/mock_field_test/20260708_*`)의 wide 프레임을 확인해, 나무톤 벽/바닥처럼 색 차이가 약한 환경에 맞춰 CLAHE, 중앙 로봇 마스크, Hough, LSD 선분 후보 검출을 추가했다.
  - 검출된 이미지 선분은 기존 wide 지면 호모그래피로 base_link 지면 좌표에 투영한 뒤, field 좌표의 4m 경기장 벽(`x/y = ±2.0m`) 근처이고 축 정렬 조건을 만족할 때만 `/localization/landmark_correction`으로 위치/각도 보정을 발행한다.
  - localizer가 `/localization/landmark_correction`을 물체 랜드마크 전용으로만 소비하지 않고, 벽 기반 보정도 같은 절대 landmark correction으로 반영할 수 있게 조건을 정리했다.
  - `test_field.yaml`에는 오늘 run의 1280x960 광각 스트림 기준 파라미터를, `perception.yaml`에는 기존 1640x1232 광각 스트림 기준 파라미터를 각각 추가했다.
- **변경 파일:**
  - `ros2_ws/src/robot_perception/robot_perception/nodes/wall_localizer_node.py` / 수정
  - `ros2_ws/src/robot_perception/robot_perception/nodes/localizer_node.py` / 수정
  - `ros2_ws/src/robot_perception/setup.py` / 수정
  - `ros2_ws/src/robot_bringup/launch/test_field.launch.py` / 수정
  - `ros2_ws/src/robot_bringup/launch/perception.launch.py` / 수정
  - `ros2_ws/src/robot_bringup/config/test_field.yaml` / 수정
  - `ros2_ws/src/robot_bringup/config/perception.yaml` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 벽/바닥 접선 기반 위치 보정 노드 실행 연결 완료 체크, 실제 경기장 주행 중 wall localizer 보정 로그 확인 및 파라미터 최종 튜닝 TODO 추가.
- **버전 근거:** 새 위치 보정 실행 경로와 launch/config 연결이 추가된 기능 확장이므로 `MINOR` 버전 증가로 `v0.7.0`으로 올렸다.

### `v0.6.0` — 2026-07-07 15:59 (KST) · 작업자: `@AI` · ID: `2026-07-07-04`
- **변경 요약:** 전진/후진/횡이동/회전 통합 튜닝 스크립트를 만들고 이전 실험용 주행 스크립트를 휴지통 폴더로 정리했다.
- **상세:**
  - `scripts/drive_tests/motion_tune.py`를 추가해 `forward`, `backward`, `right`, `left`, `cw`, `ccw`를 한 스크립트에서 튜닝할 수 있게 했다.
  - 모든 동작은 시작 boost, 고정 시간 연속 구동, 선택적 reverse brake, 실측값 입력, 다음 duration 추천 흐름을 사용한다.
  - 각 trial마다 `scales FL FR RL RR`를 입력해 동작별·모터별 세기 보정이 가능하도록 했다.
  - 전진/후진은 yaw 입력값을 바탕으로 보수적인 좌우 모터 scale 조정 추천을 출력하도록 했다.
  - 현재까지의 실측값을 바탕으로 1차 seed를 반영했다: 전진 100cm 약 `1222ms`, 후진 100cm 약 `1313ms`, 오른쪽 횡이동 100cm 약 `3041ms`, 왼쪽 횡이동 100cm 약 `3410ms`, 시계 회전 `45도=100ms`, `90도=230ms`, `180도=665ms`.
  - 이전 개별/실험용 주행 튜닝 스크립트는 삭제하지 않고 `scripts/drive_tests/trash/2026-07-07_old_drive_methods/`로 이동했다.
  - 최종 주행용 함수/모듈은 아직 만들지 않고, 최종 튜닝값 확정 후 진행하기로 했다.
- **변경 파일:**
  - `scripts/drive_tests/motion_tune.py` / 신규
  - `scripts/drive_tests/trash/2026-07-07_old_drive_methods/lateral_distance_tune.py` / 이동
  - `scripts/drive_tests/trash/2026-07-07_old_drive_methods/linear_distance_tune.py` / 이동
  - `scripts/drive_tests/trash/2026-07-07_old_drive_methods/rotation_tune.py` / 이동
  - `scripts/drive_tests/trash/2026-07-07_old_drive_methods/pulse_lateral_tune.py` / 이동
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 횡이동 최소 구동 속도와 드리프트 보정값 재측정 1차 완료 체크, 직진/회전 보정값 스크립트 정리 완료 체크, `motion_tune.py` 최종 튜닝값 확정과 최종 주행용 함수/모듈 작성 TODO 추가.
- **버전 근거:** 여러 이동 방향을 한 곳에서 다루는 새 통합 튜닝 스크립트가 추가되고 기존 실험 스크립트 구조가 정리된 기능 확장이므로 `MINOR` 버전 증가로 `v0.6.0`으로 올렸다.

### `v0.5.0` — 2026-07-07 13:41 (KST) · 작업자: `@AI` · ID: `2026-07-07-03`
- **변경 요약:** 전방 45도 대각선 이동 단독 테스트 방향을 추가했다.
- **상세:**
  - `scripts/drive_tests/encoder_distance_test.py`의 `--direction` 선택지에 `forward_right`, `forward_left`를 추가했다.
  - `forward_right`는 `[+speed, 0, 0, +speed]`, `forward_left`는 `[0, +speed, +speed, 0]` 휠 명령을 사용한다.
  - 대각선 이동처럼 0속도 바퀴가 있는 방향에서 거리 적산이 0속도 바퀴에 끌려가지 않도록, ODOM 샘플 계산 시 부호값이 0인 바퀴는 제외하도록 했다.
  - 기존 `single`, `lateral-kick`, `forward-then-lateral` 모드는 유지했다.
- **변경 파일:**
  - `scripts/drive_tests/encoder_distance_test.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** `forward_right`/`forward_left` 단독 대각선 이동의 실제 각도, 거리, 회전 틀어짐 기록 항목을 추가했다.
- **버전 근거:** 테스트 스크립트에 새 이동 방향이 추가된 기능 확장이므로 `MINOR` 버전 증가로 `v0.5.0`으로 올렸다.

### `v0.4.0` — 2026-07-07 13:38 (KST) · 작업자: `@AI` · ID: `2026-07-07-02`
- **변경 요약:** 거리 기반 전진 후 횡이동하는 `forward-then-lateral` 모드를 추가했다.
- **상세:**
  - `scripts/drive_tests/encoder_distance_test.py`에 `--mode forward-then-lateral` 옵션을 추가했다.
  - 새 모드는 먼저 `--pre-m` 거리만큼 `forward`를 엔코더 폐루프로 주행한 뒤, 정지 명령 없이 `--direction right|left` 횡이동으로 넘어간다.
  - 전진 구간 제한 시간은 `--pre-timeout`으로 조절하고, 횡이동 구간은 기존 `--target-m`, `--timeout` 값을 사용한다.
  - 전진 구간이 실패하거나 timeout 되면 정지 명령을 보내고 종료하도록 했다.
  - 전진 구간 후 횡이동 구간 timeout과 거리 적산을 새로 시작하도록 타이머와 진행값을 초기화했다.
- **변경 파일:**
  - `scripts/drive_tests/encoder_distance_test.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** `forward-then-lateral` 모드의 전진 10cm 후 오른쪽/왼쪽 10cm 횡이동 성공 여부와 최종 위치 오차 기록 항목을 추가했다.
- **버전 근거:** 기존 테스트 스크립트에 새 거리 기반 시퀀스 모드와 옵션이 추가된 기능 확장이므로 `MINOR` 버전 증가로 `v0.4.0`으로 올렸다.

### `v0.3.0` — 2026-07-07 13:29 (KST) · 작업자: `@AI` · ID: `2026-07-07-01`
- **변경 요약:** 횡이동 정지마찰 회피용 `lateral-kick` 테스트 모드를 추가했다.
- **상세:**
  - `scripts/drive_tests/encoder_distance_test.py`에 `--mode single|lateral-kick` 옵션을 추가했다.
  - 기본값 `single`은 기존 직진/후진/횡이동/회전 동작을 유지한다.
  - `lateral-kick` 모드는 `--direction right` 또는 `left`에서만 사용하며, 짧은 전방 대각선 구동 후 정지 명령 없이 기존 횡이동 폐루프 제어로 넘어간다.
  - 오른쪽 횡이동 킥은 `[+speed, 0, 0, +speed]`, 왼쪽 횡이동 킥은 `[0, +speed, +speed, 0]` 휠 명령을 사용한다.
  - `--kick-ms`, `--kick-speed-scale` 옵션으로 대각선 킥 시간과 속도 비율을 조절할 수 있게 했다.
- **변경 파일:**
  - `scripts/drive_tests/encoder_distance_test.py` / 수정
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** `lateral-kick` 모드의 오른쪽/왼쪽 횡이동 시작성, 실제 이동 거리, 드리프트 실측 기록 항목을 추가했다.
- **버전 근거:** 기존 테스트 스크립트에 새 실행 모드와 옵션이 추가된 기능 확장이므로 `MINOR` 버전 증가로 `v0.3.0`으로 올렸다.

### `v0.2.0` — 2026-07-06 19:38 (KST) · 작업자: `@AI` · ID: `2026-07-06-02`
- **변경 요약:** 엔코더 기반 베이스 주행 테스트 환경을 만들고 직진/회전 1차 보정값을 정리했다.
- **상세:**
  - `firmware/base_arm_combined/base_arm_combined.ino`에 엔코더 tick 기반 `<ODOM,fl,fr,rl,rr,t_ms>`, raw `<ENC,e1,e2,e3,e4,t_ms>`, `<ENCZERO,OK>` 응답을 추가했다.
  - QGPMaker V5.2 핀맵 기준으로 엔코더 매핑을 확인했고, 실측 결과에 맞춰 `WHEEL_ENCODER={0,3,1,2}`, `WHEEL_ENC_DIR={+1,+1,-1,+1}`로 정리했다.
  - `LIFT_PIN`이 Encoder1 B상과 충돌해 테스트 중에는 `-1`로 비활성화했다.
  - `scripts/drive_tests/encoder_distance_test.py`를 새로 만들어 ROS2 없이 serial만으로 직진/후진/횡이동/제자리 회전 테스트가 가능하도록 했다.
  - 같은 스크립트에 reverse brake pulse, trusted ODOM wheel 선택, 회전용 `cw/ccw`, `target-deg`, `k` 적산 옵션을 추가했다.
  - `scripts/drive_tests/README.md`를 추가해 펌웨어, serial 테스트, ROS2 재연결 전까지의 bring-up 흐름을 문서화했다.
  - 실측 보정 결과로 직진 60cm 기준값 `forward target-m 0.133`, `backward target-m 0.146`을 확보했다.
  - 실측 보정 결과로 제자리 90도 기준값 `cw target-deg 69.3`, `ccw target-deg 101.0`을 확보했다.
  - 60cm 정사각형 시계방향 둘레 테스트용 실행 순서를 정리했다. 각 변은 직진 60cm, 각 꼭짓점은 시계 90도 회전 기준으로 반복한다.
- **변경 파일:**
  - `firmware/base_arm_combined/base_arm_combined.ino` / 수정
  - `scripts/drive_tests/README.md` / 신규
  - `scripts/drive_tests/encoder_distance_test.py` / 신규
  - `PROJECT_LOG.md` / 수정
- **다음 할 일 반영:** 문서 로그 규칙 검증 완료 체크, 엔코더 ODOM 출력 추가 완료 체크, serial 주행 테스트 경로 구축 완료 체크, 직진 60cm 보정 완료 체크, 90도 회전 보정 완료 체크, 횡이동 드리프트 재측정/정사각형 오차 기록/프리셋 정리/IMU 결합 항목 추가.
- **버전 근거:** 펌웨어 기능 확장과 새 테스트 스크립트/문서 추가가 포함된 기능 단위 작업이므로 `MINOR` 버전 증가로 `v0.2.0`으로 올렸다.

### `v0.1.0` — 2026-07-06 (KST) · 작업자: `@AI` · ID: `2026-07-06-01`
- **변경 요약:** 공동 작업 기록 문서 최초 생성, 작업자 7명 등록.
- **상세:**
  - 버전 관리 규칙(1장), 현재 상태(2장), 작업자 명단(3장, 7명), 다음 할 일(4장), 변경 이력(5장), 엔트리 템플릿(6장) 구성.
  - "기록 삭제 금지 · 추가만 허용" 규칙 확정. 대상 브랜치는 `new`.
- **변경 파일:** `PROJECT_LOG.md` (신규)
- **다음 할 일 반영:** 문서 생성 `[x]` 완료 / 브랜치 커밋·pull, 역할 확정, 첫 작업 검증, 명명 규칙 합의 등록.
- **버전 근거:** 문서 최초 생성 → 초기 버전 `v0.1.0`.

<!-- ↑↑↑ 다음 작업부터는 위 안내 주석 바로 아래에 새 엔트리를 쌓아 올리세요. -->

---

## 6. 엔트리 작성 템플릿 (복사해서 5장 맨 위에 붙여넣기)

```markdown
### `v<버전>` — <YYYY-MM-DD HH:MM (KST)> · 작업자: `@<핸들>` · ID: `<YYYY-MM-DD-NN>`
- **변경 요약:** (한 줄로)
- **상세:**
  - (무엇을, 왜, 어떻게 바꿨는지)
- **변경 파일:** (파일명 / 신규·수정·삭제 표시)
- **다음 할 일 반영:** (완료 체크한 항목, 새로 추가한 할 일)
- **버전 근거:** (MAJOR/MINOR/PATCH 중 무엇을 왜 올렸는지)
```
