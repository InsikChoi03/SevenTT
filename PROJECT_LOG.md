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

- **현재 버전:** `v0.7.12`
- **최종 업데이트:** 2026-07-08 22:43 (KST)
- **최종 작업자:** `@AI`
- **한 줄 요약:** 오늘 작업된 경기장 인식·위치보정·주행·팔 동작 변경분을 git push 준비 상태로 정리했다.

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
- [ ] 이 문서를 `new` 브랜치 루트에 커밋·push 하고, 팀원 전원 pull 받기 — `@insik`
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
- [ ] `motion_tune.py`로 전진/후진/횡이동/회전 최종 튜닝값 확정 — `@insik`
- [ ] 최종 튜닝값 확정 후 실제 경기 주행용 함수/모듈 작성 — `@AI`
- [x] 오늘 경기장 run 광각 사진 기준 벽/바닥 접선 기반 위치 보정 노드 실행 연결 — `@AI` (2026-07-08 완료)
- [ ] 실제 경기장 주행 중 `wall_localizer_node` 로그의 `wall fix dx/dy/dth`를 확인하고 `wall_gate_m`, `axis_tol_deg`, `correction_conf` 최종 튜닝 — `@insik`

---

## 5. 변경 이력 (Changelog) — ⛔ 삭제 금지 / 최신 항목이 맨 위

<!-- 새 엔트리는 바로 이 줄 아래에 추가하세요. 기존 엔트리는 건드리지 마세요. -->

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
