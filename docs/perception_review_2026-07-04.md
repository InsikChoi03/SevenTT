# test_field 인식·융합 스택 심층 리뷰 (2026-07-04)

> ## ★ 최종 검증 완료 (멀티에이전트 워크플로우 완주, 52 에이전트)
> 6개 관점 리뷰 → 적대적 검증 → 아키텍처 3렌즈 → 심사(judge) 전부 완료.
> **결과: CONFIRMED 24 / PLAUSIBLE 1 / REFUTED 8.** REFUTED 8건은 대부분 이 세션의
> 수정(캡처스탬프·병진 dedup·approach 0.35)으로 반박됨 = 수정이 실제로 반영·작동함이 교차검증됨.
>
> **심사(judge) 최종 판정**: 듀얼카메라 융합 골격은 건전, 교체 불필요. 커밋 후 워킹트리에 대규모
> 패치가 이미 적용되어(내 인식-코드 수정 + 사용자의 mission_fsm CLASSIFY 공간게이트·ALIGN 비주얼서보)
> 리뷰가 지목한 '위생 3종'과 오픽 공간연관이 사실상 해소됨. **남은 진짜 결함 3가지**:
> 1. **중복/팬텀 트랙 dedupe 부재** (채점 최상위 리스크) → **이 세션에서 수정함** (world_model `_merge_duplicates`).
> 2. **classify 타임아웃 즉시·영구 blacklist** (재시도 없음 → 타겟 starvation) → **mission_fsm, 사용자 영역**.
> 3. **투표 무감쇠 + body 단일표 권위 + 영구 blacklist** (단일 오분류 복구경로 부재) → P8(일 단위).
>
> **judge 로드맵 대회前 컷라인**: P1 dedupe(완료) · P2 재시도카운터(fsm) · P3 **필드계측 검증**(주행 과주행/
> projected_dets 중복/conf 분포) · P4 배포모델(wide.pt vs wide_v4.pt)+임계셋 재보정 · P5 dry_pick 정합.
> 기각: 전면 EKF/GTSAM/BEV/YOLO단일화/Dempster-Shafer, 계측근거 없는 body_fov_far config 변경.
> **미검증 패치를 대회에 그대로 올리는 것이 최대 리스크 → 신규개발보다 필드 검증(P3)이 우선.**

멀티에이전트 리뷰(6개 관점 + 적대적 검증 + 아키텍처 심사 완주)
+ 세션 본체의 전 코드 직접 정독 교차검증. 검증 상태 표기: **[확인]** = 코드 라인 직접 확인,
**[기전확인]** = 코드 기전은 확인·크기는 실측 필요, **[실측필요]** = 런타임 측정 전엔 크기 미상.

## 총평

아키텍처 골격 자체는 이 제약(LiDAR 없음, **엔코더 없음**, Orin Nano 단일보드, 작은 필드, 준정적 물체)에
잘 맞는 합리적 설계다. 위치/신원 증거 분리, 소스별 투표 풀, 음의 증거, 앵커 고정+Umeyama 보정,
heading 게이팅, 두 캠의 base_link 미터 프레임 통일(정적 상태 cm급 일치)은 모두 제대로 짜여 있다.
좌표 규약(rot180 광선 반전, cam_yaw 90°, base→field 변환, 쿼터니언 yaw)도 코드-설정 간 일관적이다.

문제는 아키텍처가 아니라 **추정 위생(estimation hygiene)** 세 가지다:

1. **융합 회계 오류** — 병진이 휠 echo와 object-flow에서 이중적산 (회전만 dedup).
2. **시간 동기화 부재** — 캡처 시각이 체인에서 두 번 파괴되어, 회전 중 맵 스미어가
   랜드마크 보정으로 되먹임되는 폐루프.
3. **비가역 결정 누적** — fpc 원샷 Set2 전환, body 절대 우선, 영구 blacklist, 투표 무감쇠,
   selector 정확일치 필터가 겹쳐 "한 번의 오류에 복구 경로가 없는" 구조.

풀 SLAM(EKF/GTSAM) 전환은 이 문제들을 해결해 주지 않으며 대회 전 리스크만 키운다. **현 설계 유지 +
아래 로드맵의 표적 수정**이 정답이다.

## A. 검증된 핵심 결함

### A1. 융합 회계 (localizer)

- **[확인/critical] 병진 이중적산** — `localizer_node.py:234` 휠 x/y 무조건 적분 +
  `localizer_node.py:259-262` object-flow 병진도 적분(`object_flow_trans:true`).
  펌웨어 `<HB>`는 엔코더 없는 명령 echo(5Hz)이므로 주행 중 두 소스가 각각 ~0.2m/s → pose 최대 2배.
  deadband 8mm < 프레임당 24mm라 못 거른다. 회전의 `last_vo_time` 게이트에 대응하는 병진 게이트 부재.
  **수정: 병진 소유자 단일화** (즉효: `object_flow_trans:false`; 또는 flow fresh 시 휠 병진 억제).
- **[기전확인/critical] 회전 staleness 창 불일치** — flow dθ는 wide 프레임 간 '전체 구간' 델타인데
  휠 yaw는 0.25s, LK는 0.5s 만에 같은 구간 안에서 깨어나 이중계상(`localizer_node.py:238`).
  반대로 flow 기각 프레임에서 `world_model_node.py:959-960`이 `_prev_wide_base`를 무조건 전진시키고
  conf<0.25 메시지는 시각 갱신 없이 버려져(`localizer_node.py:253`) 그 구간 회전이 영구 유실.
  실제 wide 검출주기(GPU 직렬화 포함)가 0.25s를 넘는지 실측 필요.
  **수정: staleness 창을 실측 주기 위로 + 기각 프레임에서 prev 유지(델타 체인 보존).**
- **[확인/major] 보정 이중 발행 + in-flight 재보고** — wide/body 콜백 각각 보정 발행
  (`world_model_node.py:618,655`, 합산 ~24Hz), 스탬프 없는 파이프라인이라 보정 적용 직후 in-flight
  프레임이 같은 드리프트를 재보고 → 0.8 게인 과보정/진동 가능.
  **수정: 보정은 wide 콜백 1곳 + 적용 후 지연시간만큼 쿨다운.**
- **[확인/minor] dt 클램프 0.2s == HB 주기** — 지터 시 병진 체계적 과소적분(~12%),
  `<ODOM>`의 t_ms는 파싱도 안 함. **수정: 클램프 0.4s + t_ms 전파.**
- **[확인/minor] LK VO 상시 실행** — flow fresh여도 1640×1232 remap+LK+RANSAC이 30fps 전량 실행
  (`localizer_node.py:320` 순서), 결과는 버려짐. CPU 낭비·콜백 지연. **수정: flow_fresh 먼저 확인 후 스킵.**

### A2. 시간 동기화 (전 체인)

- **[확인/critical] 캡처 스탬프 2회 파괴 + 도착시점 pose 투영** — camera_csi가 pull 후 now(),
  `yolo_detector_node.py:202`가 추론 후 now()로 재작성, world_model은 stamp를 아예 안 읽고
  `self.robot_*`(최신 pose)로 투영. 지연 150-300ms × 회전 1.0rad/s = 8-17° 스미어 → 1m 물체 20-30cm
  접선 오투영. assoc_radius 0.15m 초과 시 중복 트랙/팬텀 앵커; 이내면 앵커 잔차가 '일관된 회전'으로
  보여 Umeyama가 지연을 드리프트로 오인 → theta_gain 0.7로 실회전 반대방향 보정 주입.
  **test_field에서 물체가 유일한 절대 기준이므로 이 폐루프가 스스로를 오염시키는 1순위 구조 결함.**
  **수정: 캡처 스탬프 전파(Image header → DetectionArray) + world_model pose 히스토리 deque 보간.
  스톱갭: |dθ/frame| 임계 초과 시 보정·신규트랙·클래스투표 억제.**
- **[확인/major] 이미지 구독 RELIABLE depth-10** — yolo/localizer/siglip 모두 기본 QoS.
  6MB×10장 큐잉 + 단일 GPU 락으로 '가장 오래된 프레임' 처리가 상시화 → 지연 예산 가산.
  **수정: 전 이미지 구독·발행을 sensor_data QoS(BEST_EFFORT, depth 1)로.**
- **[확인/major] 앵커 조기 잠금** — n_obs 5(합산 최대 24Hz = 0.3~1초)만으로 현재 pose 기준 동결
  (`world_model_node.py:807-815`). pose가 틀린 순간 잠긴 앵커가 이후 보정의 기준이 되는 자기확증.
  **수정: 잠금 조건에 최소 시간 스팬 + 복수 pose 관측 + 최근 보정 잔차 조건 추가.
  운용 대안: 기동 직후 정지 스캔으로 첫 앵커 세트 확정 후 주행.**

### A3. 분류·연관 (오픽 직결)

- **[확인/critical] CLASSIFY 게이트 공간 비연관** — `mission_fsm_node.py:365-385`는 프레임 전역
  최고-conf shape(`yolo_detector._publish_shape`)/siglip 결과를 current_target 것으로 간주.
  Classification.msg에 박스/트랙 id가 없어 확인 불가. ALIGN은 시간 대기뿐(비주얼 서보 없음).
  인접 distractor 시나리오에서 오픽(-40) 또는 진짜 타겟 오블랙리스트 직결.
  **수정: Classification에 박스 중심 px(또는 투영 xy) 추가 + FSM에서 타겟 반경 내일 때만 게이트 통과.
  단기: shape/siglip primary를 '이미지 중앙 반경 내'로 제한.**
- **[확인/major] approach 0.25m vs body 가시하한 0.29m + 게이트 0.7 스케일 혼용** —
  정지점에서 타겟이 body 캠 사각. 게이트 0.7이 (a) 도형 원시 conf(실측 0.6-0.8 분포 한가운데)와
  (b) siglip margin(노드 자체 임계는 0.5)이라는 다른 스케일에 동일 적용. 2초 창 실패 → 영구 blacklist.
  **수정: approach_dist 0.35~0.40m + 임계 분리(도형 ~0.6 / margin 0.5) + 창 최대값 판정 +
  타임아웃 시 1회 재정렬-재시도 후 blacklist.**
- **[확인/major] fpc>0 원샷 Set2 전환** — `world_model_node.py:841-844`, 단 1표(body≥0.50/wide≥0.85)
  오검출로 icosahedron 타겟이 영구 Set2 고정. **수정: 비율 규칙(fpc ≥ 0.3~0.5 × 최대 도형표) 또는 2표 이상.**
- **[확인/major] body 절대 우선 비가역 + selector 정확일치 필터** — body 오표 1개는 wide 정표
  누적으로 못 뒤집고(`world_model_node.py:846`), 오라벨된 set1 트랙은 `target_selector_node.py:89-93`에서
  explore 후보조차 안 되어 body 재검증 기회 자체가 소멸(타겟 starvation).
  **수정: body 가중합산(×3) 또는 n_body≥2 조건부 권위 + selector에서 저마진 set1 트랙 재검증 라우팅.**
- **[확인/major] SigLIP 이중 레이스** — (1) 크롭: 박스는 YOLO 추론 프레임, 크롭은 `latest_frame`
  (`siglip_gate_node.py:150`) → 이동 중 어긋난 영역 분류. (2) 부착: 결과가 stamp 매칭 없이
  `_last_body_fruit_id`(도착 시점 최신 프레임 트랙)에 부착(`world_model_node.py:670`).
  게다가 siglip은 workspace 게이트가 없어 원거리 과일 박스도 분류하는데 그 박스는 world_model에서
  투영 거부되므로 결과가 '몇 분 전 마지막 투영 트랙'에 붙을 수 있음. min_box_area 1200은
  1640×1232 기준 ~35×35px라 근거리 강제 의도를 전혀 강제하지 못함(~2-3m도 통과).
  **수정: 스탬프 링버퍼 크롭 + stamp→tid 매칭 부착 + id 리셋 + min_box_area 대폭 상향(0.68m 기준 ~2만px²)
  또는 준정지 시에만 분류.**
- **[확인/major] Greedy NN 연관** — 1:1 배정 제약 없음(`world_model_node.py:754`). 12cm 실물 2개
  영구 병합(octa/icosa 표 혼합), 같은 프레임 두 검출이 한 트랙에 중복 유입.
  **수정: 프레임 단위 거리 오름차순 1:1 배정, 트랙당 1건, 초과분은 신규 트랙.**

### A4. 수명주기·경계

- **[확인/major] forget_after 120s가 locked 앵커까지 삭제** — `world_model_node.py:1069`는
  blacklisted만 예외. 미션 타임라인상 초기 구역 120s 미복귀는 흔함 → 절대 보정이 조용히 꺼짐
  ('persist the whole-field map' 주석과 모순). **수정: locked 예외 1줄 (또는 forget_after 대폭 상향).**
- **[확인/major] wide 근거리 스킵 vs wide 음의증거 모순** — body 섹터 내 wide 검출은 연관 스킵
  (`world_model_node.py:610`)인데 `_apply_negative_evidence`(1061)는 그 트랙에 wide miss 감점 지속.
  body가 0.6s 깜빡이면 이중 감점 0.6/s → 실물 트랙 ~1.6s 만에 삭제.
  **수정: 음의증거 wide-miss 판정에서 body 섹터 내 트랙 제외(1줄).**
- **[확인/major] body 섹터(축상 ≤0.745m) vs workspace(≤0.68m) 밴드** — 0.68~0.745m에서 body 투영
  거부 + wide 연관 스킵 + 양캠 감점 동시 적용 → 정지 중 전방 ~0.7m 트랙 삭제·id 재생성(투표/anchor/
  blacklist 이력 소실). **수정: 두 경계 정합(body_fov_far ≈ 0.615 또는 ws x1 0.745).**
- **[확인/minor] wide 호모그래피 폴백 크래시** — `base`가 `_wide_H is not None` 분기에서만 바인딩,
  610행이 분기 밖에서 참조 → 폴백 경로 첫 검출에서 UnboundLocalError로 노드 사망.
  **수정: 루프 반복마다 `base = None` 초기화(1줄).**

### A5. 완결성 비평이 추가로 잡은 갭 (리뷰 관점들이 못 본 것)

1. **휠 '오도메트리'는 실측이 아니라 open-loop 명령 echo** — 스케일 캘리브 없음, 배터리 전압 의존,
   strafe 롤러 슬립, 정지마찰 데드존, 스톨 시 유령 이동. '병진 2배' 크기 추정도 이 위에 서 있음.
2. **장애물 회피 전무** — go_to_goal/explorer가 월드모델을 경로에 안 씀 → distractor를 밀어
   앵커(유일한 절대 기준)를 물리 이동시키고, 스톨 유령 오도 + 본 경기 접촉 감점 리스크.
3. **dry_pick 사람 개입 교란** — 매 픽 4s 동안 사람이 FOV에 들어옴: 가림 감점, 오검출 신규 트랙,
   object-flow/LK 오염 가능. 개입 프로토콜(FOV 밖 접근, 개입 중 감점 일시정지) 필요.
4. **자원 예산** — 실측 가용 489MB/swap 사용 중, SigLIP fp32 ~1.5GB, OOM 시 respawn 없음 +
   FSM은 분류 토픽 두절을 감지 못하고 timeout→blacklist로 진행(silent death).
5. **wide.pt가 당일 v4로 교체됐는데 임계값 세트(0.5/0.85/0.7)는 v3 분포 기준** — 재보정 미실시.
6. **고정 WB(shade + body 고정게인)** — test_field 조명이 다르면 SigLIP 과일 색 판별 계통 오류 가능.
   롤링셔터 모션블러로 회전 중 검출 소실(=object-flow가 점을 가장 필요로 하는 순간).
7. **현장 단일 장애점**: CH340 미영구화(리부팅 시 베이스 무동작), mcu_bridge read 스레드의
   spin_once 중복 호출 버그(단선 시 재접속 사망), nvargus 무복구, 리프트 마스트 무피드백
   (미상승 시 cam_height 0.885 가정 전체 붕괴) + 가감속 wobble.

## B. "이 방식이 최선인가?" — 대안 비교

| 대안 | 판정 | 근거 |
|---|---|---|
| 캡처 스탬프 + pose 히스토리 투영 | **채택(필수)** | 표준 관행, 저비용, 1순위 결함 직접 해결 |
| 병진/회전 소스 소유권 단일화 | **채택(필수)** | 융합 회계의 기본 |
| 프레임 단위 1:1 연관(간이 Hungarian) | **채택** | 수십 줄, 병합/중복 트랙 직접 해결 |
| 클래스 증거의 비율/가중 규칙(간이 Bayes) | **채택** | 비가역 결정 제거, 기존 투표 풀 재활용 |
| 3-DOF EKF(robot_localization 등) | **보류** | 공분산 없는 이질 소스라 이득 낮고 통합 리스크. 현 상보 구조로 충분 |
| GTSAM 팩터그래프 스무딩 | **기각** | 물체 5~8개·소필드에 과잉, Python/Orin 부담, 대회 전 위험 |
| SORT식 트랙별 Kalman | **선택** | 시간동기 수정 후에도 지터 남으면 도입(중간 비용) |
| ArUco 필드 앵커 | **조건부** | 본 경기장 룰 확인 필수. test_field에선 GT 계측용으로만(인식 성능 테스트 목적 훼손 방지) |
| BEV 시맨틱 그리드 | **기각** | 이산 트랙 대비 이점 없고 메모리/CPU 추가 |
| 단일 공용 YOLO 모델 | **기각** | 도메인별 학습 이점이 실증됨(현 방식 유지) |

결론: **구조 교체가 아니라 현 설계의 표적 수정.** 위 채택 항목이 전부 반영되면 이 스택은
해당 제약에서 사실상 최선의 형태다.

## C. 우선순위 로드맵

> **[2026-07-04 적용됨]** 아래 P0 #1(병진 소유권)과 P1 #8(캡처 스탬프+포즈 히스토리)을 코드로 수정 완료.
> - 병진: `object_flow_trans:false`(설정 플래그, 크루드) 대신 **더 나은 코드 게이트**를 채택 — object-flow가
>   신선하면 `on_wheel_odom`의 x/y 적분을 억제(flow가 병진 소유, 휠은 폴백). 회전 dedup과 대칭.
>   `object_flow_trans:true` 유지(grounded flow 병진 활용). E1 실측 후 소유권 재검토 여지는 남김.
> - 시간동기: `yolo_detector._build_array`가 입력 이미지 캡처 스탬프를 DetectionArray에 전파,
>   `world_model`이 포즈 히스토리 deque(`_pose_at` 보간)로 **캡처 시각 포즈**로 투영(wide/body 양경로).
>   랜드마크 보정 관측(_corr_pairs)도 자동으로 캡처시각 기준이 되어 회전-지연 오인 보정 제거.
> - 검증: 3파일 py_compile OK + `_pose_at`/게이트 로직 11개 유닛 케이스 통과(스미어 제거·각도 wrap·폴백 경계).
>   **런타임 실주행 검증(로봇 전원+카메라)은 미실시 — 필드 런 시 E1/회전 스윕으로 확인 필요.**
>
> **[2026-07-04 2차 배치 적용됨]** 인식 코드 8건 추가 수정(world_model_node.py, localizer_node.py만 —
> config·mission_fsm은 사용자 작업 중이라 미접촉). 검증 워크플로우가 REAL로 확정한 것들:
> - **wide-H 폴백 크래시**: `base=None` 루프 초기화(외재투영 폴백 시 UnboundLocalError로 노드 사망 방지).
> - **fpc 원샷 Set2 비가역**: `fpc>0` → **비율 규칙** `fpc>0 and fpc>=0.5*max(shape_votes)`. 진짜 과일큐브는
>   통과, 1프레임 오탐은 도형표에 도태(도형 타겟 영구 상실 차단).
> - **음의증거 wide-miss 모순**: body 섹터 안 트랙은 wide-miss 감점 제외(`not in_body`) — 보이는데 삭제되던 것 차단.
> - **forget_after가 locked 앵커 삭제**: 시간 기반 삭제에서 locked 예외(신뢰도 붕괴 삭제는 유지) — 전장 맵 지속.
> - **heading 게이트 병진 누출**: 게이트 시 dx,dy를 순수 평균변위(R=I)로 재계산 — 노이즈 theta 레버암 누출 제거.
> - **SigLIP 스테일 부착**: `_last_body_fruit_sec` 기록, 1s 초과 부착 거부(몇 분 전 트랙 오염 차단).
> - **dt 클램프 0.2→0.4s**: HB 주기와 동일해 정상 지터의 초과분이 체계적으로 잘리던 병진 과소적분 완화.
> - **LK VO 상시 실행**: flow 신선 + 벽보정 off면 VO 파이프라인 조기 스킵(CPU/타이머 경합 해소).
> - 검증: 2파일 py_compile OK + 신규 결정규칙 12개 유닛 케이스 통과.
>
> **[2026-07-04 3차: 최종검증 후 dedupe 적용]** judge가 지목한 최상위 잔여 리스크(중복/팬텀 트랙)를 수정:
> - **중복/팬텀 트랙 dedupe** (world_model `_merge_duplicates`, tick 서두): assoc_radius(0.15) 이내 두 트랙을
>   병합 — 오래된 id 유지, 투표풀 합산, seen_body/blacklisted/locked OR, conf max, n_obs 합. 두 confirmed
>   locked 앵커가 다른 클래스면 병합 안 함(진짜 별개 물체 보호). blacklist OR로 픽 후 재접근/중복카운트 차단.
>   검증: py_compile OK + 6개 유닛 케이스(분열병합·blacklist OR·원거리 미병합·locked 클래스보호·locked 상속).
> - CLASSIFY 공간연관·approach 0.35는 **사용자가 mission_fsm에 이미 반영**(judge가 워킹트리에서 확인).
> - **미접촉(사용자 영역/측정/일단위)**: P2 재시도카운터(fsm), P3 필드계측, P4 모델·임계 재보정,
>   P5 dry_pick 정합, body_fov_far config(측정 후), 프레임 1:1 승격(P7), log-odds 가역 투표(P8).

### P0 — 다음 런 전 (시간 단위, 대부분 1~10줄)
1. ~~병진 소유권~~ **[적용됨]** 코드 게이트로 dedup (위 노트)
2. `base = None` 루프 초기화 (폴백 크래시)
3. 음의증거: body 섹터 내 wide-miss 제외 + 섹터/workspace 경계 정합
4. `_forget_stale`에 locked 예외
5. approach_dist 0.35~0.40 + 게이트 임계 분리(도형 0.6/margin 0.5) + blacklist 전 1회 재시도
6. fpc 비율 규칙
7. staleness 창 상향(vo 0.6s/flow 1.0s, 실측 후 확정)

### P1 — 대회 전 (일 단위)
8. ~~**캡처 스탬프 전파 + pose 히스토리 보간**~~ **[적용됨, 위 노트]** — 회전 중 보정/신규트랙 억제 스톱갭은 미적용(선택)
9. **분류 공간 연관**: Classification에 박스 좌표 + FSM 근접 게이트 + SigLIP stamp 매칭·id 리셋
10. 이미지 QoS sensor_data 전환
11. 프레임 단위 1:1 연관
12. 앵커 잠금 강화(시간 스팬·복수 pose·잔차 조건)
13. 보정 단일 발행 + 쿨다운, heading 게이트 시 병진 재계산(순수 평균 변위)
14. wide_v4 conf 분포 재보정(E5)

### P2 — 여유 시
15. LK 스킵/다운스케일, 투표 감쇠, body 가중 권위, 간이 경로 회피, 노드 respawn+워치독,
    SigLIP fp16, CH340 영구화, 마스트 sanity check(전방 0.5/1.0m 테이프 프로브)

## D. 검증 실험 (완결성 비평 제안, 요약)

- **E1** open-loop 오도 스케일/슬립/스톨/배터리 캘리브 (병진 소유권 결정 근거)
- **E2** 앵커 120s 소크 (forget 수정 검증)
- **E3** 사람 개입 + 경로상 distractor 리허설
- **E4** 15분 자원 소크 + 노드 강제사망 복구
- **E5** 회전속도별 검출 dropout 곡선 + 조명/WB SigLIP 강인성 + v4 conf 분포
- **E6** 콜드부트 현장 체크리스트 + 투영 sanity 프로브 + 시리얼 재접속 + wobble 측정

## 커버리지 한계

behavior-integration·config-launch 전담 리뷰와 적대적 검증 에이전트·아키텍처 패널은 세션 토큰
한도로 미완(23:10 리셋 후 재개 가능). 단, 본 문서의 모든 발견은 세션 본체가 해당 코드 라인을
직접 읽어 확인했고, behavior 통합의 핵심(FSM 게이트, selector 라우팅)은 본체 정독으로 커버됨.
config-launch 전수 대조(파라미터 선언 vs yaml 오타·미사용 키)만 미수행.
