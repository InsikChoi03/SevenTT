# 로봇팔 집기·놓기 + 카메라 검증 — 인수인계 문서

작성일 2026-07-03. 2R 로봇팔로 **물체를 집고(집기) 뒤 보관함에 놓는(놓기)** 동작을 스크립트로 만들고,
**본체 카메라로 제대로 집었는지 검증**하는 도구까지 만든 작업 정리. 다음 사람이 이어서 하도록.

---

## 0. 한 줄 요약
- **`scripts/arm_pick2r.py`** — 2R 팔(어깨/손목/그리퍼) **집기·놓기** 재사용 모듈 + CLI. 실측·검증 완료(“잘 잡는다” 확인). ✅
- **`scripts/pick_verify.py`** — 본체캠으로 **집기 성공 판정**(잡은 뒤 앞에서 사라졌나). ✅ 작동
  - **놓기 성공 판정**(광각캠 중심영역)은 구현했으나 **보류** — 광각 top-down 뷰에서 도형이 검출이 안 됨(아래 §5).
- 오늘 **펌웨어는 안 바꿈**(모터 속도 실험했다가 원복, 플래시 안 함). 보드는 기존 `base_arm_combined` 그대로.

---

## 1. 용어 / 전제 (몰라도 되게)
| 용어 | 뜻 |
|---|---|
| **2R 팔** | 관절 2개(어깨피치+손목피치) + 그리퍼. 좌우 조준은 팔이 아니라 **메카넘 베이스 회전**이 함. 수직 평면 안에서만 움직임. |
| **통합보드** | 아두이노 1개(CH340 UNO, `/dev/ttyUSB0`)가 **베이스(휠)+팔(서보)+리프트모터** 다 제어. 펌웨어 `firmware/base_arm_combined`. |
| **`<ARM,t1,t2,t3,…>`** | 팔 명령. 서보각 0~180. **t1=ch0 어깨, t2=ch1 손목, t3=ch2 그리퍼** (t4~6=90 필러, 안 씀). |
| **boot-limp** | 팔은 **부팅 시 무구동**(안전). **첫 `<ARM>` 명령에 그 값으로 즉시 스냅**한 뒤 이후 부드럽게 보간. |
| **smooth 보간** | 펌웨어가 목표각으로 **MAX_STEP 2°/30ms(≈67°/s)** 속도제한으로 천천히 이동. 호스트는 목표각만 보내면 됨. |
| **본체캠 / 광각캠** | 본체캠=sensor-id **0**(베이스 고정, **앞 픽존**을 봄, 저왜곡). 광각캠=sensor-id **1**(상단 마스트, **로봇을 내려다보는 어안**). |
| **집기(grip) / 놓기(release)** | 집기=앞의 물체를 잡음. 놓기=잡은 걸 뒤(로봇 본체 보관함)에 떨어뜨림. |

---

## 2. `scripts/arm_pick2r.py` — 집기·놓기 (핵심 산출물)

### 2-1. 확정 서보각 (2026-07-03 실측, “잘 잡는다” 확인)
| 자세 | 어깨(ch0) | 손목(ch1) | 그리퍼(ch2) | 의미 |
|---|---|---|---|---|
| **INIT** (연결직후/대기) | 88 | 10 | 140 | 첫 `<ARM>`에 여기로 스냅(boot-limp 흡수) |
| **집기 도달** | 10 | 170 | (열림 80) | 앞으로 뻗어 내려감 |
| **집기 닫기** | 10 | 170 | **150** | 잡기 |
| **놓기 도달** | 110 | 30 | (잡은 채 150) | 뒤로 넘어감 |
| **놓기 열기** | 110 | 30 | **80** | 놓기 |

- **그리퍼: 열림 = 80, 닫힘 = 150** (숫자 작을수록 열림).
- 값은 `arm_pick2r.py` 상단 `INIT / PICK / PLACE / GRIP_OPEN / GRIP_CLOSED` 상수. 바꾸려면 여기만.

### 2-2. 동작 순서 (급하게 잡던 문제 해결됨)
집기·놓기 각 이동은 **“팔이 실제로 도착할 때까지 기다린 뒤 → 딜레이 → 그리퍼”** 순으로 동작한다.
- **왜 필요했나:** 예전엔 이동 명령 후 고정 0.8초만 기다렸는데, INIT→집기는 손목이 10→170으로 **160°**나 움직여
  67°/s로 **~2.4초** 걸린다. 팔이 도착하기 전에 그리퍼가 닫혀서 “너무 급하게” 잡았음.
- **지금:** `_move()`가 **이동거리 ÷ `SERVO_SPEED_DPS`(60°/s, 안전마진)** 로 도달시간을 계산해 그만큼 기다리고,
  그 뒤 `move_delay`(0.8s)/`grip_delay`(0.6s)를 추가로 쉰 다음 그리퍼를 움직인다.
- **놓기 후:** `init_delay`(기본 **3초**) 대기 후 INIT 복귀.

### 2-3. 사용법
```bash
# CLI (실행 즉시 INIT 88,10,140 으로 스냅 → 그리퍼 아래 손/물건 없는지 확인!)
python3 scripts/arm_pick2r.py                 # 집기 → 놓기 1회
python3 scripts/arm_pick2r.py --only grip      # 집기만
python3 scripts/arm_pick2r.py --only release   # 놓기만
python3 scripts/arm_pick2r.py --loop 3          # 집기→놓기 3회
python3 scripts/arm_pick2r.py --move-delay 1.0 --grip-delay 0.8 --init-delay 3
```
```python
# 모듈 (추후 pick_run / mission_fsm 등에서 재사용)
from arm_pick2r import Arm2R
with Arm2R("/dev/ttyUSB0") as arm:   # 열면서 INIT로 스냅
    arm.grip()      # 집기
    arm.release()   # 놓기
    arm.go_init()   # 대기자세 복귀
```

---

## 3. `scripts/pick_verify.py` — 카메라 검증

### 3-1. 원리
본체캠은 **베이스 고정으로 앞 픽존**만 본다(뒤 보관함은 시야 밖). 그래서 **집기 전/후 앞을 비교**한다.
- **집기 판정 (본체캠 sensor0, `models/cube.pt` 5클래스):**
  1. 집기 전 앞 캡처 → YOLO 검출 → **타깃 도형 식별**(가장 큰/앞의 것).
  2. 집기·놓기 실행.
  3. 다시 캡처 → 타깃이 픽존에서 **사라졌으면 성공**(집어서 뒤로 옮김) / 그대로면 **실패**.
- **놓기 판정 (광각캠 sensor1, `models/wide.pt`, 중심영역) — 보류:** §5 참조.
- nested 처리: `cube` 박스 안에 `fruit_photo_cube` 패치가 있으면 그 물체는 과일큐브로 해석.
- 저장 이미지: `data/pick/verify_before.png`, `verify_after.png`(집기), `verify_place.png`(놓기).

### 3-2. 모드
```bash
python3 scripts/pick_verify.py                 # 집기(+놓기) 전체, 팔 동작 포함
python3 scripts/pick_verify.py --check-only     # 팔 없이 '지금 앞(본체)에 뭐 있나'만
python3 scripts/pick_verify.py --wide-check      # 팔 없이 '광각 중심에 뭐 있나'만 (놓기존 튜닝용)
python3 scripts/pick_verify.py --no-arm          # before→[엔터 후 수동 픽앤플레이스]→판정
python3 scripts/pick_verify.py --no-place         # 집기 판정만
```

### 3-3. 검증 결과 (오늘 실측)
- **집기 판정 로직 정상 작동 확인.** 본체캠이 **정십이면체(dodecahedron) conf 0.97~0.98** 로 정확히 검출·타깃 선택.
- 실제 사이클 돌렸을 때 **“집기 실패”** 로 정확히 판정됨 → 이유는 버그가 아니라 **아래 §4** 때문(물체가 4px도 안 움직임 = 팔이 물체를 못 건드림).

---

## 4. ⚠️ 중요 발견 — 지금 집기는 “고정 블라인드 자세”
`arm_pick2r`의 집기는 **카메라로 본 위치로 팔을 보내는 게 아니라, 정해진 한 지점(어깨10/손목170)만** 잡는다.
- 즉 **물체가 그 고정 grab 지점에 정확히 놓여 있어야** 실제로 잡힌다.
- 검증 도구는 이걸 **정확히 잡아낸다**: 물체가 안 움직이면 “집기 실패”로 정직하게 리포트.
- **실제 검출한 물체를 집으려면 둘 중 하나 필요:**
  - **(A)** 물체를 고정 집기 지점에 맞춰 놓고 사용(지금 방식, 가장 단순).
  - **(B)** 비전 유도 집기: 호모그래피로 픽셀→팔 좌표 변환해 검출 위치로 팔 이동.
    `scripts/pick_run.py`가 이 방식이지만 **구 5-DOF `arm_ik` API(HOME_CMD/servo_cmd/GRIPPER_OPEN 등) 참조 → 새 2R `arm_ik`엔 그 함수들이 없어 현재 미작동.** 새 2R용 서보 캘리브(HOME_CMD/DIR/개폐값)를 채워야 함(arm_ik.py 하단 TODO).

---

## 5. 🚧 보류: 놓기(광각) 검증 — 왜 막혔나
- 사용자 지정 방식: **놓은 물체가 광각캠 화면 중심영역에 들어오면 놓기 성공.**
- 광각캠 확인 결과 = **상단 마스트에서 로봇을 내려다보는 어안뷰**(로봇 본체·보관함이 화면 중앙). 놓는 위치를 보는 **관점 자체는 맞다.**
- **문제: `wide.pt`가 이 top-down 근접·어안 뷰의 도형을 못 잡는다.** conf 0.2까지 낮춰도, `cube.pt`로 바꿔도 **검출 0**.
  - 이유 추정: `wide.pt`는 광각캠이 **멀리 있는 필드**를 볼 때로 학습됨 → 로봇 바로 위에서 크게·왜곡되어 보이는 도형은 **학습 분포 밖**.
- **다음 선택지:**
  - **(A)** 광각 top-down 뷰 도형 사진 모아 `wide.pt` 재학습(또는 보관함 전용 소형 모델).
  - **(B)** 보관함 위치가 고정이니 도형 분류 없이 **“중심에 물건이 생겼나”만 단순 검출**(색/blob/차영상)로 대체 — 종류는 집기 단계(본체캠)에서 이미 알고 있음.
  - **(C)** 집는 순간 **본체캠으로 그리퍼에 물체가 물렸나** 확인(관점 전환).
- 구현은 이미 `pick_verify.py`에 들어가 있음(`--center-frac` 로 중심영역 크기 조절). 검출만 되면 바로 동작.

---

## 6. 참고 — 시리얼 / 하드웨어 메모
- 포트 **`/dev/ttyUSB0`** (CH340). 팔·베이스·리프트 전부 이 한 포트.
- **리프트 모터**(광각 마스트 올리는 MOSFET, D13): `<LIFT,ms>` = ms 동안 ON 후 자동 OFF(최대 10초),
  `<LIFT,0>`=즉시 OFF. `<LIFTACK,ms>` 즉시 → `ms` 뒤 `<LIFT,done>`. **오늘 0.9/1.5/2.0/3.0초 다 정상 확인.**
- 카메라: cv2에 GStreamer 없어 **gi/appsink**(`scripts/csi_capture.py`). 본체=WB 고정게인 적용, 광각=보정 없음.
  광각(sensor1)은 부팅 probe가 가끔 -121 → rebind/재부팅 필요할 수 있음(오늘은 정상 오픈됨).
- **펌웨어 원복 주의:** 오늘 “손목만 빠르게”(관절별 MAX_STEP) 실험했다가 **원복**함. 보드엔 **안 올림**.
  다시 하려면 `firmware/base_arm_combined`의 `MAX_STEP`을 배열로 바꾸고 `arm_pick2r.SERVO_SPEED_DPS`도 관절별로 맞춘 뒤 `tools/arduino-cli … upload --fqbn arduino:avr:uno -p /dev/ttyUSB0`.

---

## 7. 다음 할 일 (우선순위)
1. **놓기 검증 방식 결정** — §5의 (A)재학습 / (B)단순검출 / (C)그리퍼확인 중 택1.
2. (선택) **비전 유도 집기** — §4의 (B). 새 2R `arm_ik` 서보 캘리브(HOME_CMD/DIR/개폐값) 채우고 `pick_run.py` 2R로 손보기.
3. 집기 성공률: 물체를 고정 grab 지점에 두고 `arm_pick2r --loop` 로 반복 성공률 확인.

## 관련 파일
- `scripts/arm_pick2r.py` (집기/놓기), `scripts/pick_verify.py` (검증), `scripts/arm_jog2.py` (수동 조그, 값 실측용)
- `scripts/csi_capture.py`, `scripts/camera_config.py` (카메라), `models/cube.pt`(본체), `models/wide.pt`(광각)
- `firmware/base_arm_combined/base_arm_combined.ino` (통합 펌웨어)
- `ros2_ws/src/robot_control/robot_control/arm_ik.py` (2R IK, 서보 캘리브는 TODO)
