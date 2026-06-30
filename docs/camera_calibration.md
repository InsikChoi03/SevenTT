# 광각 카메라 캘리브레이션 가이드

상단 광각 카메라(160°, `sensor-id=0`)의 내부 파라미터(intrinsics)와 왜곡 계수를 구해
`perception.yaml`에 채우는 절차. 이게 끝나야 `world_model_node`가 픽셀→지면 좌표 투영을 하고,
`localizer_node`가 왜곡 보정/절대 자세 추정을 합니다. (캘리브레이션 전엔 두 노드가 좌표를 못 냄.)

> 모델: 표준 OpenCV 핀홀 + 방사/접선 왜곡(`cv2.calibrateCamera`/`cv2.undistort`).
> 노드들(`localizer`=undistort, `world_model`=핀홀 ray)이 이 모델을 쓰기 때문에 동일 모델로 캘리브.

---

## 0. 준비물 — 체커보드

- **체커보드 패턴** 인쇄물. 기본값은 **내부 코너 9×6** (= 사각형 10×7칸).
  - 출처: OpenCV 샘플 `samples/data/chessboard.png`, 또는 https://calib.io/pages/camera-calibration-pattern-generator
  - **A4보다 A3 권장** (광각이라 보드가 화면을 충분히 채워야 함).
- 인쇄물을 **단단한 평판**(폼보드/하드보드/클립보드)에 휘지 않게 부착. 휘면 캘리브 망함.
- 사각형 한 변 실측(mm). 기본 25mm. **이 값은 intrinsics(fx/fy/cx/cy)·왜곡엔 영향 없음**(외부 자세 스케일만) — 대충 맞아도 됨.
- 다른 크기 보드를 쓰면 `--cols/--rows/--square-mm`로 바꿔서 두 스크립트에 **동일하게** 전달.

---

## 1. 촬영 — `calib_capture.py`

카메라를 **경기 때 장착될 위치(리프트 위 ~80cm)에 고정한 상태**로 찍는 게 이상적
(그 자세의 왜곡·시야를 그대로 반영). 모니터(:1)에 미리보기 창이 뜹니다.

```bash
cd ~/seventt/workspace
DISPLAY=:1 python3 scripts/calib_capture.py            # sensor-id 0, 9x6, data/calib/top 에 저장
```

- 보드 전체가 검출되면 미리보기가 **초록 + "BOARD OK"** → **SPACE**로 저장.
- **q / ESC** 종료.
- **좋은 데이터셋(중요):** 약 **15~25장**, 이렇게 다양하게:
  - 화면 **네 모서리/가장자리**에 보드를 놓기 (왜곡이 가장 큰 영역 — 광각은 여기가 핵심)
  - 보드를 **기울여서**(상하좌우 틸트) — 평면들이 다양한 각도
  - **거리 변화**(가까이/멀리), 화면의 1/3 이상 채우기
  - 같은 자세 중복 ❌, 흔들림/모션블러 ❌
- 카메라 마운트가 회전 장착이면 `--flip 1`(또는 3) 추가 — 캘리브 자체엔 무관하나 보기 편함.

다른 보드 예:
```bash
DISPLAY=:1 python3 scripts/calib_capture.py --cols 7 --rows 5 --square-mm 30
```

---

## 2. 풀이 + 기입 — `calib_solve.py`

```bash
# 먼저 dry-run으로 결과/오차 확인
python3 scripts/calib_solve.py
# 결과가 괜찮으면 perception.yaml에 기입
python3 scripts/calib_solve.py --write
```

출력 예:
```
used 21/22 images
RMS reprojection error = 0.42 px   (good)
fx=820.13 fy=819.7 cx=812.4 cy=614.9
dist_coeffs = [-0.31, 0.10, 0.0007, -0.0003, -0.018]
saved data/calib/top_calib.npz and data/calib/undistort_sample.png
```

- **`undistort_sample.png` 꼭 열어볼 것** (좌=원본, 우=보정). **휘었던 벽/담장 모서리가 직선**이 되면 성공.
- **RMS 판정:** `< 1.0 px` 양호, `< 0.5` 우수. 높으면 → 가장자리 샷 더 찍거나 흐린 컷 제거 후 재실행.
- 160°라 표준 모델이 극단 가장자리에서 부족하면:
  - `python3 scripts/calib_solve.py --rational --write` (8계수 모델), 또는
  - 중앙 영역만 크롭해서 사용(아키텍처 메모의 "중앙 크롭" 전략).

`--write`가 바꾸는 것: `perception.yaml`의 `localizer`·`world_model` 두 블록 **`top_fx/fy/cx/cy`** 와
`localizer`의 **`dist_coeffs`**. (주석·구조 보존, 값만 교체)

---

## 3. 반영 + 검증

```bash
cd ~/seventt/workspace/ros2_ws
colcon build --packages-select robot_bringup
source install/setup.bash
# world_model이 "projection disabled" 경고 없이 객체 좌표를 내는지 확인
ros2 launch robot_bringup perception.launch.py
ros2 topic echo /world_model        # objects[].x, y 가 채워지면 성공
```

- 캘리브 전: `world_model` 로그에 `top-cam intrinsics unset ... projection disabled`.
- 캘리브 후: 경고 사라지고 `/world_model`의 `objects`에 필드 좌표(x,y) 채워짐.

---

## 4. 본체 카메라(sensor-id=1)는?

본체 eye-in-hand 카메라는 **절대 좌표 투영에 안 쓰임**(비주얼 서보잉=상대 정렬)이라 정밀 캘리브 필수 아님.
저왜곡 M12 렌즈라 왜곡도 작음. 필요해지면 동일 절차로 `--sensor-id 1 --out data/calib/body` 촬영 후 별도 활용.

---

## 빠른 요약
```bash
DISPLAY=:1 python3 scripts/calib_capture.py     # 보드 들고 ~20장 (SPACE)
python3 scripts/calib_solve.py                   # 결과 확인 + undistort_sample.png 점검
python3 scripts/calib_solve.py --write           # perception.yaml 기입
cd ros2_ws && colcon build --packages-select robot_bringup   # 반영
```

관련: `scripts/calib_capture.py`, `scripts/calib_solve.py`, `ros2_ws/src/robot_bringup/config/perception.yaml`
