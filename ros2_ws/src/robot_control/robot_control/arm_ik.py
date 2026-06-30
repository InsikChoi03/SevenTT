"""Scipia A2T 5-DOF 위치결정 IK (+ 그리퍼).  순수 파이썬(ROS 무관) — import/단독실행 둘 다.

기구학 (실측 2026-06-07, 집게 교체 후 갱신 2026-06-24, cm):
  L0 = 1.0    베이스 회전축 → 어깨 축 (수직 위로, 실측 2026-06-24, 구 6.0). ※table_z(축→테이블, 아래로)=-11.5 는 별개
  A1 = 10.5   어깨 → 팔꿈치
  A2 = 11.0   팔꿈치 → 손목(피치) 축  (구 9.5 → 갱신)
  A3 = 17.0   손목 축 → 집게 잡는 곳(IK 타겟점). = 손목→집게회전축 4 + 집게회전축→잡는곳 13 (실측 2026-06-24).

관절(서보 채널): 0 베이스yaw, 1 어깨, 2 팔꿈치, 3 손목피치, 4 손목roll, 5 그리퍼.
  - 1/2/3 은 피치 → 베이스yaw 회전 후 '수직 평면' 안의 평면 3R 팔
  - 4(roll) 은 그리퍼를 축 둘레로 돌림 → 축상의 끝점 위치엔 영향 없음(자세만)
  - 5 = 그리퍼 개폐, IK 밖

여기 각도는 '깨끗한 기구학 좌표계'(도). 실제 서보 명령으로의 변환은 별도 보정
단계(servo_cmd) — home 보정으로 HOME_CMD/HOME_IK/DIR 채우면 됨.

각도 규약(도):
  base yaw θ0 : 0 = +x, CCW +
  shoulder θ1 : 0 = 수평(+r 방향), 위로 +
  elbow    θ2 : upper-arm 기준 상대, 0=일직선, elbowDown = 음수
  wrist    θ3 : 상대. 누적 φ = θ1+θ2+θ3 = 접근 피치(0=수평, 아래로 음수)
"""
from __future__ import annotations

import math

# ---- 링크 길이 (cm) ----  (집게 교체 + 베이스높이 실측, 갱신 2026-06-24)
L0 = 1.0            # 베이스 회전축 → 어깨 축 (수직 위로, 실측 2026-06-24, 구 6.0). table_z=-11.5는 별개(축→테이블)
A1 = 10.5
A2 = 11.0           # 팔꿈치→손목 (구 9.5 에서 갱신)
A3 = 17.0           # 손목축→집게 잡는곳 = 손목→집게회전축 4 + 회전축→잡는곳 13 (실측 2026-06-24, 구 21 폐기)
GRIPPER_TIP = 17.0  # 손목→집게 잡는곳(=A3). 손가락 끝은 조금 더(미측정) — 박힘은 grab-z로 관리

# ---- 관절 가동범위 = 서보 12~168° (양끝 12° 마진: 하드스톱 회피로 서보 보호) ----
# 사용자 피드백(2026-06-07): 서보 고장 방지 위해 최대각(0/180)은 웬만하면 안 씀.
# θ = HOME_IK + (servo-HOME_CMD)/DIR, servo∈[12,168]
SERVO_MARGIN = 12
JOINT_LIMITS = {
    0: (-58.0, 98.0),    # base    servo 12~168 (home 70)
    1: (6.0, 162.0),     # shoulder servo 12~168 (home 96)
    2: (-113.0, 43.0),   # elbow   servo 12~168 (home 55, DIR -1)
    3: (-113.0, 43.0),   # wrist   servo 12~168 (home 55, DIR -1)
}


def fk(theta0, theta1, theta2, theta3):
    """순기구학: 기구학각(deg) → 그리퍼 끝 (x,y,z) cm + 접근피치 φ(deg)."""
    t0 = math.radians(theta0)
    p1 = math.radians(theta1)
    p12 = math.radians(theta1 + theta2)
    p123 = math.radians(theta1 + theta2 + theta3)
    r = A1 * math.cos(p1) + A2 * math.cos(p12) + A3 * math.cos(p123)
    z = L0 + A1 * math.sin(p1) + A2 * math.sin(p12) + A3 * math.sin(p123)
    return r * math.cos(t0), r * math.sin(t0), z, math.degrees(p123)


def ik(x, y, z, approach_deg, elbow_up=False):
    """역기구학: 끝점(x,y,z) cm + 접근피치(deg, 0=수평/음수=아래) → (θ0,θ1,θ2,θ3) deg.

    도달 불가하면 None. (가동범위 체크는 ik_checked 사용)
    """
    t0 = math.atan2(y, x)
    r = math.hypot(x, y)
    phi = math.radians(approach_deg)

    # 손목점 = 끝점에서 접근방향으로 A3 만큼 뒤로 (어깨 기준 평면좌표)
    rw = r - A3 * math.cos(phi)
    zw = (z - L0) - A3 * math.sin(phi)

    # 2R IK (A1, A2) for (rw, zw)
    c2 = (rw * rw + zw * zw - A1 * A1 - A2 * A2) / (2 * A1 * A2)
    if c2 < -1.0 or c2 > 1.0:
        return None  # 도달 거리 밖
    s2 = math.sqrt(max(0.0, 1.0 - c2 * c2))
    if not elbow_up:
        s2 = -s2
    t2 = math.atan2(s2, c2)
    t1 = math.atan2(zw, rw) - math.atan2(A2 * math.sin(t2), A1 + A2 * math.cos(t2))
    t3 = phi - t1 - t2
    return (math.degrees(t0), math.degrees(t1), math.degrees(t2), math.degrees(t3))


def in_limits(angles):
    """기구학각 튜플(θ0..θ3)이 가동범위 안인가."""
    for j, a in enumerate(angles):
        lo, hi = JOINT_LIMITS[j]
        if a < lo - 1e-6 or a > hi + 1e-6:
            return False
    return True


def ik_checked(x, y, z, approach_deg):
    """도달 + 가동범위까지 보장. elbowDown 우선, 안되면 elbowUp 시도. 실패시 None."""
    for eu in (False, True):
        sol = ik(x, y, z, approach_deg, elbow_up=eu)
        if sol is not None and in_limits(sol):
            return sol
    return None


def ik_planar(reach, z, approach_deg, yaw_deg=0.0, elbow_up=False):
    """베이스 yaw 고정 + 수직평면 내 '부호있는 reach'(앞+/뒤−)·높이 z 로 IK.

    뒤(reach<0)는 **어깨(허리)를 뒤로 꺾어** 도달 (베이스 안 돌림). x,y 대신 평면 reach 직접.
    반환 (θ0,θ1,θ2,θ3) deg 또는 None(도달거리밖).
    """
    phi = math.radians(approach_deg)
    rw = reach - A3 * math.cos(phi)
    zw = (z - L0) - A3 * math.sin(phi)
    c2 = (rw * rw + zw * zw - A1 * A1 - A2 * A2) / (2 * A1 * A2)
    if c2 < -1.0 or c2 > 1.0:
        return None
    s2 = math.sqrt(max(0.0, 1.0 - c2 * c2))
    if not elbow_up:
        s2 = -s2
    t2 = math.atan2(s2, c2)
    t1 = math.atan2(zw, rw) - math.atan2(A2 * math.sin(t2), A1 + A2 * math.cos(t2))
    t3 = phi - t1 - t2
    return (yaw_deg, math.degrees(t1), math.degrees(t2), math.degrees(t3))


def ik_checked_planar(reach, z, approach_deg, yaw_deg=0.0):
    """ik_planar + 가동범위. elbowDown 우선. 실패시 None."""
    for eu in (False, True):
        sol = ik_planar(reach, z, approach_deg, yaw_deg, eu)
        if sol is not None and in_limits(sol):
            return sol
    return None


# ---- 서보 명령 변환 (home 보정 2026-06-07) -------------------------------
# 명령[j] = HOME_CMD[j] + DIR[j] * (기구학각[j] - HOME_IK[j])
# 채널: 0 base, 1 shoulder, 2 elbow, 3 wristPitch | 4 wristRoll(자세), 5 gripper(개폐)
HOME_CMD = [70, 96, 55, 55, 90, 5]     # home (서보 0~180). ch4=90 손목roll중립, ch5=5 집게열림(구90→CLOSED70 초과 과부하라 변경)
# home 자세 = 팔이 직상(위). 그때 위치관절(0~3)의 기구학각:
HOME_IK = [0.0, 90.0, 0.0, 0.0]        # θ0,θ1,θ2,θ3 (deg): θ1=90=수직, 나머지 0=일직선
DIR = [1, 1, -1, -1]                   # 실측 확정(2026-06-07): ch0 왼/ch1 뒤=+ , ch2/ch3 반대=−
WRIST_ROLL_HOME = HOME_CMD[4]          # 90 (자세 중립, IK 밖)
GRIPPER_HOME = HOME_CMD[5]             # 90 (부팅 중립, IK 밖)
GRIPPER_OPEN = 5                       # 제일 열림 (집게 교체 후 실측 2026-06-24, 구 35)
GRIPPER_CLOSED = 70                    # 닫힘 (실측 2026-06-24, 구 130)
# ⚠️ HOME_CMD[5]=90 은 새 CLOSED(70)보다 20° 더 닫는 값 → 부팅/HOME 때 집게 서보 과부하.
#    arm_ik HOME_CMD[5] + 펌웨어 arm_servo.ino HOME[5] 둘 다 5~70 내 안전값으로 바꿔야 함(아래 참고).


def servo_cmd(joint, kin_deg):
    """위치관절(0~3)의 기구학각(deg) → 실제 서보 명령(0~180)."""
    cmd = HOME_CMD[joint] + DIR[joint] * (kin_deg - HOME_IK[joint])
    return max(0.0, min(180.0, cmd))


# ---- 자가 검증 -----------------------------------------------------------
def _selftest():
    print(f"링크: L0={L0} A1={A1} A2={A2} A3={A3} (cm)")
    reach_max = A1 + A2 + A3
    print(f"이론 최대 도달반경 ≈ {reach_max:.1f} cm\n")

    # FK→IK→FK 왕복: elbowDown 해(θ2<0) 위주로 표본
    cases = [
        (30, 60, -80, -40),
        (0, 45, -60, -30),
        (-45, 80, -100, 10),
        (60, 30, -40, -50),
        (15, 100, -120, 0),
    ]
    print("=== FK→IK→FK 왕복 (수학 일관성) ===")
    worst = 0.0
    for (a0, a1, a2, a3) in cases:
        x, y, z, phi = fk(a0, a1, a2, a3)
        sol = ik(x, y, z, phi, elbow_up=False)
        if sol is None:
            print(f"  IK 실패: angles={a0,a1,a2,a3}")
            continue
        x2, y2, z2, phi2 = fk(*sol)
        err = max(abs(x - x2), abs(y - y2), abs(z - z2))
        worst = max(worst, err)
        print(f"  목표각{(a0,a1,a2,a3)} → 끝점({x:6.2f},{y:6.2f},{z:6.2f}) φ={phi:6.1f}"
              f" → IK복원 끝점오차 {err:.2e} cm")
    print(f"  최대 위치오차: {worst:.2e} cm  →  {'OK (수학 일치)' if worst < 1e-6 else '확인필요'}\n")

    # 좌표 직접 IK 예시 (작업영역 감각)
    print("=== 좌표 → 관절각 (도달/가동범위 체크) ===")
    targets = [
        ("앞 20cm, 높이 8cm, 수평접근", 20, 0, 8, 0),
        ("앞 15 오른10 높이5, 아래30°", 15, -10, 5, -30),
        ("앞 12, 높이 2, 수직아래", 12, 0, 2, -90),
        ("앞 30(거의 최대), 높이 6", 30, 0, 6, 0),
        ("앞 40 (범위 밖 예상)", 40, 0, 6, 0),
    ]
    for name, x, y, z, ap in targets:
        sol = ik_checked(x, y, z, ap)
        if sol is None:
            print(f"  {name:28s}: 도달불가/범위밖")
        else:
            print(f"  {name:28s}: θ=({sol[0]:6.1f},{sol[1]:6.1f},{sol[2]:6.1f},{sol[3]:6.1f}) deg")


if __name__ == "__main__":
    _selftest()
