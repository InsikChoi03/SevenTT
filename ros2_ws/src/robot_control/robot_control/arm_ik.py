"""Scipia A2T — 2R 수직평면 팔 IK (+ 그리퍼).  순수 파이썬(ROS 무관) — import/단독실행 둘 다.

2026-07-02 팔 재구성: **베이스 yaw 관절 제거**(좌우 조준은 메카넘 베이스 회전으로),
**어깨피치 + 손목피치 2관절(2R)** + 그리퍼 개폐. 수직 평면 안에서만 움직인다.
(구 5-DOF: 베이스yaw+어깨+팔꿈치+손목피치+손목roll → 폐기)

기구학 (실측 2026-07-02, cm):
  SHOULDER_Z = 16.4   어깨(팔) 회전축 높이 (바닥 기준, 실측 164mm)
  L1 = 14.8           팔: 어깨축 → 손목축 (148mm, 손목모터가 이 링크 끝)
  L2 = 15.47          그리퍼: 손목축 → 그리퍼 끝 tip (154.7mm)   (= GRIPPER_TIP)

관절: θ1 어깨피치, θ2 손목피치(상대), 그리퍼 개폐(IK 밖).
  φ = θ1 + θ2 = 접근 피치 (0=수평, 아래로 −).
  ⚠️ 2R(2자유도)는 점(reach r, 높이 z)만 도달 — **접근각 φ는 독립 지정 불가**(도달점이 φ를 결정).
     해가 둘(손목 down/up)이고 각각 φ가 다름 → 픽 로직이 상황맞는 해를 골라 쓴다.

각도 규약(도):
  shoulder θ1 : 0 = 수평(+r 앞 방향), 위로 +
  wrist    θ2 : 팔 기준 상대, 0 = 일직선(팔과 그리퍼 일자)
  r (reach)  : 앞으로 + (베이스 yaw 없음 → x,y 대신 '앞 거리' r 과 높이 z 로 표현)
  z          : 바닥 기준 높이 (cm)

⚠️ 서보 매핑(채널·HOME_CMD·HOME_IK·DIR)과 그리퍼 개폐값은 **새 하드웨어로 재보정 필요** — 하단 TODO.
"""
from __future__ import annotations

import math

# ---- 링크 길이 (cm) — 2026-07-02 2R 재구성 ----
SHOULDER_Z = 16.4     # 어깨 회전축 높이 (바닥 → 축, 실측 164mm)
L1 = 14.8             # 팔: 어깨축 → 손목축 (148mm)
L2 = 15.47            # 그리퍼: 손목축 → 그리퍼 끝 tip (154.7mm)
GRIPPER_TIP = L2      # 손목축 → 그리퍼 끝(=L2). 잡는점이 끝보다 안쪽이면 grab-z로 관리


def fk(shoulder_deg, wrist_deg):
    """순기구학: (어깨θ1, 손목θ2) deg → (reach r, 높이 z[바닥기준], 접근피치 φ) cm/deg."""
    t1 = math.radians(shoulder_deg)
    t12 = math.radians(shoulder_deg + wrist_deg)
    r = L1 * math.cos(t1) + L2 * math.cos(t12)
    z = SHOULDER_Z + L1 * math.sin(t1) + L2 * math.sin(t12)
    return r, z, math.degrees(t12)


def ik(reach, z, wrist_up=False):
    """역기구학: (reach r 앞+, 높이 z 바닥기준) → (어깨θ1, 손목θ2, 접근피치 φ) deg. 도달불가 None.

    2R이라 접근각은 못 고름 — wrist_up=False(손목 아래꺾)/True(위꺾) 두 해 중 선택, 각 φ 다름.
    """
    zr = z - SHOULDER_Z
    d2 = reach * reach + zr * zr
    c2 = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if c2 < -1.0 or c2 > 1.0:
        return None  # 도달 거리 밖
    s2 = math.sqrt(max(0.0, 1.0 - c2 * c2))
    if not wrist_up:
        s2 = -s2
    t2 = math.atan2(s2, c2)
    t1 = math.atan2(zr, reach) - math.atan2(L2 * s2, L1 + L2 * c2)
    return (math.degrees(t1), math.degrees(t2), math.degrees(t1 + t2))


# ---- 관절 가동범위 (기구학각 deg) — ⚠️ 새 서보 실측 전 '잠정값', 재보정 필요 ----
JOINT_LIMITS = {
    0: (-30.0, 150.0),    # shoulder θ1 (잠정)
    1: (-150.0, 150.0),   # wrist θ2 상대 (잠정)
}


def in_limits(shoulder_deg, wrist_deg):
    for j, a in ((0, shoulder_deg), (1, wrist_deg)):
        lo, hi = JOINT_LIMITS[j]
        if a < lo - 1e-6 or a > hi + 1e-6:
            return False
    return True


def ik_checked(reach, z, prefer_down=True):
    """도달 + 가동범위 보장. 손목 down/up 두 해 중 가능한 것. 실패 None.

    prefer_down=True: 손목 아래꺾 해 우선(대개 위에서 내려 잡기 유리).
    """
    order = (False, True) if prefer_down else (True, False)
    for wu in order:
        sol = ik(reach, z, wrist_up=wu)
        if sol is not None and in_limits(sol[0], sol[1]):
            return sol
    return None


def reach_at_height(z):
    """높이 z(바닥기준)에서 앞으로 도달 가능한 최대 reach r (cm). 도달 불가면 0."""
    zr = z - SHOULDER_Z
    rmax2 = (L1 + L2) ** 2 - zr * zr
    return math.sqrt(rmax2) if rmax2 > 0 else 0.0


# ---- 서보 명령 변환 -------------------------------------------------------
# ⚠️⚠️ 새 팔(2R) 서보 재보정 필요 — 아래는 채워야 하는 자리표시자. 사용자 확인 대기:
#   1) 서보 채널: 어깨피치=?, 손목피치=?, 그리퍼=?
#   2) HOME 자세(예: 팔 수평 또는 접힌 상태)에서 각 서보 명령값(HOME_CMD) + 그때 기구학각(HOME_IK)
#   3) DIR 부호(서보+ 방향 = 기구학각 + 방향인지)
#   4) 그리퍼 GRIPPER_OPEN / GRIPPER_CLOSED (집게 교체돼 구값 5/70 무효)
# 명령[j] = HOME_CMD[j] + DIR[j] * (기구학각[j] - HOME_IK[j])
_CALIBRATED = False   # 위 값 확정 전 True로 바꾸지 말 것
SERVO_CH = {"shoulder": None, "wrist": None, "gripper": None}
# HOME_CMD/HOME_IK/DIR = TODO (사용자 실측 후 채움)


def _selftest():
    print(f"2R 팔: SHOULDER_Z={SHOULDER_Z}  L1(팔)={L1}  L2(그리퍼)={L2} cm")
    print(f"어깨축 기준 최대 도달반경 L1+L2 = {L1 + L2:.2f} cm")
    print(f"바닥(z=0) 앞 최대 reach ≈ {reach_at_height(0):.2f} cm,  어깨높이(z={SHOULDER_Z}) ≈ {reach_at_height(SHOULDER_Z):.2f} cm\n")

    print("=== FK→IK→FK 왕복 (수학 일관성) ===")
    worst = 0.0
    for (s, w) in [(20, -40), (0, -30), (45, -60), (-10, 25), (60, -80)]:
        r, z, phi = fk(s, w)
        sol = ik(r, z, wrist_up=(w > 0))
        if sol is None:
            print(f"  IK 실패: (어깨{s},손목{w})"); continue
        r2, z2, phi2 = fk(sol[0], sol[1])
        err = max(abs(r - r2), abs(z - z2))
        worst = max(worst, err)
        print(f"  (어깨{s:+.0f},손목{w:+.0f}) → reach={r:6.2f} z={z:6.2f} φ={phi:6.1f}  복원오차 {err:.1e}cm")
    print(f"  최대 위치오차 {worst:.1e}cm → {'OK' if worst < 1e-6 else '확인필요'}\n")

    print("=== 목표점(reach, z) → 관절각 (도달/범위) ===")
    for name, r, z in [("앞20 높이5", 20, 5), ("앞15 높이2", 15, 2), ("앞25 높이10", 25, 10),
                       ("앞10 높이0(바닥)", 10, 0), ("앞30 높이5(밖?)", 30, 5)]:
        sol = ik_checked(r, z)
        if sol is None:
            print(f"  {name:18s}: 도달불가/범위밖")
        else:
            print(f"  {name:18s}: 어깨={sol[0]:6.1f} 손목={sol[1]:6.1f} 접근φ={sol[2]:6.1f} deg")


if __name__ == "__main__":
    _selftest()
