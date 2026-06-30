"""카메라 역할 ↔ nvargus sensor-id 중앙 매핑.

배선/포트가 바뀌면 **이 파일의 BODY/WIDE 숫자만** 고치면 모든 스크립트에 반영된다.
(nvargus sensor-id 번호 자체는 device-tree 열거 순서로 고정 — 여기서 '역할'만 매핑.)

현재 물리 배선 (2026-06-10):
  - 본체 eye-in-hand 카메라 = sensor-id 0  (포트 10-0010, 정상)
  - 상단 광각 카메라        = sensor-id 1  (포트 9-0010, marginal — 부팅 probe 가끔 -121 → rebind 필요)
원래(과거) 가정은 0=광각/1=본체였으나 포트 점검 중 뒤집힘. 광각 포트 안정화되면 다시 검토.
"""

BODY = 0   # 본체 eye-in-hand
WIDE = 1   # 상단 광각(top)

# 역할별 화이트밸런스 고정 게인 (B, G, R). 본체 IMX219는 마젠타 캐스트 보정 필요(reference-body-cam-whitebalance).
# 광각은 특성 미검증 → None(보정 안 함). sensor-id로 조회.
WB_GAINS = {
    BODY: (1.16, 1.08, 0.82),
    WIDE: None,
}
