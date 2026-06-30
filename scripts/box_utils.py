"""검출 박스 중복 정리 — 클래스 무관 NMS + 포함(containment) 억제.

한 물체에 박스가 겹쳐 붙거나(클래스 2개 동시), 큰 박스 안에 작은 박스가 들어간 경우
낮은 신뢰도 쪽을 제거. 단 '서로 다른 물체가 일부 가려져 겹치는' 정도(중간 IoU,
서로 포함 아님)는 보존한다.
"""
from __future__ import annotations


def _area(b):
    return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])


def dedupe(dets, iou_thr=0.6, iomin_thr=0.7, same_cls_iomin=0.45):
    """dets: [(x1,y1,x2,y2,cls,conf), ...] → 중복 제거(신뢰도 높은 것 우선).

    낮은 신뢰도 박스가 이미 채택된 박스와
      - IoU >= iou_thr (거의 같은 위치), 또는
      - IoMin >= 임계 (교집합/작은박스면적 — 작은 박스가 큰 박스 안/대부분 겹침; 양방향)
    이면 버린다. IoMin 임계는 클래스 인지형:
      - 같은 클래스 겹침 = 거의 확실히 같은 물체 → same_cls_iomin(0.45, 적극 병합)
      - 다른 클래스 겹침 = 다른 물체 부분가림일 수도 → iomin_thr(0.7, 보수적)
    """
    out = []
    for d in sorted(dets, key=lambda d: d[5], reverse=True):   # conf 내림차순
        a = _area(d)
        drop = False
        for k in out:
            ix1, iy1 = max(d[0], k[0]), max(d[1], k[1])
            ix2, iy2 = min(d[2], k[2]), min(d[3], k[3])
            inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
            if inter <= 0:
                continue
            ak = _area(k)
            union = a + ak - inter
            iou = inter / union if union > 0 else 0.0
            iomin = inter / min(a, ak) if min(a, ak) > 0 else 0.0
            thr = same_cls_iomin if d[4] == k[4] else iomin_thr   # 같은 클래스면 적극 병합
            if iou >= iou_thr or iomin >= thr:
                drop = True
                break
        if not drop:
            out.append(d)
    return out
