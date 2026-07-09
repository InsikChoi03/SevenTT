# 벽-바닥 경계 Segmentation 학습 준비

## 목적

기존 `wall_localizer_node.py`의 Canny/Hough/LSD/Lab 색상 기반 후보 생성을 보조하기 위해, `wall_floor_boundary` segmentation 모델을 별도 학습한다.

이 모델은 최종 위치 보정을 직접 만들지 않는다. 모델은 벽-바닥 접선 후보 mask만 제공하고, 최종 보정 판단은 기존 4x4 경기장 기하 게이트(`x=±2`, `y=±2`)가 맡는다.

## 라벨링

- 도구: X-AnyLabeling
- Export: `YOLO Segmentation`
- 클래스: `wall_floor_boundary`
- 형상: `Polygon`
- 두께: 경계 중심 기준 약 5~15px
- 제외: 가려진 부분, 로봇, 물체, 케이블, 사람, 강한 그림자

저장소 작업 폴더:

```text
data/wallseg_export/
  classes.txt
  images/
  labels/
```

현재 `images/`에는 wide 카메라 이미지가 들어 있으며, 라벨링 완료 후 같은 stem의 `.txt`를 `labels/`에 둔다.

YOLO segmentation label 예:

```text
0 0.123 0.456 0.130 0.460 0.140 0.462
```

## Zip 생성

```bash
cd /home/seventt/seventt/workspace
zip -r data/wallseg_dataset_v1.zip data/wallseg_export
```

## Colab 학습

1. Colab에서 `scripts/colab_train_wallseg_v1.ipynb`를 연다.
2. `data/wallseg_dataset_v1.zip` 또는 X-AnyLabeling export zip을 업로드한다.
3. 셀 순서대로 실행한다.
4. 다운로드된 파일명을 `wall_floor_boundary_seg_v1.pt`로 유지한다.

로봇 복사:

```bash
scp wall_floor_boundary_seg_v1.pt seventt@10.42.0.1:/home/seventt/seventt/workspace/models/
```

기존 detection 모델인 `models/wide.pt`, `models/cube.pt`는 절대 덮어쓰지 않는다.

## 이후 구현 방향

`wall_localizer_node.py`에 segmentation hook을 추가할 때는 fallback을 유지한다.

- `use_segmentation_mask: false`
- `segmentation_model_path: ""`
- `segmentation_input_width`
- `segmentation_input_height`
- `segmentation_min_confidence`
- `segmentation_run_rate_hz`
- `segmentation_fallback_to_edges: true`

모델이 없거나 mask 후보가 부족하면 기존 `_line_candidates()` 경로를 그대로 사용한다.
