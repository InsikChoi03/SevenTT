# 흰 도형 인식 모델 — 인수인계 문서

작성일 2026-06-24. 본체 카메라로 **흰색 다면체 4종을 검출·분류하는 YOLO 모델**을 만들고 개선하는 작업의 전체 정리.

---

## 0. 한 줄 요약
로봇이 본체 카메라로 바닥 위 **흰 도형 4종(정육면체/8면체/12면체/20면체)**이 **어디 있고(검출) 어떤 종류인지(분류)**를 알아야 한다.
→ 직접 학습시킨 **YOLOv8n** 모델(`models/cube.pt`)이 이걸 한다. **사진 모아서 라벨 달고 → 학습 → 배포 → 테스트**를 반복(=라운드)하며 정확도를 올린다. 현재 **라운드2까지 완료**.

---

## 1. 왜 이렇게 하는가 (배경)
- 도형이 **특징 없는 흰색**이라 기성 방법이 다 실패했음:
  - YOLO-World(말로 찾는 AI), 고전 영상처리(색/윤곽), ORB(특징점) → 흰 매끈한 면을 못 잡거나 종류를 못 가림.
- 결론: **이 4종 도형 전용으로 작은 검출기를 직접 학습**시키는 게 정답. (그게 지금의 `cube.pt`)
- 큰 프로젝트(AI 로봇 챌린지)의 "인식" 파트. 이 외에 ROS2·로봇팔·주행 등이 있지만 이 문서는 **도형 인식**만 다룸.

---

## 2. 용어 사전 (몰라도 되게)
| 용어 | 뜻 |
|---|---|
| **YOLO / YOLOv8n** | 이미지에서 물체 위치(박스)+종류를 한 번에 찾는 AI. `n`=nano=제일 가벼운 버전(젯슨에서 빠름). |
| **검출(detection)** | "어디에 물체가 있다" = 네모 박스(bounding box) 그리기. |
| **분류(classification)** | "그 물체가 무엇이다" = cube/octa/dodeca/icosa 중 하나 라벨링. |
| **라벨(label)** | 사진에 대한 정답. 각 이미지마다 `.txt` 파일 하나. 한 줄 = `클래스번호 중심x 중심y 폭 높이`(0~1 비율). 클래스: **0=cube, 1=octahedron(8면), 2=dodecahedron(12면), 3=icosahedron(20면)**. |
| **라벨링(labeling)** | 사람이 사진에 정답 박스를 그리는 작업. 도구 = **labelImg**(노트북에서). |
| **pre-label(자동 사전 라벨)** | 기존 모델이 먼저 박스를 대충 달아주고, 사람은 **틀린 것만 고침**. 처음부터 다 그리는 것보다 빠름. |
| **학습(training)** | 라벨 단 사진들로 모델을 만드는 것. 우리는 **Colab**(구글 무료 GPU)에서 함. 젯슨은 느려서 안 함. |
| **train / val** | 학습용(train)과 검증용(val) 데이터를 나눔. val은 **학습에 안 쓴 사진**으로 성적 매김 → 정직한 점수. |
| **augmentation(증강)** | 학습 중 사진을 좌우반전/색변경/모자이크 등으로 변형해 다양성 늘림. 파일을 늘리는 게 아니라 매번 랜덤 적용. |
| **mAP50 / mAP50-95** | 검출 정확도 점수(0~1, 높을수록 좋음). mAP50=박스 대충 맞으면 인정, mAP50-95=위치까지 정밀해야 인정(더 빡셈). |
| **Precision/Recall** | Precision=검출한 것 중 진짜 비율(오탐 적나), Recall=있는 것 중 잡은 비율(놓침 적나). |
| **conf(confidence)** | 모델 확신도. 이 값 이상만 검출로 인정(보통 0.25~0.3). |
| **dedupe** | 같은 물체에 박스가 중복으로 잡힌 걸 제거. `box_utils.py`. |
| **본체캠 / 광각캠** | 본체캠=sensor-id 0(주력, 지금 쓰는 것), 광각캠=sensor-id 1(160° 천장뷰, 아직 전용 데이터 없음). |

---

## 3. 핵심: 자가학습 루프 (self-training loop)
모델을 한 번에 완성하는 게 아니라 **반복(라운드)**으로 키운다:

```
 ① 사진 촬영        (젯슨, base_yaw_capture.py / batch_capture.py)
        ↓
 ② 현재 모델로 자동 pre-label   (젯슨, yolo_label_dir.py --model models/cube.pt)
        ↓
 ③ 노트북 labelImg로 틀린 것만 교정   (노트북, USB로 옮겨서)
        ↓
 ④ 기존 신뢰 데이터셋 + 교정분 합치기   (젯슨)
        ↓
 ⑤ Colab에서 새로 학습 → best.pt    (Colab, colab_train_yolo.ipynb)
        ↓
 ⑥ best.pt를 젯슨 models/cube.pt로 교체(배포)
        ↓
 ⑦ live 테스트 (live_yolo.py) → 약한 장면 발견 → ①로
```
**중요 원칙:** 매 라운드 이전 모델에 *이어서* 학습하지 않고, **누적된 전체 데이터로 사전학습 백본(yolov8n.pt)에서 새로** 학습한다. (교정한 내용이 깨끗이 반영되게)

---

## 4. 폴더 / 파일 지도
**작업 위치 분담:** 젯슨=촬영·추론·자동라벨, 노트북=labelImg 교정, Colab=학습. (젯슨 labelImg는 죽고, 젯슨 학습은 느림)

### 모델 (`models/`)
| 파일 | 의미 |
|---|---|
| `cube.pt` | **지금 실제로 쓰는 모델** (현재 = 라운드2) |
| `cube_v3.pt` | 라운드2 백업 |
| `cube_v2.pt` | 라운드1 백업 |
| `cube_v1_backup.pt` | 최초 모델 백업 |

### 데이터셋 (`data/`)
| 폴더 | 의미 |
|---|---|
| `dataset/` | 작업용 원본(626장). 촬영분 다 들어있음 — **불량/미정리 섞여 있어 학습에 직접 쓰지 말 것** |
| `dataset_trusted/` | 라운드1 신뢰 학습셋(404장, 불량 6장 제외판) |
| **`dataset_v2/`** | **라운드2 최종 학습셋(620장)** = trusted(404) + 교정한 신규(216) |
| `dataset_v2_2026-06-24.zip` | 위를 Colab에 올리는 zip (USB에도 복사됨) |

각 dataset 폴더 구조: `images/`(사진) + `labels/`(.txt 정답) + `classes.txt` + `data.yaml`.

### 스크립트 (`scripts/`)
| 파일 | 역할 |
|---|---|
| `base_yaw_capture.py` / `batch_capture.py` | 사진 촬영 (베이스 회전하며 여러 장). `--yolo` 주면 촬영하며 자동 pre-label |
| `yolo_label_dir.py` | **이미 찍은 사진**을 현재 모델로 일괄 자동 pre-label (라벨 없는 것만) |
| `live_yolo.py` | 모델로 **실시간 검출** (GUI 창 또는 `--shot`으로 한 장 저장) |
| `box_utils.py` / `dedupe_labels.py` | 중복 박스 제거 |
| `colab_train_yolo.ipynb` | **Colab 학습 노트북** (이걸 Colab에 올려서 학습) |
| `autolabel.py` | (구버전) FastSAM+SigLIP 자동라벨. 지금은 cube.pt pre-label이 주력 |

---

## 5. 절차 (실제 명령어)

### A. 촬영한 사진 자동 pre-label (젯슨)
```bash
cd ~/seventt/workspace
python3 scripts/yolo_label_dir.py --model models/cube.pt --conf 0.3
# → data/dataset/labels/ 에 .txt 생성 (라벨 없던 것만)
```

### B. 노트북으로 교정
- pre-label된 이미지+라벨을 USB로 노트북에 옮김
- **labelImg** 실행 → Open Dir=images, Change Save Dir=labels → 틀린 박스만 수정 → 저장
- 다시 USB로 젯슨에 회수

### C. 합치기 (젯슨) — 이건 보통 Claude가 해줌
- `dataset_trusted`(신뢰분) + 교정한 신규 = 새 학습셋 폴더 + zip 생성

### D. Colab 학습
1. https://colab.research.google.com 에서 `scripts/colab_train_yolo.ipynb` 열기
2. 런타임 → GPU 켜기
3. **셀1**: 데이터셋 zip 업로드(자동 압축해제, 기존거 자동삭제)
4. **셀2**: train/val 분리 + 설정파일 생성 *(반드시 실행 — 안 하면 셀3 에러)*
5. **셀3**: 학습 (`name='cube_v3'`처럼 라운드마다 이름 다르게)
6. **셀4**: `best.pt` 다운로드

### E. 배포 (젯슨) — 보통 Claude가 해줌
```bash
cd ~/seventt/workspace/models
cp cube.pt cube_v<이전>.pt          # 기존 백업
cp ~/Downloads/best.pt cube_v<새>.pt
cp cube_v<새>.pt cube.pt            # 교체
```

### F. live 테스트
```bash
DISPLAY=:1 python3 scripts/live_yolo.py --model models/cube.pt --conf 0.25   # 실시간 GUI창 (q/ESC 종료)
# 또는 한 장만:
python3 scripts/live_yolo.py --model models/cube.pt --shot /tmp/test.jpg
```
⚠️ `--model models/cube.pt`를 **꼭 명시** — 안 하면 옛날 runs/ 모델을 잡음.

---

## 6. 현재 상태 (2026-06-24)
- **라운드2 모델 배포 완료** (`models/cube.pt` = cube_v3)
- 성능 (held-out val 기준):

| 지표 | 라운드1 | 라운드2 |
|---|---|---|
| Precision | 0.934 | **0.983** |
| Recall | 0.960 | **0.984** |
| mAP50 | 0.982 | **0.991** |
| mAP50-95 | 0.949 | 0.946 |

→ 라운드2가 오탐·놓침 줄어 더 좋음. 학습 데이터 620장(cube 689/octa 307/dodeca 319/icosa 302 박스 + 배경 61장).

---

## 7. 다음에 할 일 / 약점
- **dodeca(12면) ↔ icosa(20면) 구분**이 역사적 최약점 (둘 다 둥근 다면체). 라운드2에서 표본 늘려 개선 중 — live에서 계속 확인.
- 약한 장면(틀리거나 놓치는 구도) 발견 시 → 그 구도로 더 촬영 → 라운드3.
- **광각캠(sensor-id 1)** 인식은 아직 미완 — 본체캠 모델 그대로 못 씀(왜곡·시점 다름). 전용 데이터 필요.
- 데이터 합칠 때 **'원본'은 `dataset_trusted`/`dataset_v2`(정리된 것)** 사용. `dataset/`(626장 원본)엔 불량이 섞여 있으니 직접 학습 금지.

---

## 8. 자주 겪는 함정
- **Colab `train/images 없음` 에러** → 셀2(분리)를 안 돌렸거나, `/content/dataset`에 이전 라운드 잔재가 섞임. 셀1(자동삭제 후 재추출)→셀2→셀3 순서로 다시.
- **live_yolo가 옛 모델 잡음** → 항상 `--model models/cube.pt` 명시.
- **labelImg가 젯슨에서 죽음** → 노트북에서 할 것.
- **서보/카메라 등 하드웨어** 함정은 별도 메모리 참고(이 문서 범위 밖).
</content>
