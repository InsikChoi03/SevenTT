# 태극기 깃발 데이터 수집·라벨링·학습 절차

경기장 종료지점의 태극기 깃발을 `arrival` 클래스로 추가한다. 최종 클래스 순서는 아래처럼 고정한다.

```text
0 cube
1 octahedron
2 dodecahedron
3 icosahedron
4 fruit_photo_cube
5 arrival
```

## 1. WASD 촬영 + JSON 내보내기

젯슨에서 로봇 주변 안전공간을 확보한 뒤 실행한다. 키를 한 번 누를 때마다 약 10cm 이동하거나 짧게 제자리 회전하고, 정지 후 body/wide 사진을 한 장씩 저장한다.

```bash
python3 scripts/flag_capture_export.py --session flag0711
```

실행 중 라이브 화면은 브라우저에서 연다.

```text
http://<jetson-ip>:8090/
```

- `w/s`: 전진/후진 약 10cm
- `a/d`: 좌/우 횡이동 약 10cm
- `j/l`: 제자리 좌/우 회전
- `q`: 정지 후 기존 `models/cube.pt`, `models/wide.pt`로 pre-label하고 zip 생성
- `Ctrl-C`: 즉시 정지, 자동 export는 생략

기본값은 `--speed 0.375`, `--move-dur 0.29`, `--strafe-dur 0.56`, `--turn-dur 0.25`다. 바닥 상태에 따라 실제 이동거리가 다르면 duration을 조금씩 조정한다.

포트가 겹치면 예를 들어 아래처럼 바꾼다.

```bash
python3 scripts/flag_capture_export.py --session flag0711 --mjpeg-port 8091
```

라이브 송출을 끄고 촬영만 할 때는 `--mjpeg-port 0`을 붙인다.

결과 위치:

```text
data/flag_capture/<session>/
  body/images/
  wide/images/
  labeled_json/
    body/*.jpg + *.json + classes.txt
    wide/*.jpg + *.json + classes.txt
    manifest.json
  <session>_body_wide_labeled_json.zip
```

기존 촬영 세션을 다시 내보낼 때:

```bash
python3 scripts/flag_capture_export.py --session flag0711 --export-only --overwrite-export
```

## 2. X-AnyLabeling 교정

`labeled_json/body`, `labeled_json/wide`를 각각 X-AnyLabeling에서 연다. 기존 모델이 달아둔 도형/과일패치 박스는 틀린 것만 고치고, 태극기 깃발은 `arrival` rectangle로 추가한다.

## 3. YOLO 학습셋 변환

교정이 끝난 JSON 폴더를 YOLO 형식으로 변환한다.

```bash
python3 scripts/labelme_to_yolo.py \
  --src data/flag_capture/flag0711/labeled_json/body \
  --out data/flag_capture/flag0711/body_yolo \
  --classes cube,octahedron,dodecahedron,icosahedron,fruit_photo_cube,arrival

python3 scripts/labelme_to_yolo.py \
  --src data/flag_capture/flag0711/labeled_json/wide \
  --out data/flag_capture/flag0711/wide_yolo \
  --classes cube,octahedron,dodecahedron,icosahedron,fruit_photo_cube,arrival
```

기존 누적 학습셋과 병합할 때도 같은 클래스 순서를 유지한다. 기존 0-4 클래스 id는 그대로 두고, 새 깃발만 5번으로 추가한다.

## 4. 기존 최종 학습셋과 병합

이전까지 학습에 쓴 최신 누적 zip은 아래 2개를 기준으로 한다.

```text
body: dataset_body_cube_v6.zip
wide: dataset_wide_v5.zip
```

`s0704pm1_*`, `s0704pm2_*` 같은 개별 라벨 zip은 위 최신 누적본에 이미 포함되어 있으므로 중복 병합하지 않는다. 벽 segmentation용 `wallseg*` zip도 이번 detection 모델에는 섞지 않는다.

```bash
mkdir -p data/arrival_merge/base_body data/arrival_merge/base_wide
unzip -o dataset_body_cube_v6.zip -d data/arrival_merge/base_body
unzip -o dataset_wide_v5.zip -d data/arrival_merge/base_wide

python3 scripts/merge_datasets.py \
  --out data/arrival_merge/dataset_body_flag_v7/dataset \
  --classes cube,octahedron,dodecahedron,icosahedron,fruit_photo_cube,arrival \
  --src data/arrival_merge/base_body/dataset \
  --src data/arrival_merge/body_yolo

python3 scripts/merge_datasets.py \
  --out data/arrival_merge/dataset_wide_flag_v6/wide_dataset \
  --classes cube,octahedron,dodecahedron,icosahedron,fruit_photo_cube,arrival \
  --src data/arrival_merge/base_wide/wide_dataset \
  --src data/arrival_merge/wide_yolo

cd data/arrival_merge/dataset_body_flag_v7
zip -r ../../../dataset_body_flag_v7.zip dataset

cd ../dataset_wide_flag_v6
zip -r ../../../dataset_wide_flag_v6.zip wide_dataset

cd /home/seventt/seventt/workspace
```

## 5. Colab 학습

- 본체 모델: `scripts/colab_train_body_flag_v7.ipynb`
- 광각 모델: `scripts/colab_train_wide_flag_v6.ipynb`

학습 후 `best.pt`를 각각 `models/cube.pt`, `models/wide.pt`로 교체한다. 런타임에서는 새 모델의 `arrival` 검출이 `set_type=3` 도착점/깃발 표식으로 월드모델에 들어간다.
