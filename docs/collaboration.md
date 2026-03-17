# 협업 워크플로우 가이드

## 전체 개요

노트북에서 코드를 작성하고 젯슨에서 테스트하는 **Push-Pull-Test** 사이클을 기반으로 합니다.

```
[노트북]                          [젯슨 오린 나노]
  │                                     │
  │  1. 코드 작성 (VS Code)             │
  │  2. git push                        │
  │──────────────────────────────────►  │
  │                                     │  3. git pull
  │                                     │  4. python scripts/run_inference.py
  │                                     │  5. 결과 확인 / 로그 확인
  │  ◄──────────────────────────────────│
  │  6. 결과 리뷰 후 수정               │
  │  (반복)                             │
```

---

## 브랜치 전략

```
main
 └── develop          ← 팀 통합 개발 브랜치 (PR을 통해서만 main으로 병합)
      ├── feature/detection-module
      ├── feature/motor-control
      └── fix/camera-pipeline
```

| 브랜치 | 용도 | 규칙 |
|--------|------|------|
| `main` | 젯슨에서 항상 동작하는 안정 코드 | 직접 push 금지, PR만 허용 |
| `develop` | 팀 통합 작업 | 기능 완성 후 PR |
| `feature/*` | 개인 기능 개발 | 완성 후 develop에 PR |
| `fix/*` | 버그 수정 | 수정 후 develop에 PR |

### 브랜치 생성 및 작업 흐름

```bash
# 1. 새 기능 시작
git checkout develop
git pull origin develop
git checkout -b feature/나의-기능명

# 2. 작업 후 커밋
git add src/detection/detector.py
git commit -m "feat: TensorRT 추론 클래스 구현"

# 3. develop에 PR 생성
git push origin feature/나의-기능명
# → GitHub에서 PR 생성 → 팀원 리뷰 → develop에 병합
```

---

## 커밋 메시지 규칙

```
<타입>: <요약> (50자 이내)

예시:
feat: TensorRT FP16 추론 클래스 구현
fix: CSI 카메라 GStreamer 파이프라인 오류 수정
docs: 모델 변환 가이드 업데이트
refactor: 전처리 함수 CUDA 가속 적용
test: detector 전처리 단위 테스트 추가
chore: requirements.txt 패키지 버전 고정
```

| 타입 | 사용 시점 |
|------|-----------|
| `feat` | 새 기능 추가 |
| `fix` | 버그 수정 |
| `docs` | 문서 변경 |
| `refactor` | 기능 변화 없는 코드 개선 |
| `test` | 테스트 추가/수정 |
| `chore` | 빌드, 설정 파일 변경 |

---

## 개발 환경 설정 (최초 1회)

### 노트북 (코드 작성 환경)

```bash
git clone <repo-url>
cd SevenTT

# 노트북에서는 TRT 없이 전처리 등 CPU 로직만 테스트
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
pip install pytest  # 테스트 실행용
```

### 젯슨 (실행 환경)

```bash
git clone <repo-url>
cd SevenTT
chmod +x setup.sh
./setup.sh                     # --system-site-packages venv 생성
source venv/bin/activate
python scripts/download_models.py
```

---

## 일상적인 작업 루틴

### 노트북에서 작업 시작

```bash
git checkout develop
git pull origin develop         # 최신 코드 동기화 (항상 먼저)
git checkout -b feature/내-기능
source venv/bin/activate

# 코드 작성 ...

# 노트북에서 가능한 테스트 먼저 실행 (전처리, 유틸 등)
pytest tests/ -v -k "not TRT"

git add .
git commit -m "feat: 기능 설명"
git push origin feature/내-기능
```

### 젯슨에서 테스트

```bash
cd SevenTT
source venv/bin/activate
git pull origin feature/내-기능    # 또는 develop

python scripts/run_inference.py --test   # 10프레임 테스트
# 또는 VS Code의 F5 (launch.json의 "Run Inference" 설정)
```

### 결과 공유 방법

- 성능 수치(FPS, 정확도)는 PR 설명 또는 팀 채널에 기록
- 오류 발생 시 `logs/app.log` 내용을 첨부

---

## 모델 파일 공유 규칙

```
Git으로 공유 (O)      Git으로 공유 (X)
─────────────────     ─────────────────────────────
소스 코드             *.pt, *.onnx, *.engine (대용량)
config/*.yaml         data/ (데이터셋)
requirements.txt      checkpoints/ (학습 체크포인트)
docs/                 logs/, results/
```

모델 파일은 **Google Drive 팀 폴더**에 업로드하고,
`scripts/download_models.py`의 파일 ID를 업데이트한 후 커밋하세요.

---

## 트러블슈팅 체크리스트

### 젯슨에서 코드가 동작하지 않을 때

```
[ ] git pull을 했는가?
[ ] source venv/bin/activate를 했는가?
[ ] 모델 파일이 models/ 에 있는가? (python scripts/download_models.py)
[ ] config/hardware_config.yaml의 카메라/포트 설정이 맞는가?
[ ] 전원 모드가 MAXN인가? (sudo nvpmodel -q)
```

### 노트북에서 테스트가 실패할 때

```
[ ] TRT 관련 테스트는 @pytest.mark.skipif 로 건너뛰어지는가?
[ ] venv가 활성화되어 있는가?
[ ] requirements.txt 가 최신인가? (pip install -r requirements.txt)
```

---

## VS Code Remote-SSH 팁

- `.vscode/launch.json`은 Git으로 공유되므로 **F5 키로 바로 실행** 가능
- `Ctrl+Shift+~` 로 젯슨 터미널을 열어 실시간 로그 확인
- 파일 저장 단축키 `Ctrl+S` 후 젯슨 터미널에서 즉시 실행하는 루틴 권장

```
[노트북 VS Code 왼쪽]   [젯슨 터미널 오른쪽]
 소스 편집               $ python scripts/run_inference.py --test
 Ctrl+S 저장             (결과 실시간 확인)
 git commit & push       $ git pull && python ...
```
