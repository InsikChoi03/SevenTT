# 모델 파일 관리

이 디렉토리의 모델 파일은 **Git으로 추적하지 않습니다.**
파일 크기와 TensorRT 엔진의 기기 종속성 때문입니다.

## TensorRT 엔진 주의사항

`.engine` 파일은 **빌드한 젯슨의 GPU 아키텍처에 종속**됩니다.
다른 기기에서 빌드한 엔진을 그대로 복사해서 사용하면 오류가 발생합니다.
반드시 각 기기에서 직접 `scripts/build_engine.py`를 실행해 빌드하세요.

## 모델 다운로드 방법

```bash
# 자동 다운로드 스크립트 (팀 공유 링크 설정 필요)
python scripts/download_models.py
```

스크립트가 준비되지 않은 경우, 아래 경로에 직접 파일을 배치하세요:

| 파일 | 설명 | 획득 방법 |
|------|------|-----------|
| `models/yolov8n.pt` | YOLOv8 원본 가중치 | Ultralytics 공식 배포 |
| `models/yolov8n.onnx` | ONNX 변환 파일 | `scripts/export_onnx.py` 실행 |
| `models/yolov8n.engine` | TRT 엔진 (젯슨 전용) | `scripts/build_engine.py` 실행 |

## 모델 변환 파이프라인

```
yolov8n.pt  →(export_onnx.py)→  yolov8n.onnx  →(build_engine.py)→  yolov8n.engine
```

자세한 변환 방법은 [docs/model_conversion.md](../docs/model_conversion.md)를 참고하세요.
