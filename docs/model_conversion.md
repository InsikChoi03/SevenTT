# 모델 변환 가이드

PyTorch (.pt) → ONNX (.onnx) → TensorRT (.engine) 변환 과정을 설명합니다.

## 변환 파이프라인

```
[노트북 또는 젯슨]          [반드시 젯슨에서]
yolov8n.pt  →(1단계)→  yolov8n.onnx  →(2단계)→  yolov8n.engine
```

- **1단계 (ONNX 변환)**: 노트북 또는 젯슨 어디서나 실행 가능
- **2단계 (TRT 빌드)**: 반드시 실제 젯슨에서 실행 (아키텍처 종속)

## 1단계: PyTorch → ONNX

```bash
python scripts/export_onnx.py \
    --weights models/yolov8n.pt \
    --output  models/yolov8n.onnx \
    --imgsz   640
```

## 2단계: ONNX → TensorRT 엔진

**반드시 젯슨에서 실행하세요.**

```bash
python scripts/build_engine.py \
    --onnx      models/yolov8n.onnx \
    --output    models/yolov8n.engine \
    --fp16 \
    --workspace 4
```

빌드 시간: 약 5~15분 (모델 크기에 따라 상이)

## FP16 vs INT8

| 모드 | 속도 | 정확도 | 비고 |
|------|------|--------|------|
| FP32 | 기준 | 기준 | 기본값 |
| FP16 | ~2x | 거의 동일 | 젯슨 권장 |
| INT8 | ~4x | 소폭 하락 | 캘리브레이션 필요 |

젯슨 오린 나노에서는 **FP16을 기본 사용**하세요.

## 엔진 파일 공유 주의사항

- `.engine` 파일은 Git에 커밋하지 마세요 (`.gitignore`에 포함됨)
- 팀원 간 엔진 파일을 공유해도 **다른 기기에서는 동작하지 않습니다**
- 각 기기에서 ONNX → 엔진 변환을 직접 실행해야 합니다
- ONNX 파일은 기기 독립적이므로 팀 공유 스토리지를 활용하세요
