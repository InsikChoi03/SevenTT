#!/bin/bash
# setup.sh - 젯슨 환경 최초 세팅 스크립트
# 팀원이 git clone 후 한 번만 실행하세요.

set -e  # 오류 발생 시 즉시 중단

echo "=================================================="
echo " SevenTT Jetson 환경 세팅"
echo "=================================================="

# Python 버전 확인
PYTHON_VERSION=$(python3 --version 2>&1)
echo "[1/4] Python 버전: $PYTHON_VERSION"

# --system-site-packages: JetPack 내장 torch, cv2(CUDA 빌드) 접근 허용
# 이 옵션 없이 venv를 만들면 CUDA 지원이 빠진 패키지가 설치됨
echo "[2/4] 가상환경 생성 (--system-site-packages)..."
python3 -m venv venv --system-site-packages
echo "      완료: venv/"

# pip 업그레이드 및 패키지 설치
echo "[3/4] pip 패키지 설치..."
source venv/bin/activate
pip install --upgrade pip --quiet
pip install -r requirements.txt --quiet
echo "      완료: requirements.txt 설치"

# 필요 디렉토리 생성
echo "[4/4] 로컬 디렉토리 준비..."
mkdir -p models data/raw data/processed logs results checkpoints
echo "      완료: models/, data/, logs/, results/, checkpoints/"

echo ""
echo "=================================================="
echo " 세팅 완료"
echo ""
echo " 다음 명령어로 환경을 활성화하세요:"
echo "   source venv/bin/activate"
echo ""
echo " 모델 파일은 아래 문서를 참고해 다운로드하세요:"
echo "   models/README.md"
echo "=================================================="
