#!/usr/bin/env python3
"""labelImg + PyQt5 5.15 박스그리기 크래시 수정.

원인: libs/canvas.py 가 drawLine/drawRect 에 float 좌표를 넘김 (PyQt5 5.15는 int 요구).
→ 해당 좌표를 int()로 감싼다. 원본은 canvas.py.bak 으로 백업.
복구: pip install --force-reinstall labelImg

  python3 scripts/fix_labelimg.py
"""
from __future__ import annotations
import importlib.util, shutil, sys

REPL = [
    ("p.drawRect(left_top.x(), left_top.y(), rect_width, rect_height)",
     "p.drawRect(int(left_top.x()), int(left_top.y()), int(rect_width), int(rect_height))"),
    ("p.drawLine(self.prev_point.x(), 0, self.prev_point.x(), self.pixmap.height())",
     "p.drawLine(int(self.prev_point.x()), 0, int(self.prev_point.x()), int(self.pixmap.height()))"),
    ("p.drawLine(0, self.prev_point.y(), self.pixmap.width(), self.prev_point.y())",
     "p.drawLine(0, int(self.prev_point.y()), int(self.pixmap.width()), int(self.prev_point.y()))"),
]


def main():
    spec = importlib.util.find_spec("libs.canvas")
    if spec is None or not spec.origin:
        print("libs.canvas 못 찾음 — labelImg 설치 확인"); return 1
    path = spec.origin
    src = open(path, encoding="utf-8").read()
    shutil.copy(path, path + ".bak")
    n = 0
    for old, new in REPL:
        if old in src:
            src = src.replace(old, new); n += 1
        elif new in src:
            n += 1  # 이미 패치됨
    open(path, "w", encoding="utf-8").write(src)
    print(f"패치 적용: {path}\n  변경/확인 {n}/{len(REPL)}곳. 백업: {path}.bak")
    print("이제 labelImg에서 w(박스 추가) 크래시 안 남. 다시 실행하세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
