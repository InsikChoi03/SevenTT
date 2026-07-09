#!/usr/bin/env python3
"""Browser live view for the newest recognition_viz live.png.

The recognition visualizer writes a refreshed PNG under:
  data/mock_field_test/run_*/<timestamp>/live.png

This server follows the newest live.png automatically and exposes it as an
MJPEG stream, so the browser behaves like a real live camera/map view instead
of a static image tab.
"""

from __future__ import annotations

import argparse
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


def newest_live(root: Path) -> Path | None:
    files = list(root.glob("run_*/*/live.png")) + list(root.glob("*/live.png"))
    files = [p for p in files if p.is_file()]
    if not files:
        return None
    return max(files, key=lambda p: p.stat().st_mtime)


def make_handler(root: Path, fps: float):
    delay = 1.0 / max(0.2, fps)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def parse_request(self) -> bool:
            # Some browser/proxy paths occasionally prepend NUL bytes before
            # "GET", which BaseHTTPRequestHandler treats as an unknown method.
            self.raw_requestline = self.raw_requestline.lstrip(b"\x00")
            return super().parse_request()

        def log_message(self, fmt: str, *args) -> None:
            return

        def do_HEAD(self) -> None:
            self.send_response(200)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            path = parsed.path
            if path in ("/", "/index.html"):
                self._index()
            elif path == "/stream.mjpg":
                self._stream()
            elif path == "/frame.png":
                self._frame()
            elif path == "/latest":
                self._latest()
            else:
                self.send_response(404)
                self.end_headers()

        def _index(self) -> None:
            html = b"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Field Live Stream</title>
  <style>
    html, body { margin: 0; width: 100%; height: 100%; background: #101010; color: #eee;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; overflow: hidden; }
    header { height: 44px; box-sizing: border-box; display: flex; align-items: center; gap: 14px;
      padding: 8px 12px; background: #1d1d1d; border-bottom: 1px solid #333; font-size: 14px; }
    strong { white-space: nowrap; }
    #latest { color: #9fe7ff; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    main { width: 100%; height: calc(100% - 44px); display: grid; place-items: center; }
    main { background: #080808; }
    canvas { max-width: 100%; max-height: 100%; width: auto; height: auto; }
  </style>
</head>
<body>
  <header><strong>Field Live Stream</strong><span id="latest">connecting...</span></header>
  <main><canvas id="frame" width="1332" height="800"></canvas></main>
  <script>
    const canvas = document.getElementById('frame');
    const ctx = canvas.getContext('2d');
    let busy = false;

    async function refreshFrame() {
      if (busy) return;
      busy = true;
      try {
        const r = await fetch('/frame.png?t=' + Date.now(), {cache: 'no-store'});
        if (!r.ok) return;
        const blob = await r.blob();
        const bmp = await createImageBitmap(blob);
        if (canvas.width !== bmp.width || canvas.height !== bmp.height) {
          canvas.width = bmp.width;
          canvas.height = bmp.height;
        }
        ctx.drawImage(bmp, 0, 0);
        bmp.close();
      } catch (e) {
        // Keep the previous frame visible.
      } finally {
        busy = false;
      }
    }

    async function tick() {
      try {
        const r = await fetch('/latest?t=' + Date.now(), {cache: 'no-store'});
        document.getElementById('latest').textContent = await r.text();
      } catch (e) {
        document.getElementById('latest').textContent = 'latest path unavailable';
      }
    }
    tick();
    refreshFrame();
    setInterval(tick, 1000);
    setInterval(refreshFrame, 200);
  </script>
</body>
</html>
"""
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)

        def _latest(self) -> None:
            live = newest_live(root)
            if live is None:
                text = "no live.png yet"
            else:
                age = time.time() - live.stat().st_mtime
                text = f"{live.relative_to(root)}  ({age:.1f}s ago)"
            data = text.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _frame(self) -> None:
            live = newest_live(root)
            if live is None:
                self.send_response(404)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                return
            try:
                data = live.read_bytes()
            except OSError:
                self.send_response(503)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

            last_data = None
            last_content_type = "image/png"
            while True:
                live = newest_live(root)
                if live is not None:
                    try:
                        last_data = live.read_bytes()
                        last_content_type = "image/png"
                    except OSError:
                        pass
                if last_data:
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(f"Content-Type: {last_content_type}\r\n".encode("ascii"))
                        self.wfile.write(f"Content-Length: {len(last_data)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(last_data)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        return
                time.sleep(delay)

    return Handler


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/home/seventt/seventt/workspace/data/mock_field_test")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--fps", type=float, default=5.0)
    args = ap.parse_args()

    root = Path(args.root).resolve()
    srv = ThreadingHTTPServer((args.host, args.port), make_handler(root, args.fps))
    print(f"Field live stream: http://{args.host}:{args.port}/", flush=True)
    srv.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
