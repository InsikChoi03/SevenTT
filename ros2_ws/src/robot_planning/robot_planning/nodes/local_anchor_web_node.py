"""Standalone browser dashboard for the isolated local-anchor test."""
from __future__ import annotations

from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time
from urllib.parse import urlparse

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.msg import BaseCommand, Classification, DetectionArray, WorldModel
from sensor_msgs.msg import Image
from std_msgs.msg import String


_PAGE = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Local Anchor Test</title>
<style>
:root { color-scheme:dark; font-family:Arial,"Noto Sans KR",sans-serif; }
* { box-sizing:border-box; }
body { margin:0; background:#090d12; color:#e9f0f8; }
header {
  display:flex; align-items:center; gap:12px; padding:12px 18px;
  background:#111923; border-bottom:1px solid #2a3746; position:sticky; top:0;
  z-index:2;
}
h1 { margin:0; font-size:20px; }
.badge { padding:5px 9px; border-radius:999px; background:#273342; font-weight:700; }
.ok { background:#174b35; color:#8ff0ba; }
.warn { background:#5b4514; color:#ffe08a; }
.bad { background:#5a2027; color:#ff9da7; }
main { padding:12px; max-width:1500px; margin:auto; }
.grid { display:grid; grid-template-columns:minmax(0,1.45fr) minmax(360px,0.75fr); gap:12px; }
.card { background:#111923; border:1px solid #263445; border-radius:10px; overflow:hidden; }
.title { padding:9px 12px; background:#17212d; font-size:14px; font-weight:700; }
.content { padding:10px; }
#wide { display:block; width:100%; min-height:300px; background:#050608; object-fit:contain; }
#anchor { width:100%; aspect-ratio:1/1; display:block; background:#0b1118; }
.kv { display:grid; grid-template-columns:140px 1fr; gap:5px 10px; font-size:14px; }
.key { color:#8fa2b8; }
.value { font-family:ui-monospace,monospace; overflow-wrap:anywhere; }
.wide { grid-column:1/-1; }
table { width:100%; border-collapse:collapse; font-size:13px; }
th,td { padding:7px 8px; border-bottom:1px solid #253240; text-align:left; }
th { color:#8fa2b8; position:sticky; top:0; background:#111923; }
.scroll { max-height:280px; overflow:auto; }
.timeline { font-family:ui-monospace,monospace; font-size:12px; line-height:1.55; }
.muted { color:#8092a6; }
@media(max-width:900px) { .grid { grid-template-columns:1fr; } .wide { grid-column:auto; } }
</style>
</head>
<body>
<header>
  <h1>LOCAL ANCHOR TEST</h1>
  <span id="state" class="badge">WAIT</span>
  <span id="competition" class="badge">competition ?</span>
  <span id="motion" class="badge">motion ?</span>
  <span id="age" class="badge">camera ?</span>
</header>
<main>
<div class="grid">
  <section class="card">
    <div class="title">광각 실시간 영상 · YOLO 검출 박스 · 현재 판단</div>
    <img id="wide" src="/wide.mjpg" alt="wide camera stream">
  </section>
  <section class="card">
    <div class="title">로컬 앵커 맵 · 반경 40 cm</div>
    <canvas id="anchor" width="620" height="620"></canvas>
    <div class="content kv">
      <div class="key">현재 단계</div><div id="detail" class="value">-</div>
      <div class="key">회전</div><div id="turn" class="value">-</div>
      <div class="key">명령</div><div id="cmd" class="value">-</div>
      <div class="key">SigLIP</div><div id="siglip" class="value">-</div>
      <div class="key">inventory</div><div id="inventory" class="value">-</div>
    </div>
  </section>
  <section class="card wide">
    <div class="title">후보별 인식 → 누적 → 판정</div>
    <div class="scroll"><table>
      <thead><tr>
        <th>ID</th><th>위치</th><th>광각 인식</th><th>누적</th>
        <th>과일 판정</th><th>상태</th>
      </tr></thead>
      <tbody id="candidates"></tbody>
    </table></div>
  </section>
  <section class="card">
    <div class="title">현재 광각 raw 상대 관측</div>
    <div class="scroll"><table>
      <thead><tr><th>label</th><th>x</th><th>y</th><th>r</th><th>conf</th></tr></thead>
      <tbody id="raw"></tbody>
    </table></div>
  </section>
  <section class="card">
    <div class="title">FSM 판단 이력</div>
    <div id="timeline" class="content timeline"></div>
  </section>
</div>
</main>
<script>
const $ = id => document.getElementById(id);
function badge(el, text, cls) { el.textContent=text; el.className='badge '+cls; }
function statusClass(state) {
  if (state==='COMPLETE') return 'ok';
  if (state==='FAULT'||state==='ABORTED') return 'bad';
  return state==='IDLE' ? 'warn' : 'ok';
}
function drawMap(d) {
  const c=$('anchor'), x=c.getContext('2d'), w=c.width, h=c.height;
  const cx=w/2, cy=h/2, scale=(Math.min(w,h)*0.43)/0.40;
  x.fillStyle='#0b1118'; x.fillRect(0,0,w,h);
  for (const r of [0.1,0.2,0.3,0.4]) {
    x.beginPath(); x.arc(cx,cy,r*scale,0,Math.PI*2);
    x.strokeStyle=r===0.4?'#60758c':'#29394a'; x.lineWidth=r===0.4?3:1; x.stroke();
    x.fillStyle='#7890a8'; x.font='13px sans-serif';
    x.fillText(Math.round(r*100)+'cm',cx+5,cy-r*scale+15);
  }
  x.strokeStyle='#34475b'; x.lineWidth=1;
  x.beginPath(); x.moveTo(cx,cy-0.43*scale); x.lineTo(cx,cy+0.43*scale); x.stroke();
  x.beginPath(); x.moveTo(cx-0.43*scale,cy); x.lineTo(cx+0.43*scale,cy); x.stroke();
  const yaw=(d.relative_yaw_deg||0)*Math.PI/180;
  arrow(x,cx,cy,cx+0.11*scale*Math.cos(yaw),cy-0.11*scale*Math.sin(yaw),'#54b7ff',5);
  const target=(d.turn_target_deg||0)*Math.PI/180;
  arrow(x,cx,cy,cx+0.39*scale*Math.cos(target),cy-0.39*scale*Math.sin(target),'#ffe173',2);
  for (const o of d.relative_objects||[]) {
    const px=cx+o.anchor_x*scale, py=cy-o.anchor_y*scale;
    x.strokeStyle='#d6e3f0'; x.lineWidth=1;
    x.beginPath(); x.moveTo(px-5,py-5); x.lineTo(px+5,py+5);
    x.moveTo(px-5,py+5); x.lineTo(px+5,py-5); x.stroke();
  }
  const colors={
    UNINSPECTED:'#a5b2c0',FACING:'#ffe173',TARGET_FRUIT:'#48df8b',
    NON_TARGET_FRUIT:'#ff9a58',WAITING_CLASSIFICATION:'#ff6675',
    IGNORED_NON_FRUIT:'#687685',PICKED_DRY:'#74d8ff'
  };
  for (const q of d.candidates||[]) {
    const px=cx+q.x*scale, py=cy-q.y*scale;
    x.beginPath(); x.arc(px,py,q.id===d.active_candidate?15:11,0,Math.PI*2);
    x.fillStyle=colors[q.status]||'#b2bdc8'; x.fill();
    if (q.id===d.active_candidate) {
      x.strokeStyle='#fff'; x.lineWidth=3; x.stroke();
    }
    x.fillStyle='#081018'; x.font='bold 13px sans-serif';
    x.fillText(String(q.id),px-4,py+5);
  }
  x.fillStyle='#d8e5f2'; x.font='14px sans-serif';
  x.fillText('anchor +X',cx+8,cy-8);
  x.fillStyle='#54b7ff'; x.fillText('파랑=현재 heading',12,22);
  x.fillStyle='#ffe173'; x.fillText('노랑=회전 목표',12,42);
  x.fillStyle='#d6e3f0'; x.fillText('x=현재 광각 raw 관측',12,62);
}
function arrow(x,x1,y1,x2,y2,col,lw) {
  const a=Math.atan2(y2-y1,x2-x1); x.strokeStyle=col; x.fillStyle=col; x.lineWidth=lw;
  x.beginPath(); x.moveTo(x1,y1); x.lineTo(x2,y2); x.stroke();
  x.beginPath(); x.moveTo(x2,y2); x.lineTo(x2-13*Math.cos(a-.45),y2-13*Math.sin(a-.45));
  x.lineTo(x2-13*Math.cos(a+.45),y2-13*Math.sin(a+.45)); x.closePath(); x.fill();
}
function row(values) { return '<tr>'+values.map(v=>'<td>'+v+'</td>').join('')+'</tr>'; }
async function update() {
  try {
    const res=await fetch('/api/status',{cache:'no-store'}), d=await res.json();
    badge($('state'),d.state||'?',statusClass(d.state));
    badge($('competition'),'competition '+(d.competition_state||'?'),
      d.competition_state==='RUNNING'?'ok':'warn');
    badge($('motion'),d.drive_enabled&&d.armed?'MOTION ARMED':'motion safe',
      d.drive_enabled&&d.armed?'warn':'ok');
    const hasImage=d.image_age_sec!==null&&d.image_age_sec!==undefined;
    const age=hasImage?Number(d.image_age_sec):NaN;
    badge($('age'),hasImage?'camera '+age.toFixed(1)+'s':'camera no frame',
      hasImage&&age<1?'ok':'bad');
    $('detail').textContent=(d.state||'?')+' · '+(d.detail||'-');
    const headingSource=d.heading_source==='imu_delta'?'IMU':'POSE';
    $('turn').textContent='['+headingSource+'] yaw '+
      Number(d.relative_yaw_deg||0).toFixed(1)+'° → '+
      Number(d.turn_target_deg||0).toFixed(1)+'° · pulses '+(d.turn_pulses||0);
    const m=d.base_command||{};
    $('cmd').textContent='vx '+Number(m.vx||0).toFixed(3)+' · vy '+
      Number(m.vy||0).toFixed(3)+' · ω '+Number(m.omega||0).toFixed(3);
    const s=d.siglip||{};
    $('siglip').textContent=s.label ? s.label+' conf '+Number(s.confidence||0).toFixed(2)+
      ' face '+(s.face_visible?'YES':'NO') : '-';
    $('inventory').textContent='scan '+Number(d.scan_completed_deg||0).toFixed(0)+'° · '+
      (d.inventory_count_ok?'OK ✓':'CHECK ⚠')+' · '+
      (d.inventory_count||0)+' / expected '+(d.expected_candidates||[]).join('~')+
      ' · route '+(d.route||[]).join(' → ');
    $('candidates').innerHTML=(d.candidates||[]).map(q=>row([
      q.id,(q.radius*100).toFixed(1)+'cm / '+q.bearing_deg.toFixed(1)+'°',
      q.label+' '+Number(q.confidence||0).toFixed(2),q.hits+' wide / '+
      (q.classification_hits||0)+' class',
      q.fruit_label||'-',q.status
    ])).join('') || row(['-','-','-','-','-','후보 없음']);
    $('raw').innerHTML=(d.relative_objects||[]).map(o=>row([
      o.label,Number(o.x).toFixed(3),Number(o.y).toFixed(3),
      (Number(o.radius)*100).toFixed(1)+'cm',Number(o.confidence).toFixed(2)
    ])).join('') || row(['-','-','-','-','관측 없음']);
    $('timeline').innerHTML=(d.timeline||[]).slice().reverse().map(e=>
      '<div><span class="muted">+'+Number(e.t).toFixed(1)+'s</span> '+
      e.state+' · '+e.detail+'</div>').join('') || '<span class="muted">이력 없음</span>';
    drawMap(d);
  } catch(e) { badge($('state'),'WEB LOST','bad'); }
}
setInterval(update,250); update();
</script>
</body></html>
"""


_BOX_COLORS = {
    "fruit_photo_cube": (220, 90, 200),
    "cube": (100, 220, 120),
    "apple": (40, 40, 230),
    "orange": (0, 145, 255),
    "banana": (60, 225, 245),
    "pineapple": (60, 195, 225),
}


class LocalAnchorWebNode(Node):
    """Serve a read-only dashboard on a port independent from the main 8080 UI."""

    def __init__(self) -> None:
        super().__init__("local_anchor_web_node")
        self.declare_parameter("web_port", 8082)
        self.declare_parameter("web_fps", 8.0)
        self.port = int(self.get_parameter("web_port").value)
        self.web_fps = max(1.0, min(15.0, float(self.get_parameter("web_fps").value)))
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.top_image: Image | None = None
        self.top_detections = []
        self.body_detection_count = 0
        self.relative_objects = []
        self.status: dict = {}
        self.siglip: dict = {}
        self.base_command = {"vx": 0.0, "vy": 0.0, "omega": 0.0}
        self.competition_state = "UNKNOWN"
        self.latest_jpeg: bytes | None = None
        self.image_time = 0.0
        self.detection_time = 0.0
        self.status_time = 0.0
        self.start_time = time.monotonic()
        self.timeline: deque[dict] = deque(maxlen=30)
        self._last_event_key = ("", "")

        self.create_subscription(
            Image,
            "/camera_top/image_raw",
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DetectionArray, "/camera_top/detections", self._on_top_detections, 10
        )
        self.create_subscription(
            DetectionArray, "/camera_body/detections", self._on_body_detections, 10
        )
        self.create_subscription(
            WorldModel,
            "/world_model/wide_relative_objects",
            self._on_relative_objects,
            10,
        )
        self.create_subscription(String, "/local_anchor_test/status", self._on_status, 10)
        self.create_subscription(
            Classification, "/classification/siglip", self._on_siglip, 10
        )
        self.create_subscription(BaseCommand, "/base_command", self._on_base_command, 10)
        self.create_subscription(String, "/competition/state", self._on_competition, 10)
        self.timer = self.create_timer(1.0 / self.web_fps, self._render_tick)

        handler = self._handler_class()
        self.httpd = ThreadingHTTPServer(("0.0.0.0", self.port), handler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(
            f"local-anchor dashboard: http://10.42.0.1:{self.port}/ "
            "(independent from main port 8080)"
        )

    def _on_image(self, msg: Image) -> None:
        with self.lock:
            self.top_image = msg
            self.image_time = time.monotonic()

    def _on_top_detections(self, msg: DetectionArray) -> None:
        with self.lock:
            self.top_detections = list(msg.detections)
            self.detection_time = time.monotonic()

    def _on_body_detections(self, msg: DetectionArray) -> None:
        with self.lock:
            self.body_detection_count = len(msg.detections)

    def _on_relative_objects(self, msg: WorldModel) -> None:
        objects = []
        for obj in msg.objects:
            x, y = float(obj.x), float(obj.y)
            objects.append(
                {
                    "label": str(obj.class_label),
                    "x": x,
                    "y": y,
                    "radius": math.hypot(x, y),
                    "confidence": float(obj.confidence),
                }
            )
        with self.lock:
            self.relative_objects = objects

    def _on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(status, dict):
            return
        state = str(status.get("state", "?"))
        detail = str(status.get("detail", ""))
        key = (state, detail)
        with self.lock:
            self.status = status
            self.status_time = time.monotonic()
            if key != self._last_event_key:
                self.timeline.append(
                    {
                        "t": time.monotonic() - self.start_time,
                        "state": state,
                        "detail": detail,
                    }
                )
                self._last_event_key = key

    def _on_siglip(self, msg: Classification) -> None:
        with self.lock:
            self.siglip = {
                "label": str(msg.label),
                "confidence": float(msg.confidence),
                "face_visible": bool(msg.image_face_visible),
                "is_target": bool(msg.is_target),
            }

    def _on_base_command(self, msg: BaseCommand) -> None:
        with self.lock:
            self.base_command = {
                "vx": float(msg.vx),
                "vy": float(msg.vy),
                "omega": float(msg.omega),
            }

    def _on_competition(self, msg: String) -> None:
        with self.lock:
            self.competition_state = str(msg.data).strip().upper()

    def _snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            payload = dict(self.status)
            raw_objects = [dict(item) for item in self.relative_objects]
            yaw = math.radians(float(payload.get("relative_yaw_deg", 0.0)))
            c, s = math.cos(yaw), math.sin(yaw)
            for item in raw_objects:
                x, y = item["x"], item["y"]
                item["anchor_x"] = c * x - s * y
                item["anchor_y"] = s * x + c * y
            payload.update(
                {
                    "relative_objects": raw_objects,
                    "wide_detection_count": len(self.top_detections),
                    "body_detection_count": self.body_detection_count,
                    "siglip": dict(self.siglip),
                    "base_command": dict(self.base_command),
                    "competition_state": self.competition_state,
                    "timeline": list(self.timeline),
                    "image_age_sec": (
                        now - self.image_time if self.image_time > 0.0 else None
                    ),
                    "detection_age_sec": (
                        now - self.detection_time if self.detection_time > 0.0 else None
                    ),
                    "status_age_sec": (
                        now - self.status_time if self.status_time > 0.0 else None
                    ),
                }
            )
        return payload

    def _render_tick(self) -> None:
        with self.lock:
            image_msg = self.top_image
            detections = list(self.top_detections)
            status = dict(self.status)
            command = dict(self.base_command)
        if image_msg is None:
            frame = self._no_frame_image(status)
        else:
            try:
                frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8").copy()
            except Exception:  # noqa: BLE001 - a later camera frame can recover
                frame = self._no_frame_image(status)
        h, w = frame.shape[:2]
        for detection in detections:
            cx, cy = float(detection.x_center), float(detection.y_center)
            bw, bh = float(detection.width), float(detection.height)
            x1, y1 = int(cx - bw / 2.0), int(cy - bh / 2.0)
            x2, y2 = int(cx + bw / 2.0), int(cy + bh / 2.0)
            label = str(detection.label)
            color = _BOX_COLORS.get(label, (180, 180, 180))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3, cv2.LINE_AA)
            text = f"{label} {float(detection.confidence):.2f}"
            cv2.rectangle(frame, (x1, max(0, y1 - 28)), (min(w - 1, x1 + 240), y1), color, -1)
            cv2.putText(
                frame,
                text,
                (x1 + 4, max(18, y1 - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (10, 15, 20),
                2,
                cv2.LINE_AA,
            )
        self._draw_overlay(frame, status, command, len(detections))
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if ok:
            with self.lock:
                self.latest_jpeg = encoded.tobytes()

    @staticmethod
    def _no_frame_image(status: dict):
        import numpy as np

        frame = np.full((720, 1280, 3), 18, dtype=np.uint8)
        cv2.putText(
            frame,
            "WAITING FOR /camera_top/image_raw",
            (250, 360),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.1,
            (80, 170, 255),
            2,
            cv2.LINE_AA,
        )
        return frame

    @staticmethod
    def _draw_overlay(frame, status: dict, command: dict, detection_count: int) -> None:
        h, w = frame.shape[:2]
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 58), (5, 8, 12), -1)
        cv2.rectangle(overlay, (0, h - 74), (w, h), (5, 8, 12), -1)
        cv2.addWeighted(overlay, 0.78, frame, 0.22, 0.0, frame)
        state = str(status.get("state", "NO STATUS"))
        detail = str(status.get("detail", "waiting for local-anchor status"))
        cv2.putText(
            frame,
            f"WIDE | YOLO dets={detection_count} | FSM={state}",
            (14, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (245, 245, 245),
            2,
            cv2.LINE_AA,
        )
        yaw = float(status.get("relative_yaw_deg", 0.0))
        target = float(status.get("turn_target_deg", 0.0))
        cv2.putText(
            frame,
            f"decision: {detail}",
            (14, h - 43),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (80, 225, 255),
            2,
            cv2.LINE_AA,
        )
        cmd_text = (
            f"yaw {yaw:+.1f} -> {target:+.1f} deg | "
            f"cmd vx={command.get('vx', 0.0):+.3f} "
            f"vy={command.get('vy', 0.0):+.3f} "
            f"omega={command.get('omega', 0.0):+.3f}"
        )
        cv2.putText(
            frame,
            cmd_text,
            (14, h - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )

    def _handler_class(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, _format, *_args):
                return

            def _headers(self, content_type: str, length: int | None = None) -> None:
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                self.send_header("Access-Control-Allow-Origin", "*")
                if length is not None:
                    self.send_header("Content-Length", str(length))
                self.end_headers()

            def do_GET(self):
                path = urlparse(self.path).path
                if path == "/":
                    data = _PAGE.encode("utf-8")
                    self._headers("text/html; charset=utf-8", len(data))
                    self.wfile.write(data)
                    return
                if path == "/api/status":
                    data = json.dumps(
                        node._snapshot(), ensure_ascii=False, allow_nan=False
                    ).encode("utf-8")
                    self._headers("application/json; charset=utf-8", len(data))
                    self.wfile.write(data)
                    return
                if path == "/wide.mjpg":
                    self._stream_wide()
                    return
                if path == "/favicon.ico":
                    self.send_error(404)
                    return
                self.send_error(404)

            def _stream_wide(self):
                self._headers("multipart/x-mixed-replace; boundary=frame")
                try:
                    while True:
                        with node.lock:
                            frame = node.latest_jpeg
                        if frame is not None:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(
                                f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                            )
                            self.wfile.write(frame)
                            self.wfile.write(b"\r\n")
                        time.sleep(1.0 / node.web_fps)
                except (BrokenPipeError, ConnectionResetError):
                    return

        return Handler

    def destroy_node(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalAnchorWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
