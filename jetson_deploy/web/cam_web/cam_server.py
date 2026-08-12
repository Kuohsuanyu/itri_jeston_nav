#!/usr/bin/env python3
"""RealSense 影像網頁檢視器 —— 彩色 + 深度,MJPEG 串流。

  D435 → realsense2_camera → 這支 → HTTP :8092 → 瀏覽器

用 MJPEG(multipart/x-mixed-replace)而不是 WebSocket:
瀏覽器原生就會把它當成一張會自己更新的 <img>,不需要任何前端解碼程式碼,
斷線也會自己重連。影像串流用這個最單純。

深度圖用 turbo 色階上色。深度是 uint16 毫米,直接顯示會是一片黑。
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

HTTP_PORT = 8092
JPEG_Q = 75           # 畫質 vs 頻寬,75 在區網上很夠
DEPTH_MAX_MM = 6000   # 深度上色的上限,D435 有效距離大約到 6 公尺

frames = {"color": None, "depth": None}        # 已編碼的 JPEG,給 HTTP 用
latest = {"color": None, "depth": None}        # 最新的原始幀,給編碼執行緒用
stats = {"color": [0, 0.0, 0.0], "depth": [0, 0.0, 0.0]}   # count, hz, window_start
lock = threading.Lock()


def encode(img):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
    return buf.tobytes() if ok else None


def encoder_loop():
    """把編碼從 ROS callback 移出來。

    rclpy 預設是單執行緒 executor,彩色和深度兩個 callback 排在同一條執行緒。
    原本在 callback 裡做通道轉換 + JPEG 編碼,兩者加起來把那條執行緒吃滿,
    ROS 就開始丟幀 —— 實測彩色被餓到 3.84 Hz(來源其實有 14 Hz)。
    現在 callback 只存參照(一個賦值,原子操作),真正的工作在這裡做。
    """
    seen = {"color": None, "depth": None}
    while True:
        did = False
        for key in ("color", "depth"):
            item = latest[key]
            if item is None or item is seen[key]:
                continue
            seen[key] = item
            did = True
            data, h, w, enc = item
            try:
                if key == "color":
                    a = np.frombuffer(data, dtype=np.uint8).reshape(h, w, -1)
                    img = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if enc == "rgb8" else a
                else:
                    d = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
                    # 0 代表沒量到。先當成最遠再上色,否則會有一堆刺眼的雜點
                    v = np.clip(d, 0, DEPTH_MAX_MM).astype(np.float32)
                    v[d == 0] = DEPTH_MAX_MM
                    u8 = (255.0 - v * (255.0 / DEPTH_MAX_MM)).astype(np.uint8)
                    img = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)
                    img[d == 0] = (40, 40, 40)      # 無效值畫成暗灰
                jpg = encode(img)
            except Exception:
                continue
            with lock:
                frames[key] = jpg
        if not did:
            time.sleep(0.005)


class CamBridge(Node):
    def __init__(self):
        super().__init__("cam_web_viewer")
        # 相機驅動搬進 Isaac 容器之後 namespace 從 camera 變成 camera0
        # (nvblox 的 NITROS 訂閱只有在同一個 component container 裡才會協商成功,
        #  所以相機不能留在 host)。兩個名稱都訂閱,誰在跑就收誰的。
        for ns in ("/camera0/camera", "/camera/camera"):
            self.create_subscription(Image, ns + "/color/image_raw",
                                     self.on_color, qos_profile_sensor_data)
            self.create_subscription(Image, ns + "/depth/image_rect_raw",
                                     self.on_depth, qos_profile_sensor_data)
        self.get_logger().info(f"cam_web_viewer 啟動 — HTTP :{HTTP_PORT}")

    def _tick(self, key):
        now = time.monotonic()
        s = stats[key]
        s[0] += 1
        if s[2] == 0.0:
            s[2] = now
        elif now - s[2] >= 1.0:
            s[1] = s[0] / (now - s[2])
            s[0] = 0
            s[2] = now

    # callback 只做兩件最便宜的事:存參照、計數。
    # 計數放在這裡才是「相機真正送進來的頻率」,不是編碼後的頻率。
    def on_color(self, msg):
        latest["color"] = (msg.data, msg.height, msg.width, msg.encoding)
        self._tick("color")

    def on_depth(self, msg):
        latest["depth"] = (msg.data, msg.height, msg.width, msg.encoding)
        self._tick("depth")


PAGE = """<!doctype html><meta charset="utf-8">
<title>RealSense D435 影像</title>
<style>
  :root{--bg:#0a0f16;--fg:#e6edf5;--dim:#7d8da3;--accent:#4ea1ff;
        --line:rgba(120,150,190,.20)}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);padding:18px;
       font:13px/1.6 ui-sans-serif,system-ui,"Noto Sans TC",sans-serif}
  h1{margin:0 0 3px;font-size:15px;letter-spacing:.06em}
  .sub{color:var(--dim);font-size:11.5px;margin-bottom:16px}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:16px}
  .card{background:rgba(16,22,30,.7);border:1px solid var(--line);
        border-radius:12px;padding:12px;overflow:hidden}
  h2{margin:0 0 9px;font-size:11px;letter-spacing:.12em;text-transform:uppercase;
     color:var(--accent);font-weight:600;display:flex;justify-content:space-between}
  h2 span{color:var(--dim);letter-spacing:0;text-transform:none;font-weight:400}
  img{width:100%;display:block;border-radius:8px;background:#11161d}
  .hint{color:var(--dim);font-size:11px;margin-top:14px;line-height:1.8}
</style>
<h1>RealSense D435</h1>
<div class="sub">640×480 @ 15 Hz — 彩色與深度串流</div>
<div class="grid">
  <div class="card"><h2>彩色 <span id="s-color">—</span></h2>
    <img src="/color.mjpg" alt="color"></div>
  <div class="card"><h2>深度 <span id="s-depth">—</span></h2>
    <img src="/depth.mjpg" alt="depth"></div>
</div>
<div class="hint">
  深度圖以 turbo 色階顯示,<b style="color:#c00">紅=近</b>、<b style="color:#00a">藍=遠</b>,
  上限 6 公尺;暗灰色是量不到的區域(太近、太遠、鏡面或吸光材質)。<br>
  白牆若大片暗灰是正常的 —— D435 靠投影 IR 點陣測距,但強光下點陣會被蓋掉。
</div>
<script>
setInterval(async () => {
  try {
    const d = await (await fetch('/stats.json')).json();
    for (const k of ['color','depth'])
      document.getElementById('s-'+k).textContent =
        d[k] > 0.1 ? d[k].toFixed(1) + ' Hz' : '無資料';
  } catch(e) {}
}, 1000);
</script>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/stats.json"):
            with lock:
                body = '{"color": %.2f, "depth": %.2f}' % (stats["color"][1],
                                                           stats["depth"][1])
            b = body.encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(b)
            return

        for key in ("color", "depth"):
            if self.path.startswith(f"/{key}.mjpg"):
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=f")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last = None
                try:
                    while True:
                        with lock:
                            jpg = frames[key]
                        # 同一張不重送,省頻寬
                        if jpg is not None and jpg is not last:
                            last = jpg
                            self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n"
                                             b"Content-Length: " +
                                             str(len(jpg)).encode() + b"\r\n\r\n")
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                        time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return

        b = PAGE.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)


def main():
    rclpy.init()
    node = CamBridge()
    threading.Thread(target=encoder_loop, daemon=True).start()
    threading.Thread(
        target=lambda: ThreadingHTTPServer(("0.0.0.0", HTTP_PORT),
                                           Handler).serve_forever(),
        daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
