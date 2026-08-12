#!/usr/bin/env python3
"""外參校準的視覺驗證 —— 把光達點投影到相機影像上疊圖。HTTP :8094

這是判斷外參對不對最直接的方法,比看重建出來的 mesh 可靠得多:
外參正確的話,光達點會**精準落在影像裡物體的邊緣上** —— 桌緣、門框、
椅腳、牆角。差 1 度,3 公尺外就偏 5 公分,在畫面上一眼看得出來。

mesh 看起來「大致有顏色」是沒有鑑別力的,錯 5 公分照樣看起來像有貼圖。

投影鏈:
    p_optical = B . M^-1 . C . p_body
      C = TF  base_link <- body                 (base_link_tf.sh 發的)
      M = base_link -> camera_link              (★ 這個就是要校的,由滑桿控制)
      B = TF  color_optical <- camera_link      (相機驅動發的,固定不動)

M 不從 TF 讀,直接由滑桿決定 —— 這樣滑桿的數值就是可以直接貼回
camera_extrinsic.sh 的值,不用再做任何換算。
"""
import io
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener

HTTP_PORT = 8094
CLOUD_TOPIC = "/cloud_registered_body"
CAM_NS = ("/camera/camera", "/camera0/camera")
COLOR_FRAME = "camera_color_optical_frame"

state = {
    "x": 0.0, "y": 0.0, "z": 0.0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "psize": 1,          # 點的半徑(像素)
    "maxr": 6.0,         # 只畫這麼近的點
    "mode": "overlay",   # overlay | edges | points
    "alpha": 1.0,        # 影像亮度(壓暗背景可以讓點更清楚)
    "loaded": False,     # 是否已從 TF 載入初始值
}
slock = threading.Lock()
frame_jpg = {"data": None, "n": 0, "hit": 0, "err": ""}


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr]])


def R_to_rpy(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        return (math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy),
                math.atan2(R[1, 0], R[0, 0]))
    return (math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_to_mat(t):
    M = np.eye(4)
    q = t.transform.rotation
    M[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    v = t.transform.translation
    M[:3, 3] = [v.x, v.y, v.z]
    return M


CMAP = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
LUT = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(-1, 1), CMAP).reshape(256, 3)


class Overlay(Node):
    def __init__(self):
        super().__init__("extrinsic_overlay")
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.color = None
        self.K = None
        self.cloud = None
        for ns in CAM_NS:
            self.create_subscription(Image, ns + "/color/image_raw",
                                     self.on_color, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, ns + "/color/camera_info",
                                     self.on_info, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, CLOUD_TOPIC,
                                 self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(f"外參疊圖檢視器 — HTTP :{HTTP_PORT}")

    def on_info(self, m):
        if self.K is None:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_color(self, m):
        if m.encoding == "rgb8":
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)[:, :, ::-1]
        elif m.encoding == "bgr8":
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        else:
            return
        self.color = a.copy()          # 存 BGR,cv2 直接用

    def on_cloud(self, m):
        n = m.width * m.height
        raw = np.frombuffer(m.data, np.uint8)[:n * m.point_step].reshape(n, m.point_step)
        xyz = raw[:, 0:12].copy().view(np.float32).reshape(n, 3).astype(np.float64)
        self.cloud = xyz[np.isfinite(xyz).all(axis=1)]

    def load_initial(self):
        """開機時從 TF 把目前的 base_link -> camera_link 讀進滑桿。"""
        try:
            M = tf_to_mat(self.buf.lookup_transform(
                "base_link", "camera_link", rclpy.time.Time()))
        except Exception:
            return False
        r, p, y = R_to_rpy(M[:3, :3])
        with slock:
            state.update(x=float(M[0, 3]), y=float(M[1, 3]), z=float(M[2, 3]),
                         roll=r, pitch=p, yaw=y, loaded=True)
        return True

    def render(self):
        if self.color is None or self.K is None or self.cloud is None:
            frame_jpg["err"] = "等待影像 / 內參 / 點雲"
            return
        try:
            B = tf_to_mat(self.buf.lookup_transform(
                COLOR_FRAME, "camera_link", rclpy.time.Time()))
            C = tf_to_mat(self.buf.lookup_transform(
                "base_link", "body", rclpy.time.Time()))
        except Exception as e:
            frame_jpg["err"] = "TF: " + str(e)[:60]
            return
        frame_jpg["err"] = ""

        with slock:
            s = dict(state)

        M = np.eye(4)
        M[:3, :3] = rpy_to_R(s["roll"], s["pitch"], s["yaw"])
        M[:3, 3] = [s["x"], s["y"], s["z"]]
        T = B @ np.linalg.inv(M) @ C

        img = self.color
        H, W = img.shape[:2]
        if s["mode"] == "points":
            out = np.zeros_like(img)
        else:
            out = (img.astype(np.float32) * s["alpha"]).clip(0, 255).astype(np.uint8)
            if s["mode"] == "edges":
                e = cv2.Canny(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 60, 160)
                out[e > 0] = (255, 255, 255)

        P = self.cloud @ T[:3, :3].T + T[:3, 3]
        z = P[:, 2]
        m = (z > 0.25) & (z < s["maxr"])
        n_in = 0
        if m.any():
            fx, fy, cx, cy = self.K
            zz = z[m]
            u = np.rint(fx * P[m, 0] / zz + cx).astype(np.int32)
            v = np.rint(fy * P[m, 1] / zz + cy).astype(np.int32)
            ok = (u >= 0) & (u < W) & (v >= 0) & (v < H)
            u, v, zz = u[ok], v[ok], zz[ok]
            n_in = len(u)
            if n_in:
                t = np.clip(zz / s["maxr"], 0, 1)
                col = LUT[(t * 255).astype(np.uint8)]
                r = int(s["psize"])
                for dy in range(-r, r + 1):
                    vy = np.clip(v + dy, 0, H - 1)
                    for dx in range(-r, r + 1):
                        out[vy, np.clip(u + dx, 0, W - 1)] = col

        # 十字準心:判斷偏移方向時有個固定參考點很有用
        cv2.line(out, (W // 2 - 12, H // 2), (W // 2 + 12, H // 2), (0, 255, 255), 1)
        cv2.line(out, (W // 2, H // 2 - 12), (W // 2, H // 2 + 12), (0, 255, 255), 1)

        ok, enc = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if ok:
            frame_jpg["data"] = enc.tobytes()
            frame_jpg["n"] += 1
            frame_jpg["hit"] = n_in


PAGE = r"""<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>外參校準疊圖</title>
<style>
:root{--bg:#080b11;--panel:rgba(16,22,32,.9);--line:rgba(120,160,220,.18);
      --fg:#dbe6f5;--dim:#7d8da3;--accent:#4ea1ff;--ok:#39d98a;--bad:#ff5f6d}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);display:flex;min-height:100vh;
     font:13px/1.55 ui-monospace,"Cascadia Mono",Consolas,monospace}
#view{flex:1;display:grid;place-items:center;padding:14px;min-width:0}
#view img{max-width:100%;max-height:94vh;border-radius:8px;
          border:1px solid var(--line);image-rendering:pixelated}
#side{width:330px;flex:none;background:var(--panel);border-left:1px solid var(--line);
      padding:16px;overflow-y:auto;max-height:100vh}
h1{margin:0 0 4px;font-size:12px;letter-spacing:.12em;text-transform:uppercase;
   color:var(--accent);font-weight:600}
.note{color:var(--dim);font-size:11px;line-height:1.7;margin:0 0 14px}
.g{margin:14px 0 0}
.g label{display:flex;justify-content:space-between;color:var(--dim);font-size:11px}
.g input[type=range]{width:100%;accent-color:var(--accent);height:3px;margin:3px 0 0}
.val{color:var(--fg);font-variant-numeric:tabular-nums}
.btns{display:flex;gap:6px;margin-top:12px}
button{flex:1;background:rgba(78,161,255,.10);color:var(--fg);border:1px solid var(--line);
       border-radius:7px;padding:7px 4px;cursor:pointer;font:inherit;font-size:11px}
button:hover{background:rgba(78,161,255,.24);border-color:var(--accent)}
button.on{background:var(--accent);color:#04101f;font-weight:600}
pre{background:#05080d;border:1px solid var(--line);border-radius:7px;padding:10px;
    font-size:11px;color:var(--ok);overflow-x:auto;margin:8px 0 0}
.row{display:flex;justify-content:space-between;padding:1px 0;font-size:11px}
.row span:first-child{color:var(--dim)}
#err{color:var(--bad);font-size:11px;min-height:16px;margin-top:6px}
hr{border:0;border-top:1px solid var(--line);margin:16px 0}
</style></head><body>
<div id="view"><img id="im" src="/stream.mjpg"></div>
<div id="side">
  <h1>外參校準疊圖</h1>
  <p class="note">外參正確時,光達點會<b>精準落在物體邊緣上</b> ——
  桌緣、門框、椅腳、牆角。對著 2~4 公尺處有明顯直線邊緣的東西看最準。<br><br>
  顏色 = 距離(藍近紅遠)。</p>

  <div class="btns">
    <button id="m-overlay" class="on">疊在影像上</button>
    <button id="m-edges">影像邊緣</button>
    <button id="m-points">只有點</button>
  </div>

  <hr>
  <div class="row"><span>投影到畫面的點</span><span class="val" id="s-hit">—</span></div>
  <div class="row"><span>畫面更新</span><span class="val" id="s-fps">—</span></div>
  <div id="err"></div>

  <hr>
  <div class="g"><label>點大小 <span class="val" id="v-psize">1</span></label>
    <input type="range" id="psize" min="0" max="3" step="1" value="1"></div>
  <div class="g"><label>最遠距離 <span class="val" id="v-maxr">6.0 m</span></label>
    <input type="range" id="maxr" min="1" max="12" step="0.5" value="6"></div>
  <div class="g"><label>影像亮度 <span class="val" id="v-alpha">1.00</span></label>
    <input type="range" id="alpha" min="0.15" max="1" step="0.05" value="1"></div>

  <hr>
  <p class="note" style="margin-bottom:0">微調外參。改完看點有沒有貼齊,
  貼齊了就把下面那段複製進 <b>~/slam2d/camera_extrinsic.sh</b>。</p>
  <div class="g"><label>X 前後 <span class="val" id="v-x">0</span></label>
    <input type="range" id="x" min="-0.5" max="0.5" step="0.002" value="0"></div>
  <div class="g"><label>Y 左右 <span class="val" id="v-y">0</span></label>
    <input type="range" id="y" min="-0.5" max="0.5" step="0.002" value="0"></div>
  <div class="g"><label>Z 上下 <span class="val" id="v-z">0</span></label>
    <input type="range" id="z" min="-0.5" max="0.5" step="0.002" value="0"></div>
  <div class="g"><label>roll <span class="val" id="v-roll">0</span></label>
    <input type="range" id="roll" min="-0.35" max="0.35" step="0.001" value="0"></div>
  <div class="g"><label>pitch <span class="val" id="v-pitch">0</span></label>
    <input type="range" id="pitch" min="-1.2" max="1.2" step="0.001" value="0"></div>
  <div class="g"><label>yaw <span class="val" id="v-yaw">0</span></label>
    <input type="range" id="yaw" min="-0.35" max="0.35" step="0.001" value="0"></div>

  <div class="btns">
    <button id="b-reload">從 TF 重讀</button>
    <button id="b-copy">複製設定</button>
  </div>
  <pre id="out"></pre>
</div>
<script>
const $ = i => document.getElementById(i);
const KEYS = ['x','y','z','roll','pitch','yaw'];
let sending = false;

function fmt(k, v){
  if (k === 'psize') return String(v);
  if (k === 'maxr')  return v.toFixed(1) + ' m';
  if (k === 'alpha') return v.toFixed(2);
  if (['roll','pitch','yaw'].includes(k))
    return v.toFixed(4) + '  (' + (v*180/Math.PI).toFixed(2) + '°)';
  return v.toFixed(4) + ' m';
}

function snippet(){
  const l = KEYS.map(k => {
    const n = k === 'roll' || k === 'pitch' || k === 'yaw' ? k.toUpperCase() : k.toUpperCase();
    return n + '=' + parseFloat($(k).value).toFixed(4);
  });
  return l.join('\n');
}

async function send(){
  if (sending) return;
  sending = true;
  const q = [...KEYS, 'psize', 'maxr', 'alpha']
      .map(k => k + '=' + $(k).value).join('&');
  try { await fetch('/set?' + q); } catch(e) {}
  sending = false;
}

[...KEYS, 'psize', 'maxr', 'alpha'].forEach(k => {
  $(k).addEventListener('input', () => {
    $('v-' + k).textContent = fmt(k, parseFloat($(k).value));
    $('out').textContent = snippet();
    send();
  });
});

function setMode(m){
  ['overlay','edges','points'].forEach(x => $('m-'+x).classList.toggle('on', x===m));
  fetch('/set?mode=' + m);
}
['overlay','edges','points'].forEach(m => $('m-'+m).onclick = () => setMode(m));

async function pull(){
  try{
    const s = await (await fetch('/state.json')).json();
    KEYS.concat(['psize','maxr','alpha']).forEach(k => {
      $(k).value = s[k];
      $('v-' + k).textContent = fmt(k, parseFloat(s[k]));
    });
    $('out').textContent = snippet();
  }catch(e){}
}
$('b-reload').onclick = async () => { await fetch('/reload'); pull(); };
$('b-copy').onclick = () => {
  navigator.clipboard.writeText(snippet());
  $('b-copy').textContent = '已複製';
  setTimeout(() => $('b-copy').textContent = '複製設定', 1200);
};

let lastN = 0;
setInterval(async () => {
  try{
    const s = await (await fetch('/stats.json')).json();
    $('s-hit').textContent = s.hit.toLocaleString();
    $('s-fps').textContent = (s.n - lastN) + ' /s';
    lastN = s.n;
    $('err').textContent = s.err || '';
  }catch(e){}
}, 1000);
pull();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        d = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(d)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(d)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=f")
            self.end_headers()
            try:
                while True:
                    d = frame_jpg["data"]
                    if d:
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: " + str(len(d)).encode()
                                         + b"\r\n\r\n" + d + b"\r\n")
                    time.sleep(0.1)
            except Exception:
                pass
            return

        if u.path == "/set":
            q = parse_qs(u.query)
            with slock:
                for k in ("x", "y", "z", "roll", "pitch", "yaw", "maxr", "alpha"):
                    if k in q:
                        state[k] = float(q[k][0])
                if "psize" in q:
                    state["psize"] = int(float(q["psize"][0]))
                if "mode" in q:
                    state["mode"] = q["mode"][0]
            self._send(200, b"{}", "application/json")
            return

        if u.path == "/reload":
            Handler.node.load_initial()
            self._send(200, b"{}", "application/json")
            return

        if u.path == "/state.json":
            with slock:
                self._send(200, json.dumps(state).encode(), "application/json")
            return

        if u.path == "/stats.json":
            self._send(200, json.dumps({"n": frame_jpg["n"], "hit": frame_jpg["hit"],
                                        "err": frame_jpg["err"]}).encode(),
                       "application/json")
            return

        self._send(200, PAGE, "text/html; charset=utf-8")


def main():
    rclpy.init()
    node = Overlay()
    Handler.node = node
    threading.Thread(
        target=lambda: ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler).serve_forever(),
        daemon=True).start()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # 等 TF 和第一批資料到齊,再把目前的外參讀進滑桿
    for _ in range(40):
        time.sleep(0.5)
        if node.load_initial():
            break

    # 算繪迴圈跟 ROS 分開 —— 在 callback 裡做 JPEG 編碼會讓單執行緒的
    # rclpy executor 掉幀(cam_server.py 就是這樣從 15.9Hz 掉到 3.8Hz 的)
    while True:
        t0 = time.time()
        try:
            node.render()
        except Exception as e:
            frame_jpg["err"] = "render: " + str(e)[:70]
        time.sleep(max(0.0, 0.1 - (time.time() - t0)))


if __name__ == "__main__":
    main()
