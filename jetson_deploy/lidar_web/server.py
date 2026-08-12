#!/usr/bin/env python3
"""
lidar_web_viewer — 把 FAST-LIO 的點雲和位姿推到瀏覽器,並用 D435 的彩色影像上色。

訂閱:
  /cloud_registered                      sensor_msgs/PointCloud2   世界座標的配準點雲
  /Odometry                              nav_msgs/Odometry         即時位姿
  <ns>/color/image_raw                   sensor_msgs/Image         上色來源
  <ns>/color/camera_info                 sensor_msgs/CameraInfo    內參
  <ns>/aligned_depth_to_color/image_raw  sensor_msgs/Image         遮蔽判斷用(可有可無)

對外:
  HTTP  :8080   靜態網頁
  WS    :8081   binary = 新增的點 (float32 x,y,z,intensity,rgb),text = JSON 狀態

點雲以體素去重,只推「新出現的體素」,所以頻寬跟環境大小有關,
而不是跟掃描頻率有關 —— 站著不動時幾乎不佔頻寬。

--- 為什麼是這個架構而不是 nvblox ---
幾何全部來自光達(40 m、360deg、公分級),相機只負責「把顏色刷上去」。
D435 的深度只用來判斷遮蔽,不參與建圖 —— 它的有效距離 4 m,
拿來當幾何來源在這個場景是降級。
"""

import asyncio
import json
import os
import threading
import time
from collections import deque

import numpy as np
import rclpy
import websockets
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time as RclTime
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener

from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

WEB_DIR = os.path.dirname(os.path.abspath(__file__))
HTTP_PORT = 8080
WS_PORT = 8081

VOXEL = 0.02          # 體素邊長 (m)
                      # 原本 0.04 —— 4cm 去重會把桌椅、箱子的邊緣抹平,
                      # 看起來就是一坨球。2cm 讓表面點數約 4 倍,
                      # 才分得出物體輪廓。室內場景約 2M 點,還在 MAX_POINTS 之內。
MAX_POINTS = 4_000_000
CHUNK = 20_000        # 補送歷史點雲時的分塊大小(點數)
                      # 原本 150_000,每塊就是 2.4 MB 的單一 WebSocket frame,
                      # 在 WiFi 上要送一兩秒,期間 /Odometry 全部排在後面,
                      # 解除阻塞後十幾筆一次湧入 —— 這是軌跡出問題的觸發條件。
                      # 縮到 20_000 讓 odom 有機會插隊,資料流順很多。
STRIDE = 5            # 每點 5 個 float32:x, y, z, intensity, rgb
COL_RGB = 4           # rgb 在一筆點裡的欄位索引

# --- 動態物體處理 ---------------------------------------------------------
# 原本的行為是「掃到一次就永久保留」,結果走過的人、揮動的手、推過去的車
# 全都在圖上拖出一條永遠不會消失的殘影,場景越久越髒。
#
# 兩道機制配合:
#   CONFIRM_HITS  體素要被看到幾幀才准進圖。人走過某個位置停留遠不到 0.3 秒,
#                 根本累積不到門檻,所以連進都進不來。
#   DECAY_SEC     進圖之後若這麼久沒再被觀測到就移除。Mid-360 是 360deg 視野,
#                 牆面地面每一幀都會被重新觀測所以會留著;人離開後就會淡出。
#
# 代價要講清楚:機器人開離某個區域之後,那區也會在 DECAY_SEC 後淡出。
# 需要「永久保留」的地圖請看 2D occupancy map(:8090),那才是持久化的產物。
CONFIRM_HITS = 3
DECAY_SEC = 25.0
SWEEP_SEC = 5.0       # 多久掃一次過期體素
PENDING_TTL = 2.0     # 還沒達到門檻的候選體素,這麼久沒再出現就丟掉

# 衰減只作用在「目前感測得到」的範圍內。
#
# 只看時間會犯一個嚴重錯誤:走過的區域一離開視野就沒有新觀測,
# 25 秒後整片被刪掉 —— 大範圍建圖時等於邊走邊擦掉自己的成果。
#
# 「東西不見了」這個結論需要有新觀測來否定它。離開視野的地方沒有任何資訊,
# 不能當作消失。所以只有離感測器 DECAY_RADIUS 以內的體素才納入衰減判斷,
# 範圍外一律保留。這樣走動的人(一定在近處)照樣會被清掉,
# 而已經建好的遠處環境不會被動到。
DECAY_RADIUS = 8.0

# --- 上色 -----------------------------------------------------------------
# 相機的 namespace。realsense2_camera 跑在 host 上時是 /camera/camera;
# 之前搬進 Isaac 容器時變成 /camera0/camera。兩個都訂閱,誰在跑就吃誰,
# 反正同一時間只會有一個。
CAM_NS = ("/camera/camera", "/camera0/camera")
COLOR_FRAME = os.environ.get("COLOR_FRAME", "camera_color_optical_frame")

COLOR_MIN_RANGE = 0.35   # 離相機這麼近的點不上色(幾乎都是自身結構)
COLOR_MAX_RANGE = 5.0    # 超過這個距離,外參的角度誤差會被距離放大成明顯錯位,
                         # 上出來的顏色是錯的。寧可留白等靠近再補。
OCCLUSION_TOL = 0.25     # 點到相機的距離 vs 對齊深度圖讀到的距離,
                         # 差超過這個就判定「這個點其實被別的東西擋住了」,不上色。
                         # 沒有這道檢查的話,牆的顏色會被刷到牆後面的物體上。
IMG_BUF = 12             # 保留幾張最近的彩色影像
IMG_MAX_AGE = 0.6        # 影像跟點雲的時間差超過這麼多就不用(單位秒)
RECOLOR_REBUILD = 3000   # 累積這麼多個「舊點拿到顏色」就整份重送一次,
                         # 否則瀏覽器那邊的舊點永遠是灰的。


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]],
        dtype=np.float64)


class Hub:
    """管理瀏覽器連線,負責廣播。"""

    def __init__(self):
        self.clients = set()
        self.loop = None
        self.lock = threading.Lock()
        self.buf = np.zeros((MAX_POINTS, STRIDE), dtype=np.float32)
        self.count = 0
        self.colored = 0        # 已經有顏色的點數,只做顯示用,每次 sweep 重算
        self.recolored = 0      # 上次重送之後有多少舊點拿到顏色

        # 體素簿記全部放在 Hub,跟 buf 共用同一把鎖,
        # 免得 ROS 執行緒和衰減掃描各自持有一份狀態而對不起來。
        self.live = {}                                        # voxel key -> slot
        self.pending = {}                                     # voxel key -> [hits, last_t]
        self.key_of = np.zeros(MAX_POINTS, dtype=np.int64)    # slot -> voxel key
        self.last_of = np.zeros(MAX_POINTS, dtype=np.float64) # slot -> 最後觀測時間

    # --- 由 ROS 執行緒呼叫 ---

    def ingest(self, keys, pts, now):
        """吃一幀的去重體素,回傳「這一幀新確認、要廣播出去」的點。

        keys / pts 是同長度的 array,keys 已經是 unique。
        """
        out = []
        with self.lock:
            live, pending = self.live, self.pending
            for i, k in enumerate(keys.tolist()):
                slot = live.get(k)
                if slot is not None:
                    self.last_of[slot] = now        # 老朋友,續命
                    # 之前沒顏色、這次相機看到了 -> 補上。
                    # 先到先贏:能通過 COLOR_MAX_RANGE 的顏色品質都夠好,
                    # 一直改反而會讓畫面隨視角閃動。
                    if self.buf[slot, COL_RGB] < 0.0 and pts[i, COL_RGB] >= 0.0:
                        self.buf[slot, COL_RGB] = pts[i, COL_RGB]
                        self.recolored += 1
                    continue

                e = pending.get(k)
                if e is None:
                    pending[k] = [1, now]
                    continue
                e[0] += 1
                e[1] = now
                if e[0] < CONFIRM_HITS:
                    continue

                # 通過確認門檻,正式進圖
                if self.count >= MAX_POINTS:
                    break
                s = self.count
                self.buf[s] = pts[i]
                self.key_of[s] = k
                self.last_of[s] = now
                live[k] = s
                self.count = s + 1
                del pending[k]
                out.append(pts[i])

        return np.asarray(out, dtype=np.float32) if out else None

    def sweep(self, now, sensor):
        """移除「感測範圍內、且太久沒被觀測到」的體素。

        sensor 是目前感測器位置 (x, y, z)。範圍外的體素一律保留 ——
        那些地方沒有新觀測,不能推論它們消失了。
        回傳 True 表示圖變了、需要全量重送。
        """
        with self.lock:
            # 候選體素也要清,否則 pending 會無限長大
            stale = [k for k, e in self.pending.items() if now - e[1] > PENDING_TTL]
            for k in stale:
                del self.pending[k]

            n = self.count
            if n == 0:
                self.colored = 0
                return False

            self.colored = int((self.buf[:n, COL_RGB] >= 0.0).sum())

            expired = (now - self.last_of[:n]) > DECAY_SEC
            d = self.buf[:n, 0:3] - np.asarray(sensor, dtype=np.float32)
            near = np.einsum("ij,ij->i", d, d) <= (DECAY_RADIUS * DECAY_RADIUS)
            # 只有「過期」而且「還在感測範圍內」才刪
            keep = ~(expired & near)
            n_keep = int(keep.sum())
            if n_keep == n:
                return False

            # 壓實:buf / key_of / last_of 一起搬,再重建 key -> slot 對照
            self.buf[:n_keep] = self.buf[:n][keep]
            self.key_of[:n_keep] = self.key_of[:n][keep]
            self.last_of[:n_keep] = self.last_of[:n][keep]
            self.count = n_keep
            self.colored = int((self.buf[:n_keep, COL_RGB] >= 0.0).sum())
            self.live = {int(k): i for i, k in enumerate(self.key_of[:n_keep].tolist())}
            return True

    def take_recolored(self):
        with self.lock:
            v = self.recolored
            self.recolored = 0
            return v

    def push(self, payload):
        if self.loop is None or not self.clients:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self.loop)

    def snapshot(self):
        with self.lock:
            return self.buf[:self.count].copy()

    def reset(self):
        with self.lock:
            self.count = 0
            self.colored = 0
            self.recolored = 0
            self.live.clear()
            self.pending.clear()

    async def _broadcast(self, payload):
        dead = []
        for ws in list(self.clients):
            try:
                await ws.send(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)


class CameraFeed:
    """保存最近幾張彩色影像(附帶對齊過的深度)和內參。

    為什麼要留一整串而不是只留最新一張:
    相機用「現在時間」蓋時戳,但 FAST-LIO 的 TF 會落後 0.15~0.3 秒。
    所以查最新影像時戳的 TF 幾乎一定失敗。
    (nvblox 就是死在這件事上 —— 它的 maximum_input_queue_length 預設 3,
     等於只有 0.2 秒的緩衝,深度幀還沒等到 TF 就被丟光,map 永遠是空的。)
    留一串影像,由新到舊挑第一個「TF 查得到」的來用,就繞過這個時序落差。
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.frames = deque(maxlen=IMG_BUF)   # 由舊到新
        self.K = None                          # (fx, fy, cx, cy)
        self.last_depth = None                 # (stamp_sec, (H,W) float32 公尺)

    def set_info(self, msg):
        with self.lock:
            if self.K is None:
                self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def add_depth(self, msg):
        if msg.encoding not in ("16UC1", "mono16"):
            return
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        with self.lock:
            self.last_depth = (stamp_sec(msg.header.stamp),
                               d.astype(np.float32) * 0.001)

    def add_color(self, msg):
        if msg.encoding == "rgb8":
            c = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding == "bgr8":
            c = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)[:, :, ::-1]
        else:
            return
        t = stamp_sec(msg.header.stamp)
        with self.lock:
            depth = None
            if self.last_depth is not None and abs(self.last_depth[0] - t) < 0.05:
                depth = self.last_depth[1]
            # 存 rclpy 的 Time,不是原始的 msg —— tf2 的 lookup_transform
            # 只吃 rclpy.time.Time,丟 builtin_interfaces 的 Time 會炸
            self.frames.append({"t": t, "stamp": RclTime.from_msg(msg.header.stamp),
                                "color": c, "depth": depth})

    def recent(self, t_cloud):
        """由新到舊回傳夠接近 t_cloud 的影像。"""
        with self.lock:
            if self.K is None:
                return []
            return [f for f in reversed(self.frames)
                    if abs(f["t"] - t_cloud) < IMG_MAX_AGE]


def stamp_sec(s):
    return s.sec + s.nanosec * 1e-9


class Bridge(Node):
    def __init__(self, hub):
        super().__init__("lidar_web_viewer")
        self.hub = hub
        self.int_off = None      # intensity 在 point_step 內的位移,首幀自動偵測
        self.rebuilding = False  # 衰減重送進行中,期間暫停推即時點
        self.sensor = (0.0, 0.0, 0.0)   # 目前感測器位置,衰減的距離閘門要用

        self.feed = CameraFeed()
        self.tfbuf = Buffer()
        self.tflisten = TransformListener(self.tfbuf, self)
        self.world_frame = None     # 第一次查成功的世界座標系名稱,之後沿用
        self.color_warned = False

        self.create_subscription(
            PointCloud2, "/cloud_registered", self.on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/Odometry", self.on_odom, 10)

        # 兩個 namespace 都訂,誰在跑就吃誰
        for ns in CAM_NS:
            self.create_subscription(Image, ns + "/color/image_raw",
                                     self.feed.add_color, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, ns + "/color/camera_info",
                                     self.feed.set_info, qos_profile_sensor_data)
            self.create_subscription(Image, ns + "/aligned_depth_to_color/image_raw",
                                     self.feed.add_depth, qos_profile_sensor_data)

        self.create_timer(SWEEP_SEC, self.on_sweep)
        self.get_logger().info(
            f"lidar_web_viewer 啟動 — HTTP :{HTTP_PORT}  WS :{WS_PORT}  voxel={VOXEL}m "
            f"confirm={CONFIRM_HITS}幀 decay={DECAY_SEC}s 上色={COLOR_FRAME}"
        )

    # --- 上色 ---

    def lookup(self, target, source, stamp):
        try:
            return self.tfbuf.lookup_transform(target, source, stamp)
        except Exception:
            return None

    def colorize(self, xyz, cloud_frame, t_cloud):
        """回傳長度 N 的 float32:打包好的 rgb,查不到顏色的位置是 -1。

        rgb 打包成 r*65536 + g*256 + b。float32 的尾數有 24 bit,
        剛好裝得下 0~16777215,不會失真;瀏覽器端用整數運算拆回來。
        這樣一個點只多 4 bytes,不用擴成三個 float。
        """
        n = len(xyz)
        out = np.full(n, -1.0, dtype=np.float32)

        frames = self.feed.recent(t_cloud)
        if not frames:
            return out

        # 世界座標系的名字:優先用點雲自己宣告的,查不到再試常見的幾個。
        # FAST-LIO 有時發 camera_init,有時被 remap 成 odom。
        candidates = [cloud_frame] if self.world_frame is None else [self.world_frame]
        if self.world_frame is None:
            candidates += [f for f in ("odom", "camera_init", "map") if f != cloud_frame]

        tf = None
        frame = None
        for f in frames:
            for src in candidates:
                tf = self.lookup(COLOR_FRAME, src, f["stamp"])
                if tf is not None:
                    self.world_frame = src
                    frame = f
                    break
            if tf is not None:
                break

        if tf is None:
            if not self.color_warned:
                self.color_warned = True
                self.get_logger().warn(
                    f"查不到 {candidates} -> {COLOR_FRAME} 的 TF,點雲暫時不上色。"
                    f"確認 camera_extrinsic.sh 有跑、相機有開。")
            return out

        q = tf.transform.rotation
        v = tf.transform.translation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        t = np.array([v.x, v.y, v.z], dtype=np.float64)

        P = xyz.astype(np.float64) @ R.T + t          # 世界座標 -> 相機光學座標
        z = P[:, 2]
        idx = np.nonzero((z > COLOR_MIN_RANGE) & (z < COLOR_MAX_RANGE))[0]
        if idx.size == 0:
            return out

        color = frame["color"]
        depth = frame["depth"]
        H, W = color.shape[:2]
        fx, fy, cx, cy = self.feed.K

        zz = z[idx]
        u = np.rint(fx * P[idx, 0] / zz + cx).astype(np.int32)
        vv = np.rint(fy * P[idx, 1] / zz + cy).astype(np.int32)
        inside = (u >= 0) & (u < W) & (vv >= 0) & (vv < H)
        idx, u, vv, zz = idx[inside], u[inside], vv[inside], zz[inside]
        if idx.size == 0:
            return out

        if depth is not None:
            # 對齊過的深度跟彩色同一個像素格,所以直接查同一個 (u,v)。
            # 深度 0 代表這個像素量不到(黑色、反光、太近),不能判斷遮蔽,
            # 這時候放行 —— 從缺不上色會讓大片深色表面永遠是灰的。
            dz = depth[vv, u]
            ok = (dz <= 0.0) | (np.abs(dz - zz) < OCCLUSION_TOL)
            idx, u, vv = idx[ok], u[ok], vv[ok]
            if idx.size == 0:
                return out

        c = color[vv, u].astype(np.float32)
        out[idx] = c[:, 0] * 65536.0 + c[:, 1] * 256.0 + c[:, 2]
        return out

    # --- ROS callbacks ---

    def on_cloud(self, msg):
        n = msg.width * msg.height
        if n == 0:
            return

        if self.int_off is None:
            for f in msg.fields:
                if f.name == "intensity":
                    self.int_off = f.offset
            if self.int_off is None:
                self.int_off = -1
            self.get_logger().info(
                f"point_step={msg.point_step} intensity_offset={self.int_off} "
                f"frame_id={msg.header.frame_id}"
            )

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        need = n * msg.point_step
        if raw.size < need:
            return
        raw = raw[:need].reshape(n, msg.point_step)

        xyz = raw[:, 0:12].copy().view(np.float32).reshape(n, 3)
        if self.int_off >= 0:
            o = self.int_off
            inten = raw[:, o:o + 4].copy().view(np.float32).reshape(n)
        else:
            inten = np.zeros(n, dtype=np.float32)

        finite = np.isfinite(xyz).all(axis=1)
        xyz, inten = xyz[finite], inten[finite]
        if xyz.shape[0] == 0:
            return

        q = np.floor(xyz / VOXEL).astype(np.int64)
        key = ((q[:, 0] & 0x1FFFFF) << 42) | ((q[:, 1] & 0x1FFFFF) << 21) | (q[:, 2] & 0x1FFFFF)

        uniq, first = np.unique(key, return_index=True)

        pts = np.empty((len(first), STRIDE), dtype=np.float32)
        pts[:, 0:3] = xyz[first]
        pts[:, 3] = inten[first]
        # 只對去重後的體素上色,不是原始的兩萬點 —— 省一半以上的投影計算
        pts[:, COL_RGB] = self.colorize(pts[:, 0:3], msg.header.frame_id,
                                        stamp_sec(msg.header.stamp))

        # 整幀的體素都交給 Hub:已在圖上的續命,新的先累積命中數,
        # 達到 CONFIRM_HITS 才回傳給我們廣播出去。
        now = time.monotonic()
        stored = self.hub.ingest(uniq, pts, now)
        if stored is None or len(stored) == 0:
            return
        # 重送期間瀏覽器的寫入游標歸零在覆蓋舊資料,這時再推即時點會寫錯位置。
        # 少推這幾幀無所謂,下一輪 ingest 還是會補上。
        if self.rebuilding:
            return
        self.hub.push(np.ascontiguousarray(stored).tobytes())

    def on_sweep(self):
        """定期淘汰過期體素,然後整份重送。

        重送**不能**先送 cleared —— 那會讓瀏覽器立刻把 drawRange 歸零,
        而 15 萬點要好幾個 frame 才分塊送完,中間畫面全空,結果就是每 5 秒
        整片閃一次。改用 begin_rebuild/end_rebuild:瀏覽器原地從索引 0 覆蓋,
        畫面持續顯示舊內容,收完才一次切換 drawRange。
        新集合等於舊集合扣掉過期的點,所以覆蓋過程視覺上是無縫的。

        除了「有點被淘汰」之外,「夠多舊點拿到顏色」也要重送 ——
        那些點早就送到瀏覽器了,不重送的話它們會一直是沒上色的樣子。
        """
        changed = self.hub.sweep(time.monotonic(), self.sensor)
        if not changed and self.hub.recolored < RECOLOR_REBUILD:
            return
        self.hub.take_recolored()      # 這一輪要重送了,計數歸零
        snap = self.hub.snapshot()

        # 重送期間先停掉即時點的推送,否則兩條寫入路徑會搶同一個游標
        self.rebuilding = True
        try:
            self.hub.push(json.dumps({"t": "begin_rebuild", "n": int(len(snap))}))
            for i in range(0, len(snap), CHUNK):
                self.hub.push(np.ascontiguousarray(snap[i:i + CHUNK]).tobytes())
            self.hub.push(json.dumps({"t": "end_rebuild"}))
        finally:
            self.rebuilding = False

    def on_odom(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.sensor = (p.x, p.y, p.z)
        self.hub.push(
            json.dumps(
                {
                    "t": "odom",
                    "p": [p.x, p.y, p.z],
                    "q": [o.x, o.y, o.z, o.w],
                    "n": self.hub.count,
                    "c": self.hub.colored,
                }
            )
        )


async def ws_handler(ws, hub):
    hub.clients.add(ws)
    try:
        snap = hub.snapshot()
        await ws.send(json.dumps({"t": "begin_snapshot", "n": int(len(snap))}))
        for i in range(0, len(snap), CHUNK):
            await ws.send(np.ascontiguousarray(snap[i:i + CHUNK]).tobytes())
        await ws.send(json.dumps({"t": "end_snapshot"}))

        async for message in ws:
            if isinstance(message, str) and message == "clear":
                hub.reset()
                await hub._broadcast(json.dumps({"t": "cleared"}))
    except Exception:
        pass
    finally:
        hub.clients.discard(ws)


class WebHandler(SimpleHTTPRequestHandler):
    """靜態檔案 + /stats.json。

    /stats.json 是給 Windows 控制台輪詢的 —— 「上色率」是判斷相機這條線
    有沒有活著的唯一指標。點數在跑但上色率卡在 0,代表 TF 斷了,
    光看 process 還在不在完全看不出來。
    """

    hub = None

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/stats.json"):
            h = WebHandler.hub
            n = h.count if h else 0
            c = h.colored if h else 0
            body = json.dumps({
                "points": int(n),
                "colored": int(c),
                "colored_pct": round(100.0 * c / n, 1) if n else 0.0,
                "clients": len(h.clients) if h else 0,
            }).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        SimpleHTTPRequestHandler.do_GET(self)


def serve_http():
    handler = lambda *a, **kw: WebHandler(*a, directory=WEB_DIR, **kw)
    ThreadingHTTPServer.allow_reuse_address = True
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), handler)
    srv.serve_forever()


def main():
    hub = Hub()
    WebHandler.hub = hub

    threading.Thread(target=serve_http, daemon=True).start()

    rclpy.init()
    node = Bridge(hub)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    async def run():
        hub.loop = asyncio.get_running_loop()
        # websockets 10.x 的 handler 是 (ws, path),12.x 之後只有 (ws) —— 兩種都吃
        async with websockets.serve(
            lambda *a: ws_handler(a[0], hub), "0.0.0.0", WS_PORT, max_size=None
        ):
            await asyncio.Future()

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
