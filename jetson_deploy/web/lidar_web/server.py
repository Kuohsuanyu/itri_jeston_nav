#!/usr/bin/env python3
"""
lidar_web_viewer — 把 FAST-LIO 的點雲和位姿推到瀏覽器。

訂閱:
  /cloud_registered  sensor_msgs/PointCloud2   世界座標的配準點雲
  /Odometry          nav_msgs/Odometry         即時位姿

對外:
  HTTP  :8080   靜態網頁
  WS    :8081   binary = 新增的點 (float32 x,y,z,intensity),text = JSON 狀態

點雲以體素去重,只推「新出現的體素」,所以頻寬跟環境大小有關,
而不是跟掃描頻率有關 —— 站著不動時幾乎不佔頻寬。
"""

import asyncio
import math
import json
import os
import threading
import time

import numpy as np
import rclpy
import websockets
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

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
                      # 縮到 20_000(320 KB)讓 odom 有機會插隊,資料流順很多。
STRIDE = 4            # 每點 4 個 float32:x, y, z, intensity

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

# --- 顯示用的扶正 ------------------------------------------------------
# /cloud_registered 在 camera_init 座標系裡,那是 FAST-LIO 開機瞬間的光達
# 姿態 —— 光達機構上斜 29.7 度,所以畫出來整片是歪的。
#
# 這裡量重力算一個固定旋轉,只作用在「送去瀏覽器之前」。體素索引、衰減、
# 距離閘門全都在旋轉後的座標系裡一致地運作(旋轉是剛體變換,不改變任何
# 相對關係),所以不影響行為,只影響看起來正不正。
#
# ★ 這個旋轉不會發布成 TF,也不影響 /scan、slam_toolbox、Nav2。
#   那些走的是 TF,跟這裡完全無關。
LEVEL_SAMPLES = 400      # 200 Hz 下約 2 秒



def level_basis(up):
    """給一個「上」方向(在來源座標系裡),回傳把它轉正的旋轉矩陣。

    z 軸對齊 up;x 軸取原座標系 x 投影到水平面 —— 這樣航向不會亂轉,
    只是把傾斜扳平。回傳 R 滿足 v_水平 = R @ v_原。
    """
    z = up / np.linalg.norm(up)
    x = np.array([1.0, 0.0, 0.0])
    x = x - np.dot(x, z) * z
    nx = np.linalg.norm(x)
    if nx < 1e-6:                      # 光達幾乎側躺,改用 y 當提示
        x = np.array([0.0, 1.0, 0.0])
        x = x - np.dot(x, z) * z
        nx = np.linalg.norm(x)
    x /= nx
    y = np.cross(z, x)
    # 直行是 v_原 = [x|y|z] @ v_水平,所以反過來要轉置
    return np.column_stack([x, y, z]).T


class Hub:
    """管理瀏覽器連線,負責廣播。"""

    def __init__(self):
        self.clients = set()
        self.loop = None
        self.lock = threading.Lock()
        self.buf = np.zeros((MAX_POINTS, STRIDE), dtype=np.float32)
        self.count = 0

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
                return False

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
            self.live = {int(k): i for i, k in enumerate(self.key_of[:n_keep].tolist())}
            return True

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


class Bridge(Node):
    def __init__(self, hub):
        super().__init__("lidar_web_viewer")
        self.hub = hub
        self.int_off = None      # intensity 在 point_step 內的位移,首幀自動偵測
        self.rebuilding = False  # 衰減重送進行中,期間暫停推即時點
        self.sensor = (0.0, 0.0, 0.0)   # 目前感測器位置,衰減的距離閘門要用

        # 顯示用的扶正。None = 還在收 IMU,這段期間畫面是歪的(約 2 秒)。
        self.level = None
        self.acc = []
        self.create_subscription(
            Imu, "/livox/imu", self.on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "/cloud_registered", self.on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/Odometry", self.on_odom, 10)
        self.create_timer(SWEEP_SEC, self.on_sweep)
        self.get_logger().info(
            f"lidar_web_viewer 啟動 — HTTP :{HTTP_PORT}  WS :{WS_PORT}  voxel={VOXEL}m "
            f"confirm={CONFIRM_HITS}幀 decay={DECAY_SEC}s"
        )

    def on_imu(self, msg):
        """只在啟動時取樣一次。算完就不再更新 —— 每幀重算的話,
        車子上下坡或加減速時整個畫面會跟著晃,反而更難看。"""
        if self.level is not None:
            return
        a = msg.linear_acceleration
        self.acc.append([a.x, a.y, a.z])
        if len(self.acc) < LEVEL_SAMPLES:
            return
        A = np.asarray(self.acc, dtype=np.float64)
        mag = np.linalg.norm(A, axis=1)
        jitter = float(mag.std() / max(mag.mean(), 1e-9))
        up = A.mean(axis=0)
        up /= np.linalg.norm(up)
        self.level = level_basis(up).astype(np.float32)
        tilt = math.degrees(math.acos(min(1.0, max(-1.0, float(up[2])))))
        self.get_logger().info(
            "扶正完成:光達傾斜 %.2f 度,取樣抖動 %.4f%s"
            % (tilt, jitter,
               "  (抖動偏大,車可能在動,重啟可重新取樣)" if jitter > 0.02 else "")
        )
        self.acc = []

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
                f"point_step={msg.point_step} intensity_offset={self.int_off}"
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

        # 扶正。體素索引在旋轉後的座標系裡算,所以整份地圖是一致的 ——
        # 不會出現「舊點是歪的、新點是正的」那種撕裂。
        if self.level is not None:
            xyz = xyz @ self.level.T

        q = np.floor(xyz / VOXEL).astype(np.int64)
        key = ((q[:, 0] & 0x1FFFFF) << 42) | ((q[:, 1] & 0x1FFFFF) << 21) | (q[:, 2] & 0x1FFFFF)

        uniq, first = np.unique(key, return_index=True)

        pts = np.empty((len(first), STRIDE), dtype=np.float32)
        pts[:, 0:3] = xyz[first]
        pts[:, 3] = inten[first]

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
        """
        if not self.hub.sweep(time.monotonic(), self.sensor):
            return
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
        pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        quat = [o.x, o.y, o.z, o.w]
        if self.level is not None:
            L = self.level.astype(np.float64)
            pos = L @ pos
            # 姿態也要轉,否則軌跡標記的三軸方向跟點雲對不上
            R = np.array([
                [1 - 2 * (o.y * o.y + o.z * o.z), 2 * (o.x * o.y - o.z * o.w), 2 * (o.x * o.z + o.y * o.w)],
                [2 * (o.x * o.y + o.z * o.w), 1 - 2 * (o.x * o.x + o.z * o.z), 2 * (o.y * o.z - o.x * o.w)],
                [2 * (o.x * o.z - o.y * o.w), 2 * (o.y * o.z + o.x * o.w), 1 - 2 * (o.x * o.x + o.y * o.y)]])
            Rl = L @ R
            tr = Rl[0, 0] + Rl[1, 1] + Rl[2, 2]
            if tr > 0:
                s = math.sqrt(tr + 1.0) * 2
                quat = [(Rl[2, 1] - Rl[1, 2]) / s, (Rl[0, 2] - Rl[2, 0]) / s,
                        (Rl[1, 0] - Rl[0, 1]) / s, 0.25 * s]
            elif Rl[0, 0] > Rl[1, 1] and Rl[0, 0] > Rl[2, 2]:
                s = math.sqrt(1.0 + Rl[0, 0] - Rl[1, 1] - Rl[2, 2]) * 2
                quat = [0.25 * s, (Rl[0, 1] + Rl[1, 0]) / s, (Rl[0, 2] + Rl[2, 0]) / s,
                        (Rl[2, 1] - Rl[1, 2]) / s]
            elif Rl[1, 1] > Rl[2, 2]:
                s = math.sqrt(1.0 + Rl[1, 1] - Rl[0, 0] - Rl[2, 2]) * 2
                quat = [(Rl[0, 1] + Rl[1, 0]) / s, 0.25 * s, (Rl[1, 2] + Rl[2, 1]) / s,
                        (Rl[0, 2] - Rl[2, 0]) / s]
            else:
                s = math.sqrt(1.0 + Rl[2, 2] - Rl[0, 0] - Rl[1, 1]) * 2
                quat = [(Rl[0, 2] + Rl[2, 0]) / s, (Rl[1, 2] + Rl[2, 1]) / s, 0.25 * s,
                        (Rl[1, 0] - Rl[0, 1]) / s]
        self.sensor = (float(pos[0]), float(pos[1]), float(pos[2]))
        self.hub.push(
            json.dumps(
                {
                    "t": "odom",
                    "p": [float(pos[0]), float(pos[1]), float(pos[2])],
                    "q": [float(q) for q in quat],
                    "n": self.hub.count,
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


def serve_http():
    handler = lambda *a, **kw: SimpleHTTPRequestHandler(*a, directory=WEB_DIR, **kw)
    ThreadingHTTPServer.allow_reuse_address = True
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), handler)
    srv.serve_forever()


def main():
    hub = Hub()

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
