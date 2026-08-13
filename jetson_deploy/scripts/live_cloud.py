#!/usr/bin/env python3
"""即時三維點雲檢視 —— 在瀏覽器上看 Jetson 的雷射回波。

    python3 ~/slam2d/live_cloud.py            預設 :8100
    python3 ~/slam2d/live_cloud.py 8100 2     指定埠號和抽稀倍率

然後在筆電上開  http://192.168.40.98:8100

── 為什麼用 /cloud_registered_body 而不是 /livox/lidar ──────────────
/livox/lidar 是 livox_ros_driver2/CustomMsg —— 真正的原始格式,每點自帶
line(掃描線)和 offset_time(幀內時間)。但 rclpy 把它反序列化成一個
**Python 物件的 list**,兩萬個點就要跑兩萬次屬性存取,一幀要好幾百毫秒。
那樣的話畫面永遠落後半秒以上,「即時」就沒意義了。

/cloud_registered_body 是 sensor_msgs/PointCloud2,底層是一塊連續的
bytes,可以用 numpy 的 structured dtype **一次切完**,幾毫秒的事。
代價是少了 line / offset_time,而且它是 FAST-LIO 去畸變之後的雲。

要看真正的原始欄位,用快照的方式(grab 一段下來離線看),不要走即時。

── 這支不會被 bringup_all.sh 自動啟動 ─────────────────────────────
要看的時候才開,看完 Ctrl-C。理由是 2026-08-12 的教訓:三個常駐的網頁
檢視器加起來吃掉近 100% CPU,load average 衝到 5.4,核心來不及處理網路
封包(RX dropped 5289),從筆電 ping Jetson 變成 26% 遺失、最大 1096 ms。
"""
import http.server
import socket
import socketserver
import struct
import sys
import threading
import time
import pathlib

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8100
# 每 STEP 點取 1。預設 1 —— /cloud_registered_body 已經被 FAST-LIO 抽稀過
# (point_filter_num 4 + filter_size_surf 0.5),實測一幀只有 4778 點,
# 再抽一半就太稀了。要更省頻寬才調大。
STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 1
TOPIC = "/cloud_registered_body"

# int16 量化。0.002 m 一階 -> ±65.5 m,涵蓋 Mid-360 的 40 m 射程還有餘裕。
# 快照版用 0.00025(±8.19 m)是因為那批資料在車庫;走廊會超出去,
# 用小的階距會**默默地把遠處的點折回來**,看起來像鬼影。
SCALE = 0.002
MAGIC = b"LVX1"

HTML = (pathlib.Path(__file__).parent / "live_cloud.html").read_bytes()

_lock = threading.Lock()
_frame = {"buf": None, "seq": 0, "stamp": 0.0, "n": 0, "hz": 0.0, "raw": 0}

_PF = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def parse(msg):
    """照 PointField 的 offset 解出 xyz + intensity。

    ★ 不要用 point_cloud2.read_points_numpy —— 2026-08-11 實測它回傳的
      距離中位數是 3867 公尺(正確值 2 公尺附近)。它假設欄位是緊密排列的,
      而 FAST-LIO 的雲有 padding。自己照 offset 建 dtype 才對。
    """
    names = [f.name for f in msg.fields]
    dt = np.dtype({
        "names": names,
        "formats": [_PF[f.datatype] for f in msg.fields],
        "offsets": [f.offset for f in msg.fields],
        "itemsize": msg.point_step,
    })
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    xyz = np.stack([a["x"], a["y"], a["z"]], -1).astype(np.float32)
    inten = (a["intensity"].astype(np.float32) if "intensity" in names
             else np.zeros(len(a), np.float32))
    return xyz, inten


class Sub(Node):
    def __init__(self):
        super().__init__("live_cloud")
        self.times = []          # 最近的到達時刻,用來算真實幀率
        self.create_subscription(
            PointCloud2, TOPIC, self.cb,
            QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))
        self.get_logger().info("subscribed %s" % TOPIC)

    def cb(self, msg):
        # ★ 幀率用「兩秒內收到幾則」算,不要用 EMA。
        #   原本寫 hz = 0.8*hz + 0.2/dt,兩則訊息幾乎同時到時 dt 趨近 0,
        #   0.2/dt 會爆成幾百,再花二十幾幀才衰減回來 —— 實測顯示 49 Hz
        #   而真實值是 8.9 Hz。計數法沒有這個問題。
        now = time.time()
        self.times.append(now)
        while self.times and now - self.times[0] > 2.0:
            self.times.pop(0)
        span = now - self.times[0] if len(self.times) > 1 else 0.0
        hz = (len(self.times) - 1) / span if span > 0 else 0.0

        xyz, inten = parse(msg)
        raw_n = len(xyz)
        ok = np.isfinite(xyz).all(1)
        xyz, inten = xyz[ok][::STEP], inten[ok][::STEP]
        if not len(xyz):
            return
        q = np.clip(np.round(xyz / SCALE), -32768, 32767).astype("<i2")
        it = np.clip(inten, 0, 255).astype(np.uint8)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        buf = (struct.pack("<4sIIfd", MAGIC, _frame["seq"] + 1, len(q), SCALE, stamp)
               + q.tobytes() + it.tobytes())
        with _lock:
            _frame.update(buf=buf, seq=_frame["seq"] + 1, stamp=stamp,
                          n=len(q), hz=hz, raw=raw_n)


class H(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass                                    # 不要每次抓幀都印一行

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        p = self.path.split("?")[0]
        if p in ("/", "/index.html"):
            self._send(200, "text/html; charset=utf-8", HTML)
        elif p == "/f":
            with _lock:
                buf = _frame["buf"]
            if buf is None:
                self._send(503, "text/plain", b"no cloud yet")
            else:
                self._send(200, "application/octet-stream", buf)
        elif p == "/s":
            with _lock:
                s = ('{"seq":%d,"n":%d,"raw":%d,"hz":%.2f,"step":%d,"topic":"%s"}'
                     % (_frame["seq"], _frame["n"], _frame["raw"],
                        _frame["hz"], STEP, TOPIC))
            self._send(200, "application/json", s.encode())
        else:
            self._send(404, "text/plain", b"not found")


class Srv(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def ips():
    out = []
    for host in ("192.168.40.1", "192.168.0.1"):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((host, 1))
            out.append(s.getsockname()[0])
        except OSError:
            pass
        finally:
            s.close()
    return sorted(set(out))


def main():
    srv = Srv(("0.0.0.0", PORT), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print("=" * 54)
    for ip in ips():
        print("  http://%s:%d" % (ip, PORT))
    print("  topic %s   每 %d 點取 1" % (TOPIC, STEP))
    print("  Ctrl-C 結束(看完就關,不要一直開著)")
    print("=" * 54)

    rclpy.init()
    n = Sub()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        print("\n收工")
    finally:
        srv.shutdown()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
