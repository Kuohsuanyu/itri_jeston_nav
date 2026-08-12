#!/usr/bin/env python3
"""把即時 /scan 疊在地圖上畫成 PNG —— 一眼看出定位對不對。

check_localization.py 給的是數字(命中率、距離中位數),知道「不好」
但不知道「怎麼個不好法」:是整體平移?旋轉?還是收斂到別的地方?

這支畫出來:
    灰   未知
    白   空曠
    黑   地圖上的牆
    紅   即時雷射點       ← 紅點要蓋在黑線上
    藍   車的位置和朝向

用法:
    python3 overlay_scan.py                       輸出 /tmp/overlay.png
    python3 overlay_scan.py /tmp/x.png            指定輸出
    python3 overlay_scan.py /tmp/x.png ~/maps/m.yaml
"""
import glob
import math
import os
import struct
import sys
import time
import zlib

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

MARGIN = 40          # 車周圍要留多少格才裁切(0 = 整張圖)


def read_pgm(path):
    d = open(path, "rb").read()
    tok, i = [], 2
    while len(tok) < 3:
        while d[i:i + 1].isspace():
            i += 1
        if d[i:i + 1] == b"#":
            while d[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        s = i
        while not d[i:i + 1].isspace():
            i += 1
        tok.append(int(d[s:i]))
    i += 1
    w, h, _ = tok
    return np.frombuffer(d[i:i + w * h], dtype=np.uint8).reshape(h, w)


def write_png(path, a):
    h, w = a.shape[:2]
    raw = b"".join(b"\x00" + a[r].tobytes() for r in range(h))

    def chunk(t, data):
        c = struct.pack(">I", len(data)) + t + data
        return c + struct.pack(">I", zlib.crc32(t + data) & 0xffffffff)

    open(path, "wb").write(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/overlay.png"
    ymls = sorted(glob.glob(os.path.expanduser("~/maps/*.yaml")),
                  key=os.path.getmtime, reverse=True)
    yaml_path = sys.argv[2] if len(sys.argv) > 2 else (ymls[0] if ymls else None)
    if not yaml_path or not os.path.isfile(yaml_path):
        print("✗ 找不到地圖")
        return 1

    meta = {}
    for line in open(yaml_path, encoding="utf-8"):
        if ":" in line:
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    ox, oy = [float(x) for x in meta["origin"].strip("[]").split(",")][:2]
    img = read_pgm(os.path.join(os.path.dirname(yaml_path), meta["image"]))
    h, w = img.shape

    rclpy.init()
    n = Node("overlay_scan")
    buf = Buffer()
    TransformListener(buf, n)
    pts = []
    pose = {}

    def cb(msg):
        try:
            t = buf.lookup_transform("map", msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tx, ty = t.transform.translation.x, t.transform.translation.y
        pose.update(x=tx, y=ty, yaw=yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        r, a = r[ok], a[ok]
        lx, ly = r * np.cos(a), r * np.sin(a)
        pts.append(np.stack([tx + c * lx - s * ly, ty + s * lx + c * ly], -1))

    n.create_subscription(
        LaserScan, "/scan", cb,
        QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST))
    t0 = time.time()
    while time.time() - t0 < 8 and len(pts) < 12:
        rclpy.spin_once(n, timeout_sec=0.1)

    if not pts:
        print("✗ 收不到 /scan,或查不到 map -> 雷射的變換")
        print("   AMCL 有沒有設初始位置?ros2 topic echo /amcl_pose --once")
        return 1

    P = np.vstack(pts)
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    occ, free = img < 90, img > 220
    rgb[~occ & ~free] = (150, 150, 150)
    rgb[free] = (255, 255, 255)
    rgb[occ] = (0, 0, 0)

    # 雷射點畫紅色。PGM 第一列是 y 最大,所以 row = h-1-gy
    gx = ((P[:, 0] - ox) / res).astype(int)
    gy = ((P[:, 1] - oy) / res).astype(int)
    m = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
    rgb[h - 1 - gy[m], gx[m]] = (255, 0, 0)

    # 車:藍點 + 朝向線
    cx = int((pose["x"] - ox) / res)
    cy = h - 1 - int((pose["y"] - oy) / res)
    for dx in range(-3, 4):
        for dy in range(-3, 4):
            if 0 <= cy + dy < h and 0 <= cx + dx < w:
                rgb[cy + dy, cx + dx] = (0, 80, 255)
    for d in range(0, 22):
        px = cx + int(d * math.cos(pose["yaw"]))
        py = cy - int(d * math.sin(pose["yaw"]))
        if 0 <= py < h and 0 <= px < w:
            rgb[py, px] = (0, 80, 255)

    # 裁切到車附近,不然 81 公尺的圖縮成一條線什麼都看不到
    if MARGIN > 0:
        span = max(MARGIN, int(np.abs(P[:, 0] - pose["x"]).max() / res) + 20)
        r0, r1 = max(0, cy - span), min(h, cy + span)
        c0, c1 = max(0, cx - span), min(w, cx + span)
        rgb = rgb[r0:r1, c0:c1]

    write_png(out, rgb)
    print("車在 (%.2f, %.2f)  朝向 %.1f 度" % (pose["x"], pose["y"], math.degrees(pose["yaw"])))
    print("雷射點 %d 個,%d 幀" % (len(P), len(pts)))
    print("輸出 %s  (%d x %d)" % (out, rgb.shape[1], rgb.shape[0]))
    print()
    print("看法:紅點應該蓋在黑線上。整體平移 = 初始位置給偏了;")
    print("      紅點形狀跟黑線對不上 = 收斂到別的地方,或環境變了。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
