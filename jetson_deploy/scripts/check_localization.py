#!/usr/bin/env python3
"""定位對不對?把 /scan 投影到地圖上,量有多少雷射點落在牆上。

── 為什麼需要客觀判準 ────────────────────────────────────────────
AMCL 一定會給你一個位姿 —— 就算完全錯它也照發,而且不會有任何警告。
在 RViz 裡用肉眼看「紅點有沒有貼在牆上」可以判斷,但那要有人盯著,
而且說不出「貼得多好」。

這支把同一件事量化:

    對每個雷射點,找地圖上最近的佔據格。
    距離在容差內的算命中。命中率高 = 定位對。

    命中率 > 70%   定位good
    50 ~ 70%       大致對但有偏移,或環境跟建圖時有變(家具移動之類)
    < 50%          定位錯了,或根本收斂到別的地方

用法:
    python3 check_localization.py                        用最新的地圖
    python3 check_localization.py ~/maps/map_xxx.yaml
    python3 check_localization.py <地圖> 15              量 15 秒
"""
import glob
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

TOL = 0.25          # 命中容差(公尺)。0.25 = 5 個格子,容得下地圖本身的量化誤差
MAP_FRAME = "map"
# 最近鄰只搜鄰近 9 格(邊長 TOL),所以量得到的距離上限是 1.5*TOL*sqrt(2)。
# 超過的一律當「附近沒有牆」,不能拿哨兵值去算中位數。
CAP = 1.5 * TOL * math.sqrt(2)


def load_map(yaml_path):
    """讀 map_server 格式的 .yaml + .pgm,回傳佔據格的世界座標。"""
    meta = {}
    for line in open(yaml_path, encoding="utf-8"):
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    origin = [float(x) for x in meta["origin"].strip("[]").split(",")]
    occ_th = float(meta.get("occupied_thresh", 0.65))
    pgm = os.path.join(os.path.dirname(yaml_path), meta["image"])

    # 自己解 PGM(P5 二進位)。不想為了讀一張灰階圖裝 PIL/OpenCV。
    with open(pgm, "rb") as f:
        data = f.read()
    # 標頭:P5 <寬> <高> <最大值>,中間可能夾註解行
    tok, i = [], 2
    while len(tok) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        s = i
        while i < len(data) and not data[i:i + 1].isspace():
            i += 1
        tok.append(int(data[s:i]))
    i += 1
    w, h, maxv = tok
    img = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)

    # map_server 的 trinary:亮 = 空曠,暗 = 障礙。
    # occupancy = (255 - pixel) / 255,大於 occupied_thresh 就是佔據。
    occ = (255.0 - img.astype(np.float64)) / 255.0 > occ_th
    ys, xs = np.nonzero(occ)
    # PGM 第一列是影像最上面,對應地圖 y 最大的那一列 —— 要翻轉
    wx = origin[0] + (xs + 0.5) * res
    wy = origin[1] + (h - 1 - ys + 0.5) * res
    return np.stack([wx, wy], axis=-1), res, (w, h), origin


def main():
    args = [a for a in sys.argv[1:]]
    ymls = sorted(glob.glob(os.path.expanduser("~/maps/*.yaml")),
                  key=os.path.getmtime, reverse=True)
    yaml_path = args[0] if args and args[0].endswith(".yaml") else (ymls[0] if ymls else None)
    secs = float(args[-1]) if args and not args[-1].endswith(".yaml") else 10.0
    if not yaml_path or not os.path.isfile(yaml_path):
        print("✗ 找不到地圖。~/maps 裡有:")
        for y in ymls:
            print("   ", y)
        return 1

    print("地圖 %s" % yaml_path)
    occ, res, (w, h), origin = load_map(yaml_path)
    print("  %d x %d 格 @ %.3f m = %.1f x %.1f 公尺,origin %s"
          % (w, h, res, w * res, h * res, origin[:2]))
    print("  佔據格 %d 個" % len(occ))
    if len(occ) == 0:
        print("✗ 地圖裡沒有任何障礙物格,無法比對")
        return 1

    # 用格點雜湊做最近鄰。scipy 不一定裝了,而且這樣夠快:
    # 把佔據格丟進以 TOL 為邊長的格子,查詢時只看鄰近 9 格。
    #
    # ★ 只看 9 格 = 量得到的距離有上限。超過上限的點 near() 回傳哨兵值,
    #   呼叫端要用 CAP 濾掉,不能直接拿去算中位數。
    cell = TOL
    grid = {}
    for x, y in occ:
        grid.setdefault((int(x // cell), int(y // cell)), []).append((x, y))

    def near(px, py):
        cx, cy = int(px // cell), int(py // cell)
        best = 1e9
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for ox, oy in grid.get((cx + dx, cy + dy), ()):
                    d = (ox - px) ** 2 + (oy - py) ** 2
                    if d < best:
                        best = d
        return math.sqrt(best)

    rclpy.init()
    n = Node("check_localization")
    buf = Buffer()
    TransformListener(buf, n)
    stat = {"scans": 0, "pts": 0, "hit": 0, "d": [], "notf": 0}

    def cb(msg):
        try:
            t = buf.lookup_transform(MAP_FRAME, msg.header.frame_id,
                                     rclpy.time.Time())
        except Exception:
            stat["notf"] += 1
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        tx, ty = t.transform.translation.x, t.transform.translation.y
        c, s = math.cos(yaw), math.sin(yaw)
        stat["scans"] += 1
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            px, py = tx + c * lx - s * ly, ty + s * lx + c * ly
            d = near(px, py)
            stat["pts"] += 1
            stat["d"].append(d)
            if d <= TOL:
                stat["hit"] += 1

    n.create_subscription(
        LaserScan, "/scan", cb,
        QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST))

    print("\n取樣 %.0f 秒..." % secs)
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(n, timeout_sec=0.1)

    if stat["notf"] and not stat["scans"]:
        print("✗ 查不到 %s -> %s 的變換 —— AMCL 還沒設初始位置?"
              % (MAP_FRAME, "雷射的 frame"))
        return 1
    if not stat["pts"]:
        print("✗ 一個有效的雷射點都沒有。/scan 有資料嗎?")
        return 1

    rate = 100.0 * stat["hit"] / stat["pts"]
    # ★ near() 找不到牆時回傳 sqrt(1e9) = 31622.777。那是哨兵值,不是距離 ——
    #   混進中位數裡就會印出「距離中位數 31622.777 m」這種無意義的東西
    #   (2026-08-13 實測)。分開統計:有找到的算距離,沒找到的算比例。
    #
    #   搜尋範圍只有鄰近 9 格(邊長 TOL),所以量得到的距離上限是
    #   1.5 * TOL * sqrt(2) ≈ %.2f m —— 超過就一律是「附近沒有牆」。
    ds = sorted(d for d in stat["d"] if d < CAP)
    miss = stat["pts"] - len(ds)
    med = ds[len(ds) // 2] if ds else None
    p90 = ds[int(len(ds) * 0.9)] if ds else None

    try:
        t = buf.lookup_transform(MAP_FRAME, "base_footprint", rclpy.time.Time())
        v = t.transform.translation
        q = t.transform.rotation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y * q.y + q.z * q.z)))
        print("\n目前位置  (%.2f, %.2f)  朝向 %.1f 度" % (v.x, v.y, yaw))
    except Exception:
        pass

    print("\n%d 幀 / %d 點" % (stat["scans"], stat["pts"]))
    print("  命中率(距離最近牆面 <= %.2f m)  %.1f %%" % (TOL, rate))
    if med is not None:
        print("  在 %.2f m 內找得到牆的點:%d(%.1f %%)"
              % (CAP, len(ds), 100.0 * len(ds) / stat["pts"]))
        print("    這些點的距離:中位數 %.3f m   90%% 分位 %.3f m" % (med, p90))
    print("  附近完全沒有牆的點:%d(%.1f %%)"
          % (miss, 100.0 * miss / stat["pts"]))
    print("    ↑ 這個比例高 = 掃到的東西在地圖上根本不存在,")
    print("      通常是車不在地圖範圍內(例如停在車庫),不是參數問題")
    print()
    if rate > 70:
        print("  ✓ 定位good")
    elif rate > 50:
        print("  ~ 大致對,但有偏移。可能是:")
        print("      初始位置給得不夠準 —— 在 RViz 用 2D Pose Estimate 修")
        print("      環境跟建圖時不同(家具、門開關)")
        print("      這張圖是 08-10 建的,當時 /scan 的高度帶還差 12 公分")
    else:
        print("  ✗ 定位錯了。粒子可能收斂到長得像的另一個位置。")
        print("      在 RViz 用 2D Pose Estimate 點車子的實際位置")
        print("      或推車走幾公尺讓掃描匹配自己修正")
    return 0


if __name__ == "__main__":
    sys.exit(main())
