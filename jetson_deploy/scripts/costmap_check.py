#!/usr/bin/env python3
"""檢查 global_costmap 到底長什麼樣,以及目標點落在哪一格。

規劃出空路徑最常見的兩個原因:
  1. static_layer 沒收到 /map(QoS 不符),costmap 停在預設小尺寸
  2. 目標點落在 costmap 範圍外,或落在致命代價的格子上
"""
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener

got = {}


class C(Node):
    def __init__(self):
        super().__init__("costmap_check")
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 lambda m: got.setdefault("cm", m), qos)
        self.create_subscription(OccupancyGrid, "/map",
                                 lambda m: got.setdefault("map", m), qos)
        self.buf = Buffer()
        TransformListener(self.buf, self)


def show(name, m):
    if m is None:
        print(f"  {name}: 沒收到")
        return None
    i = m.info
    d = np.asarray(m.data, dtype=np.int8)
    print(f"  {name}: {i.width} x {i.height} @ {i.resolution:.3f} m  "
          f"原點 ({i.origin.position.x:.2f}, {i.origin.position.y:.2f})  "
          f"涵蓋 {i.width*i.resolution:.1f} x {i.height*i.resolution:.1f} m")
    unk = int((d < 0).sum())
    free = int((d == 0).sum())
    leth = int((d >= 253).sum())
    print(f"        未知 {unk} ({100*unk/d.size:.0f}%)  "
          f"free {free} ({100*free/d.size:.0f}%)  致命 {leth}")
    return m


def cell_at(m, x, y):
    i = m.info
    cx = int((x - i.origin.position.x) / i.resolution)
    cy = int((y - i.origin.position.y) / i.resolution)
    if not (0 <= cx < i.width and 0 <= cy < i.height):
        return None, (cx, cy)
    return int(np.asarray(m.data, dtype=np.int8)[cy * i.width + cx]), (cx, cy)


def main():
    rclpy.init()
    n = C()
    t0 = time.time()
    while time.time() - t0 < 15 and ("cm" not in got or "map" not in got):
        rclpy.spin_once(n, timeout_sec=0.2)

    print("=== 圖層尺寸 ===")
    mp = show("/map            ", got.get("map"))
    cm = show("/global_costmap ", got.get("cm"))

    if cm is None:
        print("\n✗ costmap 沒發布 —— planner_server 可能沒 active")
        return

    # 現在位置
    tf = None
    t0 = time.time()
    while time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.2)
        try:
            tf = n.buf.lookup_transform("map", "base_link", rclpy.time.Time())
            break
        except Exception:
            pass
    if tf is None:
        print("\n拿不到 TF")
        return
    x, y = tf.transform.translation.x, tf.transform.translation.y
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    print(f"\n=== 位置與目標在 costmap 上的代價值 ===")
    print("    (-1=未知  0=free  253+=致命  中間值=膨脹區)")
    v, c = cell_at(cm, x, y)
    print(f"  機器人 ({x:6.2f},{y:6.2f}) 格({c[0]:4d},{c[1]:4d}) -> "
          f"{'範圍外' if v is None else v}")
    for d in (1.0, 2.0, 3.0):
        gx, gy = x + d * math.cos(yaw), y + d * math.sin(yaw)
        v, c = cell_at(cm, gx, gy)
        print(f"  前方{d:.0f}m ({gx:6.2f},{gy:6.2f}) 格({c[0]:4d},{c[1]:4d}) -> "
              f"{'範圍外' if v is None else v}")


if __name__ == "__main__":
    main()
