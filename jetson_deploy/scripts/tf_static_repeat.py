#!/usr/bin/env python3
"""定期重發 /tf_static,讓晚到的訂閱者也拿得到。

── 為什麼需要這支 ────────────────────────────────────────────────
/tf_static 用 TRANSIENT_LOCAL,每個發布者**開機時發一次就閉嘴**。同機的
DDS 訂閱者靠 QoS 的保留樣本可以補到,但一旦中間隔了東西就不行:

    底盤 rsp ─DDS─ Jetson 的 zenoh bridge ─TCP─ WSL 的 bridge ─DDS─ RViz
                   ↑ 這一跳要 replay        ↑ 這一跳也要

zenoh-bridge-ros2dds 沒有保留歷史訊息的能力(要另外配 storage plugin),
所以每一端只收得到「它連上之後」才發的東西。任何一端重啟,靜態變換就斷。

實測(2026-08-11):Jetson 上 base_footprint -> base_link 查得到,WSL 的
RViz 完全收不到任何靜態變換,樹只剩 map -> multi_odom -> base_footprint。
車體、光達、相機全部畫不出來。之前偶爾成功是碰巧有發布者在兩端都連著時
重新發布 —— 那是運氣不是機制。

── 做法 ──────────────────────────────────────────────────────
訂閱 /tf_static(TRANSIENT_LOCAL,所以啟動時就收得到現有的全部),
累積成一張「child -> transform」的表,然後每 PERIOD 秒把整張表重發一次。

★ 重發同樣的靜態變換不會製造雙父節點。tf2 的靜態緩衝以 child 為鍵,
  同一個 child 配同一個 parent、同樣的數值,只是覆寫成一樣的東西。
  真正會出事的是**不同 parent**,那跟重發無關。

★ 成本可以忽略:十幾筆變換,每筆約 100 bytes,兩秒一次 = 不到 1 KB/s。

── 用法 ─────────────────────────────────────────────────────
    python3 tf_static_repeat.py          預設每 2 秒
    python3 tf_static_repeat.py 5        每 5 秒

robot_tf.sh 會自己啟動它,一般不用手動跑。
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from tf2_msgs.msg import TFMessage

PERIOD = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0

# TRANSIENT_LOCAL + RELIABLE:這是 /tf_static 的標準 QoS,
# 訂閱時會立刻收到所有現存發布者的保留樣本。
QOS = QoSProfile(depth=200,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST)


class Repeater(Node):
    def __init__(self):
        super().__init__("tf_static_repeat")
        self.store = {}          # child_frame_id -> TransformStamped
        self.n_pub = 0
        # 先建 publisher 再建 subscription。反過來的話,自己發的第一批會在
        # subscription 建好前送出,雖然不影響正確性但 log 會少一次計數。
        self.pub = self.create_publisher(TFMessage, "/tf_static", QOS)
        self.create_subscription(TFMessage, "/tf_static", self.on_static, QOS)
        self.create_timer(PERIOD, self.republish)
        self.create_timer(30.0, self.report)
        self.get_logger().info(
            "tf_static_repeat 啟動,每 %.1f 秒重發一次" % PERIOD)

    def on_static(self, msg):
        for t in msg.transforms:
            key = t.child_frame_id
            old = self.store.get(key)
            if old is not None and old.header.frame_id != t.header.frame_id:
                # 這是真正該擔心的情況:同一個 child 換了 parent。
                # 不是我們造成的(我們發的跟收到的一模一樣),但既然看到了就講。
                self.get_logger().warn(
                    "%s 的父節點從 %s 變成 %s —— 有兩個來源在搶,查一下"
                    % (key, old.header.frame_id, t.header.frame_id))
            self.store[key] = t

    def republish(self):
        if not self.store:
            return
        msg = TFMessage()
        # 時間戳保持原樣。靜態變換的時戳在 tf2 裡本來就不參與內插,
        # 改成「現在」反而會讓查詢舊時間點的人失敗。
        msg.transforms = list(self.store.values())
        self.pub.publish(msg)
        self.n_pub += 1

    def report(self):
        self.get_logger().info(
            "重發 %d 次,目前 %d 筆:%s"
            % (self.n_pub, len(self.store), ", ".join(sorted(self.store))))


def main():
    rclpy.init()
    n = Repeater()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
