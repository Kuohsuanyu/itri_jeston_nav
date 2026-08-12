#!/usr/bin/env python3
"""把底盤發的輪子 TF 從 Jetson 本機再發一次,讓它過得了 zenoh bridge。

── 問題 ──────────────────────────────────────────────────────────
zenoh-bridge-ros2dds 是按「**探索到的 ROS 節點**」建路由的,不是按 topic。
而 bridge 探索不到底盤(樹莓派)的節點。

★ 2026-08-12 排除過的假設,都不是原因:
    「bridge 啟動太早」  在確認 ros2 node list 看得到底盤之後才重啟 —— 還是 0 個
    「initialPeersList 蓋掉 multicast」  拿掉 fastdds_peers.xml —— 還是 0 個
    「WiFi 丟包」        ping 0% 遺失、資料流 20 秒 0 次斷點
  而且 ros2 node list **看得到**那兩個節點,bridge 就是看不到 ——
  兩者用的探索機制不同,bridge 那條對底盤失效。

  真正的根因(RMW 差異?bridge 的 USER_DATA 解析?)沒有釘死,
  但已知在這個組合下復現率 100%,所以繞過它。

實際觀測:

    ros2 node list          看不到 /chassis_driver /robot_state_publisher
    bridge 的 log           Discovered ROS Node 清單裡一個底盤的都沒有
    但資料收得到            /tf 上的輪子 5~10 Hz、/joint_states 10 Hz

原因是 WiFi 上的 DDS 圖探索不對稱:端點探索完成過(所以資料在流),但
節點層級的公告靠持續的 multicast 維持,而 AP 對 multicast 不可靠。
試過 initialPeersList 寫死底盤 IP,沒有用。

結果:凡是底盤節點發的東西,bridge 都不建路由 —— 輪子的 TF 到不了 WSL,
RViz 的 RobotModel 就少了四個輪子的 frame。RobotModel 是**逐 link 查 TF**
決定位置的,查不到的 link 畫不出來,整台車看起來像散掉。

── 做法 ──────────────────────────────────────────────────────────
訂閱 /tf,把 child 是 wheel_* 的變換存起來,再用**這個節點自己**發一次。
bridge 探索得到本機節點,就會幫它建路由。

★ 不會無限迴圈:用計時器定頻重發「最新一筆」,不是收到就轉。收到自己發的
  只會把儲存的內容覆寫成一樣的東西,不會放大。

★ 不會製造雙父節點:parent 一樣是 base_link,值也一樣,只是同一筆變換
  有兩個發布者。tf2 的動態緩衝以 (parent, child, 時間) 存,重複無害。
  真正會出事的是**不同 parent**,那跟這裡無關。

★ 為什麼不乾脆在 Jetson 跑 robot_state_publisher:它會連
  base_footprint -> base_link 和 /robot_description 一起發,跟底盤的重複。
  值一樣所以不會壞,但多兩個來源就多兩個以後會不一致的地方。只轉發缺的那部分。
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

# 只轉發這些前綴的 child frame。留成清單是因為以後可能還有別的
# 底盤節點發的動態變換(例如雲台),到時候加進來就好。
PREFIXES = ("wheel_",)
HZ = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0

QOS = QoSProfile(depth=100,
                 reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST)


class Relay(Node):
    def __init__(self):
        super().__init__("tf_relay_wheels")
        self.store = {}          # child -> TransformStamped
        self.n_in = self.n_out = 0
        self.pub = self.create_publisher(TFMessage, "/tf", QOS)
        self.create_subscription(TFMessage, "/tf", self.on_tf, QOS)
        self.create_timer(1.0 / HZ, self.tick)
        self.create_timer(30.0, self.report)
        self.get_logger().info(
            "tf_relay_wheels 啟動,轉發 %s 開頭的 frame,%.0f Hz"
            % ("/".join(PREFIXES), HZ))

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id.startswith(PREFIXES):
                self.n_in += 1
                self.store[t.child_frame_id] = t

    def tick(self):
        if not self.store:
            return
        m = TFMessage()
        # 時間戳保持原樣。改成「現在」會讓 tf2 以為有更新的資料,
        # 底盤真的斷線時反而看不出來 —— 寧可讓它照實過期。
        m.transforms = list(self.store.values())
        self.pub.publish(m)
        self.n_out += 1

    def report(self):
        if self.store:
            self.get_logger().info(
                "收到 %d 筆、轉發 %d 次,目前 %d 個 frame:%s"
                % (self.n_in, self.n_out, len(self.store),
                   ", ".join(sorted(self.store))))
        else:
            self.get_logger().warn(
                "還沒收到任何 %s 開頭的變換 —— 底盤的 robot_state_publisher "
                "有在跑嗎?(ros2 topic hz /joint_states)" % PREFIXES[0])


def main():
    rclpy.init()
    n = Relay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
