#!/usr/bin/env python3
"""底盤沒開時,由 FAST-LIO 補上 lidar_odom -> box_link。

底盤在線時這支不該跑 —— 那時 box_link 掛在底盤的 base_link 底下,
兩邊同時發 box_link 的父節點就會變成雙父節點,而 tf2 不報錯,只會在兩個
答案之間隨機翻轉。robot_tf.sh 用模式參數保證同一時間只有一個來源。

★ 我們一次都不發 base_link —— 那是樹莓派的。所以底盤上線時最多只會
  多出一條並存的分支,不會撞名。

數學:
    T(odom → box_link)(t) = T_origin⁻¹ · T(camera_init → body)(t) · T(body → box_link)

其中 T_origin 是第一筆訊息時的 T(camera_init → box_link),
也就是把 odom 的原點釘在「啟動那一刻 box_link 所在的位置」。

這樣做順便解決了扶正:box_link 是盒底安裝面,跟車體平行,車停平地時就是
水平的。把 odom 定成「啟動時 box_link 的姿態」,odom 自動就是水平的 ——
不需要另外量重力再補一個傾斜的靜態變換。

★ 為什麼扶正很重要:camera_init 是 FAST-LIO 啟動瞬間光達 IMU 的姿態,
  光達機構上斜 29.7 度,camera_init 就跟著斜。slam_toolbox 把軌跡投影到
  那個斜平面做二維定位,地上走 1 公尺只記 cos(29.7°) = 0.87 公尺,
  14.5% 的尺度誤差 —— 繞一圈回不到原點,迴路閉合必然失敗。
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

ODOM_TOPIC = "/Odometry"
# ★ 2026-08-11 從 "odom" 改名。standalone 模式理論上底盤沒開,但要是
#   底盤中途上線(publish_tf 又是 true),兩邊都叫 odom 就撞名了。
#   換個名字就永遠不會撞。slam_params.yaml 的 odom_frame 要跟著改。
ODOM_FRAME = "lidar_odom"
BASE_FRAME = "box_link"
BODY_FRAME = "body"


def q2R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R2q(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


def tf2mat(t):
    M = np.eye(4)
    q = t.transform.rotation
    M[:3, :3] = q2R(q.x, q.y, q.z, q.w)
    v = t.transform.translation
    M[:3, 3] = [v.x, v.y, v.z]
    return M


class BaseTf(Node):
    def __init__(self):
        super().__init__("fastlio_base_tf")
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.br = TransformBroadcaster(self)
        self.T_body_box = None    # box_link -> body 的反矩陣,查一次就好
        self.T_origin = None
        self.warned = False
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 20)
        self.get_logger().info("fastlio_base_tf 啟動:%s -> %s" % (ODOM_FRAME, BASE_FRAME))

    def ensure_static(self):
        if self.T_body_box is not None:
            return True
        try:
            # box_link -> body 是 robot_tf.sh 發的靜態,反過來查即可
            self.T_body_box = tf2mat(self.buf.lookup_transform(
                BODY_FRAME, BASE_FRAME, rclpy.time.Time()))
        except Exception:
            if not self.warned:
                self.warned = True
                self.get_logger().warn(
                    "查不到 %s -> %s。robot_tf.sh 的靜態變換發了嗎?"
                    % (BODY_FRAME, BASE_FRAME))
            return False
        z = self.T_body_box[2, 3]
        self.get_logger().info("靜態鏈已取得:box_link 在 body 座標系裡 z = %.4f" % z)
        return True

    def on_odom(self, msg):
        if not self.ensure_static():
            return

        T_ci_body = np.eye(4)
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        T_ci_body[:3, :3] = q2R(q.x, q.y, q.z, q.w)
        T_ci_body[:3, 3] = [p.x, p.y, p.z]

        T_ci_box = T_ci_body @ self.T_body_box
        if self.T_origin is None:
            self.T_origin = np.linalg.inv(T_ci_box)
            self.get_logger().info(
                "odom 原點鎖在啟動位置。box_link 跟車體平行,所以 odom 自動水平 —— "
                "光達那 29.7 度的傾斜到此為止,不會傳進二維投影")

        T = self.T_origin @ T_ci_box

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = BASE_FRAME
        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])
        qx, qy, qz, qw = R2q(T[:3, :3])
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.br.sendTransform(t)


def main():
    rclpy.init()
    n = BaseTf()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
