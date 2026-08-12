#!/usr/bin/env python3
"""把 FAST-LIO 的位姿轉發成 odom -> base_footprint。

為什麼需要這支:
  改用 URDF 之後,robot_state_publisher 會發 base_footprint -> base_link -> body。
  而 FAST-LIO 自己也發 camera_init -> body。這樣 body 就有**兩個父節點**,
  違反 TF 的單一父節點規則。tf2 不會報錯 —— 它會交替採用兩者,
  結果是整棵樹以 10 Hz 抖動,症狀看起來像里程計在飄,非常難查。

  正確做法是讓 URDF 成為唯一的機構真實來源,里程計只負責 odom -> base_footprint:

      odom ──(這支)── base_footprint ──(RSP/URDF)── base_link ──┬── body
                                                                └── camera_link

  所以 FAST-LIO 自己的 TF 必須關掉。它沒有參數可以關,用 launch 的 remap
  把 /tf 導到別的名字即可:

      ros2 launch fast_lio mapping.launch.py ... \
        --ros-args -r /tf:=/tf_fastlio_unused

數學:
  FAST-LIO 給的是 body 在 camera_init 裡的位姿 T_ci_body(t)。
  URDF 給的是 T_body_bf(固定)。
  第一筆訊息時記下 T_ci_bf(0),把它當成 odom 的原點:

      T_odom_bf(t) = T_ci_bf(0)^-1 . T_ci_body(t) . T_body_bf

  這樣 odom 自動落在「出發時 base_footprint 所在的位置」——
  而 base_footprint 依定義就在地面上、而且是水平的,
  所以 **odom 自動就是水平的**,不需要另外用 IMU 去扶正。

  這一點很重要:之前的 odom -> camera_init identity 讓 odom 跟著光達斜 30 度,
  slam_toolbox 把軌跡投影到那個斜掉的 x-y 平面,造成約 13% 的尺度誤差。
  改成這個做法之後,水平性是從 URDF 的 LIDAR_PITCH 推導出來的,
  只要那個角度是量準的,odom 就是準的。
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

ODOM_TOPIC = "/Odometry"
ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"
BODY_FRAME = "body"


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
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


def tf_to_mat(t):
    M = np.eye(4)
    q = t.transform.rotation
    M[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    v = t.transform.translation
    M[:3, 3] = [v.x, v.y, v.z]
    return M


class OdomTf(Node):
    def __init__(self):
        super().__init__("fastlio_odom_tf")
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.br = TransformBroadcaster(self)
        self.T_body_bf = None      # URDF 給的,固定
        self.T_origin = None       # T_ci_bf(0) 的反矩陣
        self.warned = False
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 20)
        self.get_logger().info("fastlio_odom_tf 啟動:%s -> %s" % (ODOM_FRAME, BASE_FRAME))

    def ensure_static(self):
        if self.T_body_bf is not None:
            return True
        try:
            self.T_body_bf = tf_to_mat(self.buf.lookup_transform(
                BODY_FRAME, BASE_FRAME, rclpy.time.Time()))
        except Exception:
            if not self.warned:
                self.warned = True
                self.get_logger().warn(
                    "查不到 %s -> %s。robot_state_publisher 有在跑嗎?"
                    % (BODY_FRAME, BASE_FRAME))
            return False
        z = self.T_body_bf[2, 3]
        self.get_logger().info(
            "URDF: base_footprint 在 body 座標系裡的 z = %.4f m(光達離地高度)" % (-z))
        return True

    def on_odom(self, msg):
        if not self.ensure_static():
            return

        T_ci_body = np.eye(4)
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        T_ci_body[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        T_ci_body[:3, 3] = [p.x, p.y, p.z]

        T_ci_bf = T_ci_body @ self.T_body_bf
        if self.T_origin is None:
            self.T_origin = np.linalg.inv(T_ci_bf)
            self.get_logger().info("odom 原點鎖定在出發位置(地面、水平)")

        T = self.T_origin @ T_ci_bf

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = BASE_FRAME
        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])
        qx, qy, qz, qw = R_to_quat(T[:3, :3])
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.br.sendTransform(t)


def main():
    rclpy.init()
    n = OdomTf()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
