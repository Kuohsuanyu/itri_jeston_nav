#!/usr/bin/env python3
"""把兩個里程計整理成 EKF 能用的形式。

  /odom      底盤輪速   odom → base_footprint      ─┐
                                                    ├─→ 這支 ─→ /odom_wheel_cov
  /Odometry  FAST-LIO   camera_init → body         ─┘         ─→ /odom_lidar_cov
                                                                        │
                                                                        ▼
                                                        ekf_node → multi_odom → base_footprint

做三件事,每一件都是 EKF 沒辦法自己處理的:

1. 補共變異數
   兩邊發出來的共變異數全是 0。EKF 把 0 解讀成「這個測量完全確定」,結果是
   後到的來源永遠蓋掉前一個 —— 那不是融合,是輪流覆寫。
   FAST-LIO 的 1e-7(標準差 0.3 mm)同樣不真實,會讓它完全壓過輪速。

2. 換算到 base_footprint
   EKF 估的是 base_footprint(= 底盤原本 odom 的子節點,我們現在要取代它)。
   輪速本來報的就是 base_footprint,不用動;光達報的是 body,
   相對車體有 2.5 cm 橫向偏移**而且傾斜 29.7 度**,所以它的 yaw 根本不是
   車體的 yaw —— 不換算的話轉彎時兩路會嚴重打架。

   ★ 2026-08-11 從 box_link 改成 base_footprint:樹莓派設 publish_tf: false
     之後,odom -> base_footprint 那一段空出來由 EKF 接手,兩棵樹併成一棵。
     附帶好處是輪速這一路變成恆等變換,少一次矩陣乘法也少一個出錯的地方。

3. 統一成 differential
   camera_init 和 odom 是兩個不同原點的世界,絕對位姿差一個常數偏移。
   兩邊都設 differential,EKF 只吃位姿的變化量,偏移就不影響。
   (這支不做 differential,是在 ekf 設定裡設;這裡只要保證兩邊的
    child_frame_id 都是 base_footprint,EKF 才不會再去查 TF ——
    這很重要,因為 camera_init 根本不在 TF 樹上,查得到才有鬼。)
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# ---- 共變異數(標準差,不是變異數;下面會平方)-------------------------
# 輪速:氣胎 + 差動輪,轉彎打滑是主要誤差來源。每公尺 2%、每弧度 5%。
WHEEL_SIGMA_XY = 0.05      # m
WHEEL_SIGMA_YAW = 0.06     # rad
# 光達:FAST-LIO 實測靜止漂移 0.015 m、逐幀跳動中位 0.005 m。
# 比輪速可信約一個數量級 —— EKF 會以它為主,輪速當高頻補間。
LIDAR_SIGMA_XY = 0.02      # m
LIDAR_SIGMA_YAW = 0.01     # rad

# ---- 外參:直接從 robot_tf.sh 讀 ----------------------------------------
# ★ 原本這裡是手抄的常數,註解寫著「改那邊就要改這邊」—— 然後 2026-08-13
#   改了 robot_tf.sh 卻漏了這裡,舊值 BOX_X=0.0 / BOX_Z=0.3705 / AXLE_Z=0.2032
#   繼續用了一整天。X 差 13 公分是**旋轉時會放大的槓桿臂誤差**:車原地轉一圈,
#   換算出來的 base_footprint 位置會畫一個半徑 13 公分的圓。
#
#   而且不會有任何錯誤訊息 —— 兩份都是合法的數字。
#
#   所以改成直接讀 robot_tf.sh。那支是唯一正本,這裡不再有第二份。
import os
import re

_TF_SH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_tf.sh")


def _read_extrinsics(path):
    """從 robot_tf.sh 撈 BOX_* / BODY_* 常數。"""
    want = ["BOX_X", "BOX_Y", "BOX_Z", "BOX_YAW",
            "BODY_X", "BODY_Y", "BODY_Z", "BODY_ROLL", "BODY_PITCH", "BODY_YAW"]
    with open(path, encoding="utf-8") as f:
        txt = f.read()
    got = {m.group(1): float(m.group(2))
           for m in re.finditer(r"^([A-Z_]+)=(-?[0-9.]+)\s*$", txt, re.M)}
    missing = [k for k in want if k not in got]
    if missing:
        raise SystemExit("robot_tf.sh 裡找不到:%s" % ", ".join(missing))
    return got


_E = _read_extrinsics(_TF_SH)
BOX_X, BOX_Y, BOX_Z, BOX_YAW = (_E["BOX_X"], _E["BOX_Y"], _E["BOX_Z"], _E["BOX_YAW"])
BODY = (_E["BODY_X"], _E["BODY_Y"], _E["BODY_Z"],
        _E["BODY_ROLL"], _E["BODY_PITCH"], _E["BODY_YAW"])

# base_footprint → base_link = 輪半徑 - wheel_z。
# ★ 實機是 DD-M-HH(加高版):wheel_radius 0.2032,wheel_z -0.0056939 -> 0.2089。
#   DD-M 是 0.2032。差 5.7 mm。實機 tf2_echo base_footprint base_link 回 0.209。
#   這個值在 chassis_description/urdf/chassis_DD-M-HH.xacro,不在 robot_tf.sh,
#   所以只能寫在這裡 —— 換車型的話要改。
AXLE_Z = 0.2089


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr]])


def q_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_q(R):
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


def mat(xyz, rpy):
    M = np.eye(4)
    M[:3, :3] = rpy_to_R(*rpy)
    M[:3, 3] = xyz
    return M


# base_footprint → box_link:先上到輪軸,再上到盒底
T_BF_BOX = mat([0, 0, AXLE_Z], [0, 0, 0]) @ mat([BOX_X, BOX_Y, BOX_Z], [0, 0, BOX_YAW])
T_BOX_BODY = mat(BODY[0:3], BODY[3:6])
# body → base_footprint:整條鏈反過來
T_BODY_BF = np.linalg.inv(T_BF_BOX @ T_BOX_BODY)
# 輪速報的就是 base_footprint,不用換算
T_IDENT = np.eye(4)


def stamp_sec(s):
    """ROS 時間戳 -> 秒(float)。用訊息自己的時間,不要用牆鐘 ——
    牆鐘會把網路抖動算進 dt,微分出來的速度會亂跳。"""
    return s.sec + s.nanosec * 1e-9


def wrap_pi(a):
    """把角度差收進 (-pi, pi]。不做的話從 +179 度轉到 -179 度會被當成
    轉了 358 度,微分出來的角速度會爆掉。"""
    return (a + math.pi) % (2 * math.pi) - math.pi


def cov6(sxy, syaw):
    """六自由度共變異數矩陣(row-major 36 個值)。

    z / roll / pitch 給很大的值 —— 這台車在平地上跑,two_d_mode 會把它們
    鎖死,但填大一點可以避免 EKF 在數值上鑽牛角尖。
    """
    d = [sxy ** 2, sxy ** 2, 1e6, 1e6, 1e6, syaw ** 2]
    C = [0.0] * 36
    for i in range(6):
        C[i * 7] = d[i]
    return C


class Relay(Node):
    def __init__(self):
        super().__init__("odom_cov_relay")
        q = QoSProfile(depth=20)
        q.reliability = ReliabilityPolicy.RELIABLE

        self.pub_w = self.create_publisher(Odometry, "/odom_wheel_cov", q)
        self.pub_l = self.create_publisher(Odometry, "/odom_lidar_cov", q)
        self.create_subscription(Odometry, "/odom", self.on_wheel, q)
        self.create_subscription(Odometry, "/Odometry", self.on_lidar, q)

        self.n_w = self.n_l = 0
        self.prev = {}          # key -> (時間, x, y, yaw),用來微分算速度
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            "odom_cov_relay 啟動:/odom -> /odom_wheel_cov,/Odometry -> /odom_lidar_cov "
            "(共變異數 輪速 %.3f m / 光達 %.3f m)" % (WHEEL_SIGMA_XY, LIDAR_SIGMA_XY))

    def report(self):
        self.get_logger().info("轉發計數 輪速 %d  光達 %d" % (self.n_w, self.n_l))

    def _emit(self, src, T_src_bf, pub, sxy, syaw, key):
        """把 src 報的位姿換算成 base_footprint 的位姿,補上共變異數再發出去。

        ★ 2026-08-13 起也**自己微分算出速度**。原因見 twist 那一段。
        """
        p = src.pose.pose.position
        o = src.pose.pose.orientation
        T = np.eye(4)
        T[:3, :3] = q_to_R(o.x, o.y, o.z, o.w)
        T[:3, 3] = [p.x, p.y, p.z]
        T = T @ T_src_bf                       # 世界 → base_footprint

        out = Odometry()
        out.header = src.header
        out.child_frame_id = "base_footprint"  # 換算過了,EKF 不用再查 TF
        out.pose.pose.position.x = float(T[0, 3])
        out.pose.pose.position.y = float(T[1, 3])
        out.pose.pose.position.z = float(T[2, 3])
        qx, qy, qz, qw = R_to_q(T[:3, :3])
        out.pose.pose.orientation.x = float(qx)
        out.pose.pose.orientation.y = float(qy)
        out.pose.pose.orientation.z = float(qz)
        out.pose.pose.orientation.w = float(qw)
        out.pose.covariance = cov6(sxy, syaw)

        # ── twist:自己微分算 ───────────────────────────────────────
        # FAST-LIO 的 /Odometry 不填 twist(實測 linear.x 恆為 0),底盤的
        # /odom 也一樣。原本就直接放棄不轉發。
        #
        # ★ 那會讓 EKF 的**速度狀態完全沒有觀測**。robot_localization 的
        #   15 維狀態裡有 vx/vy/vyaw,只餵位姿的話它們只能在過程雜訊下
        #   隨機游走,而位置又是速度的積分 —— 於是車停著位置照樣飄。
        #
        #   2026-08-13 實測(車靜止 45 秒):
        #       FAST-LIO /Odometry           漂移  0.13 mm/s   角度  0.2 度/分
        #       EKF multi_odom->base         漂移 16.92 mm/s   角度 -33.4 度/分
        #   EKF 的漂移是輸入的 130 倍(位置)/ 167 倍(角度)。不是感測器爛,
        #   是濾波器的速度狀態沒人管。
        #
        #   先前輪速還在時沒這麼明顯,因為那個(壞掉的)輸入一直說「沒動」,
        #   剛好當了速度為零的錨。把它停用之後問題就浮出來了。
        #
        # 速度要在 **child_frame(base_footprint)** 座標系,也就是車體固定
        # 座標系 —— 不是世界座標系。所以世界系的位移要轉回車體系。
        now = stamp_sec(src.header.stamp)
        prev = self.prev.get(key)
        vx = vy = vyaw = 0.0
        ok = False
        if prev is not None:
            dt = now - prev[0]
            # dt 太小除出來會爆,太大代表中間掉幀、微分沒有意義
            if 1e-3 < dt < 0.5:
                dx = float(T[0, 3]) - prev[1]
                dy = float(T[1, 3]) - prev[2]
                th = math.atan2(T[1, 0], T[0, 0])
                dth = wrap_pi(th - prev[3])
                c, s = math.cos(th), math.sin(th)
                vx = (c * dx + s * dy) / dt      # 世界系 → 車體系
                vy = (-s * dx + c * dy) / dt
                vyaw = dth / dt
                ok = True
        self.prev[key] = (now, float(T[0, 3]), float(T[1, 3]),
                          math.atan2(T[1, 0], T[0, 0]))

        if ok:
            out.twist.twist.linear.x = vx
            out.twist.twist.linear.y = vy
            out.twist.twist.angular.z = vyaw
            # 微分會放大位姿雜訊:sigma_v ≈ sqrt(2) * sigma_pose / dt。
            # 用實際的 dt 算,不要寫死 —— 掉幀時 dt 變大,速度反而更可信。
            sv = 1.414 * sxy / max(dt, 1e-3)
            sw = 1.414 * syaw / max(dt, 1e-3)
            out.twist.covariance = cov6(sv, sw)
        else:
            # 第一幀或掉幀:標成完全不可信,EKF 會略過
            out.twist.covariance = cov6(1e3, 1e3)
        pub.publish(out)

    def on_wheel(self, msg):
        self.n_w += 1
        self._emit(msg, T_IDENT, self.pub_w, WHEEL_SIGMA_XY, WHEEL_SIGMA_YAW, "w")

    def on_lidar(self, msg):
        self.n_l += 1
        self._emit(msg, T_BODY_BF, self.pub_l, LIDAR_SIGMA_XY, LIDAR_SIGMA_YAW, "l")


def main():
    rclpy.init()
    n = Relay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
