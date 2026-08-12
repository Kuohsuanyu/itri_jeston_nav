#!/usr/bin/env python3
"""錄下原始 /Odometry,分析軌跡取樣是否夠密、轉角是否被切掉。

直角走成斜線只有兩種可能:
  A. FAST-LIO 的位姿本身就切彎(配準跟不上轉動)
  B. 位姿是對的,只是中間有幾筆沒送到瀏覽器,線把兩個遠點直接連起來

這支繞過瀏覽器直接看原始資料:
  - 取樣間隔:正常 10Hz 就是 0.1s。出現 0.3s 以上的空洞 = 發布端就在掉幀
  - 相鄰點距離:走路 1 m/s + 10Hz 應該是 10cm 一點。出現 50cm 以上的跳距 = B
  - 轉向角速度:對照 IMU 的實際角速度,可以看出配準有沒有跟上 = A
"""
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

CSV = "/tmp/path_rec.csv"


class Rec(Node):
    def __init__(self, secs):
        super().__init__("path_rec")
        self.rows = []
        self.gz = 0.0
        self.prev = None
        self.gaps = []      # 取樣間隔
        self.steps = []     # 相鄰點距離
        self.create_subscription(Odometry, "/Odometry", self.on_odom, 50)
        self.create_subscription(Imu, "/livox/imu", self.on_imu, qos_profile_sensor_data)
        self.create_timer(float(secs), self.done)
        print(f"錄製 {secs} 秒 —— 請開始走直角路徑", flush=True)

    def on_imu(self, m):
        self.gz = m.angular_velocity.z

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.rows.append((t, p.x, p.y, p.z, yaw, self.gz))
        if self.prev:
            dt = t - self.prev[0]
            d = math.dist((p.x, p.y, p.z), self.prev[1:4])
            if dt > 1e-4:
                self.gaps.append(dt)
                self.steps.append(d)
        self.prev = (t, p.x, p.y, p.z)

    def done(self):
        with open(CSV, "w") as f:
            f.write("t,x,y,z,yaw,gyro_z\n")
            for r in self.rows:
                f.write("%.6f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % r)

        n = len(self.rows)
        if n < 3:
            print("資料太少"); raise SystemExit

        total = sum(self.steps)
        g = sorted(self.gaps)
        s = sorted(self.steps)
        big_gap = [x for x in self.gaps if x > 0.3]
        big_step = [x for x in self.steps if x > 0.5]

        print(f"\n=== 原始 /Odometry ===")
        print(f"  筆數 {n},總移動距離 {total:.2f} m,平均 {n/max(g[-1],1e-9):.0f}…")
        print(f"  取樣間隔  中位數 {g[len(g)//2]*1000:.0f} ms   最大 {g[-1]*1000:.0f} ms")
        print(f"  相鄰距離  中位數 {s[len(s)//2]*100:.1f} cm    最大 {s[-1]*100:.1f} cm")
        print(f"  >0.3s 的空洞: {len(big_gap)} 次   >50cm 的跳距: {len(big_step)} 次")
        if big_gap:
            print(f"    最大空洞 {max(big_gap):.2f} s  ← 發布端掉幀")
        print(f"\n  CSV 已存到 {CSV}")
        print("  判讀:空洞/跳距多 = 傳輸掉幀(B);都很密但畫出來仍是斜線 = 配準切彎(A)")
        raise SystemExit


def main():
    secs = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    rclpy.init()
    try:
        rclpy.spin(Rec(secs))
    except SystemExit:
        pass


if __name__ == "__main__":
    main()
