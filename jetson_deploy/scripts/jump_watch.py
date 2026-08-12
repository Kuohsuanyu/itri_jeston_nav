#!/usr/bin/env python3
"""抓 FAST-LIO 位姿的突跳。

/Odometry 是 10Hz,所以相鄰兩則之間的位移除以 dt 就是瞬時速度。
手持或推車不可能超過 3 m/s,超過的就是配準跳掉,不是真的移動。
一併記錄該時刻的 IMU 加速度大小,用來分辨是「IMU 尖峰」還是「純配準失敗」。
"""
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

JUMP_MPS = 3.0          # 超過這個瞬時速度就當突跳


class Watch(Node):
    def __init__(self, secs):
        super().__init__("jump_watch")
        self.prev = None
        self.n = 0
        self.jumps = 0
        self.max_step = 0.0
        self.acc = 0.0
        self.acc_max = 0.0
        self.create_subscription(Odometry, "/Odometry", self.on_odom, 20)
        self.create_subscription(Imu, "/livox/imu", self.on_imu, qos_profile_sensor_data)
        self.create_timer(float(secs), self.done)
        print(f"監看 {secs} 秒,門檻 {JUMP_MPS} m/s", flush=True)

    def on_imu(self, m):
        a = m.linear_acceleration
        self.acc = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        self.acc_max = max(self.acc_max, self.acc)

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        cur = (t, p.x, p.y, p.z)
        if self.prev is not None:
            dt = cur[0] - self.prev[0]
            if dt > 1e-4:
                d = math.dist(cur[1:], self.prev[1:])
                v = d / dt
                self.max_step = max(self.max_step, v)
                if v > JUMP_MPS:
                    self.jumps += 1
                    print(f"  突跳 #{self.jumps}: {d:.3f} m / {dt:.3f} s = {v:.1f} m/s"
                          f"  位置 ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
                          f"  IMU|a|={self.acc:.2f}g", flush=True)
        self.prev = cur
        self.n += 1

    def done(self):
        print(f"\n總計 {self.n} 則 /Odometry,{self.jumps} 次突跳,"
              f"最大瞬時速度 {self.max_step:.2f} m/s,IMU |a| 峰值 {self.acc_max:.2f}g",
              flush=True)
        raise SystemExit


def main():
    secs = int(sys.argv[1]) if len(sys.argv) > 1 else 120
    rclpy.init()
    try:
        rclpy.spin(Watch(secs))
    except SystemExit:
        pass


if __name__ == "__main__":
    main()
