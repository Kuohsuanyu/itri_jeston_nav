#!/usr/bin/env python3
"""在 Jetson 上算出四個輪子的 TF —— 底盤的 robot_state_publisher 靠不住。

    python3 tf_relay_wheels.py [Hz]      預設 10

── 為什麼不讓底盤自己發 ────────────────────────────────────────────
底盤的 robot_state_publisher **有在跑,而且 /tf_static 發得出來**
(base_footprint -> base_link 收得到),但它不處理 /joint_states,
所以四個 continuous 關節的 TF 一則都沒有。2026-08-14 實測:

    /joint_states     4 個輪子關節,10 Hz,角度值正常
    /tf_static        base_link, body, box_link, camera_link  —— 沒有輪子
    /tf               完全沒有 wheel_*(12 秒)

這是底盤**同機 DDS 訂閱斷掉**的老毛病,跟先前
"/chassis/motor_state is stale (11104.3s)" 是同一個病灶 ——
發布端正常、訂閱端收不到,而且不會有任何錯誤。從 Jetson 修不了。

★ 但 Jetson 自己收得到 /joint_states。所以這支不再「轉發」,而是
  **自己做 robot_state_publisher 該做的事**:讀關節角度,套上 URDF 裡的
  固定幾何,算出 base_link -> wheel_*_link。

  好處是完全不依賴底盤的內部 DDS —— 只要 /joint_states 過得來就有輪子。

── 幾何來源 ───────────────────────────────────────────────────────
chassis_description/urdf/chassis_DD-M-HH.xacro + chassis_common.xacro:

    joint origin  xyz = (±front_x, ±track_y, wheel_z)   rpy = 0
    axis          (0, ±1, 0)          -> 繞 Y 軸轉
    parent        base_link

★ 車型是 DD-M-HH(加高版)。DD-M 的 front_x 是 0.275、wheel_z 是 0,
  用錯的話輪子會偏 5 mm 並浮起來 5.7 mm。
  判斷依據:tf2_echo base_footprint base_link 回 0.209(DD-M 是 0.2032)。

── 定時發布,不要在回呼裡直接發 ───────────────────────────────────
★ 這支同時訂閱 /joint_states 又發 /tf。如果在回呼裡發,而 /joint_states
  又剛好因為橋接繞回來,就會自我觸發成無窮迴圈。用定時器隔開:
  回呼只更新狀態,定時器負責發。
"""
import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

RATE = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0

# chassis_DD-M-HH.xacro。改車型的話這裡要改。
FRONT_X, REAR_X = 0.28, -0.28
TRACK_Y = 0.31105
WHEEL_Z = -0.0056939

# 關節名 -> (x, y, z, 轉軸方向)
WHEELS = {
    "wheel_front_left_joint":  (FRONT_X,  TRACK_Y, WHEEL_Z,  1.0),
    "wheel_front_right_joint": (FRONT_X, -TRACK_Y, WHEEL_Z, -1.0),
    "wheel_rear_left_joint":   (REAR_X,   TRACK_Y, WHEEL_Z,  1.0),
    "wheel_rear_right_joint":  (REAR_X,  -TRACK_Y, WHEEL_Z, -1.0),
}
PARENT = "base_link"


def child_of(joint):
    return joint.replace("_joint", "_link")


class Wheels(Node):
    def __init__(self):
        super().__init__("tf_relay_wheels")
        self.br = TransformBroadcaster(self)
        self.ang = {j: 0.0 for j in WHEELS}
        self.got = 0
        self.warned = False

        self.create_subscription(
            JointState, "/joint_states", self.on_js,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))
        self.create_timer(1.0 / RATE, self.tick)
        self.create_timer(30.0, self.report)
        self.get_logger().info(
            "從 /joint_states 算輪子 TF,%.0f Hz(底盤的 rsp 不處理 joint_states)"
            % RATE)

    def on_js(self, msg):
        self.got += 1
        for name, pos in zip(msg.name, msg.position):
            if name in self.ang:
                self.ang[name] = float(pos)

    def report(self):
        if self.got == 0 and not self.warned:
            self.warned = True
            self.get_logger().warn(
                "還沒收到 /joint_states —— 底盤的 chassis_driver 起來了嗎?"
                "輪子會停在角度 0 的位置(位置仍然正確,只是不會轉)")
        else:
            self.get_logger().info("已處理 %d 則 /joint_states" % self.got)

    def tick(self):
        now = self.get_clock().now().to_msg()
        out = []
        for joint, (x, y, z, axis) in WHEELS.items():
            th = self.ang[joint] * axis          # 繞 Y 軸
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = PARENT
            t.child_frame_id = child_of(joint)
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = math.sin(th / 2.0)
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = math.cos(th / 2.0)
            out.append(t)
        self.br.sendTransform(out)


def main():
    rclpy.init()
    n = Wheels()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
