#!/usr/bin/env python3
"""乾跑模式的橋接:收 /goal_pose,叫規劃器算路徑,發到 /plan 給 RViz 畫。

    python3 goal_to_plan.py

── 為什麼需要 ──────────────────────────────────────────────────
RViz 的「2D Goal Pose」只是把目標發到 /goal_pose,它自己不會叫任何人規劃。
完整的 Nav2 是 bt_navigator 訂閱 /goal_pose,然後跑行為樹:
    ComputePathToPose -> FollowPath -> 車就開走了

乾跑模式刻意不起 bt_navigator 和 controller_server,這樣就算算出路徑也
**沒有任何節點會發 /cmd_vel** —— 安全上是實質隔離,不是靠設定壓速度。

代價是沒有人接 /goal_pose,點了完全沒反應。這支補上中間那一段:
只呼叫 ComputePathToPose,拿到路徑就發到 /plan,絕不碰 /cmd_vel。

── 為什麼發到 /plan ────────────────────────────────────────────
完整模式時 planner_server 本來就把路徑發在 /plan。用同一個 topic,
RViz 的 Path 顯示器兩種模式都不用改設定。
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class GoalToPlan(Node):
    def __init__(self):
        super().__init__("goal_to_plan")
        self.cli = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.pub = self.create_publisher(Path, "/plan", 10)
        # RViz 的 SetGoal 工具用 RELIABLE 發,QoS 配不上的話訊息會安靜地掉。
        self.create_subscription(
            PoseStamped, "/goal_pose", self.on_goal,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))
        self.busy = False
        self.get_logger().info(
            "乾跑橋接啟動:/goal_pose -> ComputePathToPose -> /plan(不碰 /cmd_vel)")

    def on_goal(self, msg):
        if self.busy:
            self.get_logger().warn("上一個還在算,忽略這次")
            return
        if not self.cli.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                "compute_path_to_pose 沒有回應 —— planner_server 起來了嗎?"
                "lifecycle 是 active 嗎?")
            return
        self.busy = True
        self.get_logger().info("目標 (%.2f, %.2f) frame=%s"
                               % (msg.pose.position.x, msg.pose.position.y,
                                  msg.header.frame_id))
        g = ComputePathToPose.Goal()
        g.goal = msg
        g.use_start = False          # 起點用現在的 TF,不要自己填
        self.cli.send_goal_async(g).add_done_callback(self.on_accepted)

    def on_accepted(self, fut):
        h = fut.result()
        if h is None or not h.accepted:
            self.get_logger().error("目標被拒絕")
            self.busy = False
            return
        h.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        self.busy = False
        res = fut.result()
        if res is None or not res.result.path.poses:
            self.get_logger().warn(
                "規劃出空路徑 —— 目標可能在障礙裡、地圖外,或起點被膨脹層蓋住")
            return
        p = res.result.path
        d = sum(math.dist((p.poses[i].pose.position.x, p.poses[i].pose.position.y),
                          (p.poses[i + 1].pose.position.x, p.poses[i + 1].pose.position.y))
                for i in range(len(p.poses) - 1))
        # 直線距離拿來對照:繞路太多通常表示地圖有破洞或被膨脹層擋住
        s, e = p.poses[0].pose.position, p.poses[-1].pose.position
        straight = math.dist((s.x, s.y), (e.x, e.y))
        ratio = d / straight if straight > 0.1 else 0
        self.get_logger().info(
            "路徑 %d 點,長 %.2f m(直線 %.2f m,繞路 %.1f 倍),0.10 m/s 約 %.0f 秒"
            % (len(p.poses), d, straight, ratio, d / 0.10))
        if ratio > 2.5:
            self.get_logger().warn(
                "繞路超過 2.5 倍 —— 直線方向可能被擋住,看看地圖那段是不是有破洞")
        self.pub.publish(p)


def main():
    rclpy.init()
    n = GoalToPlan()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
