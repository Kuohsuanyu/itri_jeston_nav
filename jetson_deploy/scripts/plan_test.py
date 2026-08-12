#!/usr/bin/env python3
"""驗證 Nav2 的 ComputePathToPose 能不能用。

ros2 CLI 在這台常常探索逾時,所以直接用節點測 —— 這也是 map_server.py
之後要走的同一條路。從 TF 取現在位置,往前方 2 公尺要一條路徑。
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node("plan_test")
    buf = Buffer()
    TransformListener(buf, node)

    ac = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    print("等待 action server…", flush=True)
    if not ac.wait_for_server(timeout_sec=20.0):
        print("  ✗ compute_path_to_pose 不存在 —— planner_server 沒 active")
        return 1
    print("  ✓ action server 在")

    # 等 TF
    print("等待 map -> base_link…", flush=True)
    t0 = time.time()
    tf = None
    while time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tf = buf.lookup_transform("map", "base_link", rclpy.time.Time())
            break
        except Exception:
            pass
    if tf is None:
        print("  ✗ 拿不到 map -> base_link")
        return 1
    x = tf.transform.translation.x
    y = tf.transform.translation.y
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    print(f"  ✓ 目前位置 ({x:.2f}, {y:.2f})  朝向 {math.degrees(yaw):.0f}d")

    # 目標:正前方 2 公尺
    dist = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = x + dist * math.cos(yaw)
    goal.pose.position.y = y + dist * math.sin(yaw)
    goal.pose.orientation.w = 1.0
    print(f"目標 ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})，規劃中…", flush=True)

    g = ComputePathToPose.Goal()
    g.goal = goal
    g.use_start = False

    fut = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    if not fut.done() or fut.result() is None or not fut.result().accepted:
        print("  ✗ goal 沒被接受")
        return 1

    rf = fut.result().get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=20.0)
    if not rf.done() or rf.result() is None:
        print("  ✗ 沒拿到結果(逾時)")
        return 1

    path = rf.result().result.path
    n = len(path.poses)
    if n == 0:
        print("  ✗ 規劃出空路徑 —— 目標可能在未知區或障礙裡")
        return 1

    length = 0.0
    for i in range(n - 1):
        a = path.poses[i].pose.position
        b = path.poses[i + 1].pose.position
        length += math.dist((a.x, a.y), (b.x, b.y))
    print(f"  ✓ 規劃成功:{n} 個路徑點,長度 {length:.2f} m")
    print(f"    起點 ({path.poses[0].pose.position.x:.2f}, {path.poses[0].pose.position.y:.2f})"
          f"  終點 ({path.poses[-1].pose.position.x:.2f}, {path.poses[-1].pose.position.y:.2f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
