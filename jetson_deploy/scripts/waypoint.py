#!/usr/bin/env python3
"""具名航點 —— 記住地圖上的位置,用名字設定起點或導航目標。

── 為什麼需要 ────────────────────────────────────────────────────
這條走廊 81 公尺、兩側門洞週期性重複。實測把同一幀掃描沿走廊滑動:

    x = +34.60   分數 0.0500
    x = +23.35   分數 0.0500
    x = +20.85   分數 0.0500      三個相距十幾公尺的位置,分數一模一樣

**車停著不動時,單靠掃描在數學上就分不出在第幾個門洞。** 那不是參數
調不好,是環境本身的歧義。AMCL 是濾波器,要靠「移動時經過的門洞序列」
才收斂 —— 但每次開機都要人推著走一段才定得住,太麻煩。

所以改成:把常用的位置存起來,開機時直接指定「我在 A 點」。
一次設定,以後每次開機一行指令。

── 用法 ──────────────────────────────────────────────────────────
    python3 waypoint.py list                  列出所有航點
    python3 waypoint.py save 充電站            把**目前位置**存成航點
    python3 waypoint.py init 充電站            告訴 AMCL「我在這裡」(設起點)
    python3 waypoint.py goto 走廊東端          發 Nav2 導航目標 ★ 車會動
    python3 waypoint.py del 舊點

存檔位置:~/slam2d/waypoints.yaml(在 git repo 裡,會一起版控)

★ save 之前要先確認定位是對的:
      python3 check_localization.py     命中率 > 70%
  存到錯的位置,以後每次 init 都會錯,而且錯得很一致很難發現。
"""
import math
import os
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

WP = os.path.expanduser("~/slam2d/waypoints.yaml")
MAP_FRAME = "map"
BASE_FRAME = "base_footprint"


def load():
    if not os.path.isfile(WP):
        return {}
    with open(WP, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def save_all(d):
    with open(WP, "w", encoding="utf-8") as f:
        f.write("# 具名航點。座標是 map 座標系,yaw 是度。\n")
        f.write("# 用 waypoint.py save <名字> 新增,不建議手改。\n")
        yaml.safe_dump(d, f, allow_unicode=True, sort_keys=True,
                       default_flow_style=False)


def make_node(name):
    rclpy.init()
    return Node(name)


def current_pose(n, secs=6.0):
    """查 map -> base_footprint。回傳 (x, y, yaw度) 或 None。"""
    buf = Buffer()
    TransformListener(buf, n)
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            t = buf.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
        except Exception:
            continue
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y,
                math.degrees(yaw))
    return None


def cmd_list(_):
    d = load()
    if not d:
        print("還沒有任何航點。先確認定位正確,再:")
        print("    python3 waypoint.py save <名字>")
        return 0
    print("%-16s %10s %10s %8s   %s" % ("名字", "x", "y", "yaw", "備註"))
    for k in sorted(d):
        v = d[k]
        print("%-16s %10.2f %10.2f %7.1f°   %s"
              % (k, v["x"], v["y"], v["yaw_deg"], v.get("note", "")))
    print("\n存放於 %s" % WP)
    return 0


def cmd_save(args):
    if not args:
        print("要給名字:waypoint.py save <名字> [備註]")
        return 1
    name = args[0]
    note = " ".join(args[1:]) if len(args) > 1 else ""
    n = make_node("wp_save")
    p = current_pose(n)
    if p is None:
        print("✗ 查不到 %s -> %s" % (MAP_FRAME, BASE_FRAME))
        print("   AMCL 有沒有設初始位置?定位起來了嗎?")
        return 1
    x, y, yaw = p
    d = load()
    if name in d:
        old = d[name]
        print("覆蓋既有航點 %s:(%.2f, %.2f, %.1f°) -> (%.2f, %.2f, %.1f°)"
              % (name, old["x"], old["y"], old["yaw_deg"], x, y, yaw))
    d[name] = {"x": round(x, 3), "y": round(y, 3),
               "yaw_deg": round(yaw, 2), "note": note}
    save_all(d)
    print("已存 %s = (%.2f, %.2f) %.1f°" % (name, x, y, yaw))
    print("\n★ 存之前有確認定位是對的嗎?")
    print("   python3 ~/slam2d/check_localization.py     命中率要 > 70%")
    return 0


def cmd_init(args):
    if not args:
        print("要給名字:waypoint.py init <名字>")
        return 1
    d = load()
    if args[0] not in d:
        print("✗ 沒有這個航點:%s" % args[0])
        return cmd_list(None)
    v = d[args[0]]
    n = make_node("wp_init")
    # ★ RELIABLE。AMCL 的 /initialpose 訂閱是 RELIABLE,用 BEST_EFFORT
    #   發的話配不上,訊息會安靜地被丟掉。
    pub = n.create_publisher(
        PoseWithCovarianceStamped, "/initialpose",
        QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST))
    m = PoseWithCovarianceStamped()
    m.header.frame_id = MAP_FRAME
    m.pose.pose.position.x = float(v["x"])
    m.pose.pose.position.y = float(v["y"])
    th = math.radians(v["yaw_deg"])
    m.pose.pose.orientation.z = math.sin(th / 2)
    m.pose.pose.orientation.w = math.cos(th / 2)
    # 共變異數給小一點:這是「我確定我在這裡」,不是「大概在附近」。
    # 給大的話粒子會散開,在這種週期性走廊反而會飄到隔壁門洞。
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.10      # x/y 標準差約 0.32 m
    cov[35] = 0.03              # yaw 標準差約 10 度
    m.pose.covariance = cov
    for _ in range(6):
        m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.2)
    print("已告訴 AMCL:目前在 %s = (%.2f, %.2f) %.1f°"
          % (args[0], v["x"], v["y"], v["yaw_deg"]))
    print("\n驗證:python3 ~/slam2d/check_localization.py")
    return 0


def cmd_goto(args):
    if not args:
        print("要給名字:waypoint.py goto <名字>")
        return 1
    d = load()
    if args[0] not in d:
        print("✗ 沒有這個航點:%s" % args[0])
        return cmd_list(None)
    v = d[args[0]]
    print("★ 這會讓車**真的移動**到 %s = (%.2f, %.2f) %.1f°"
          % (args[0], v["x"], v["y"], v["yaw_deg"]))
    print("  Nav2 要在跑,而且 twist_mux 的輸出要接到 /cmd_vel")
    print("  人要在旁邊,手放在遙控介面的「停」上")
    try:
        if input("  確定?打 yes:") != "yes":
            print("  取消")
            return 0
    except EOFError:
        print("  非互動模式,取消。要跳過確認請直接發 /goal_pose")
        return 0
    n = make_node("wp_goto")
    pub = n.create_publisher(
        PoseStamped, "/goal_pose",
        QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST))
    m = PoseStamped()
    m.header.frame_id = MAP_FRAME
    m.pose.position.x = float(v["x"])
    m.pose.position.y = float(v["y"])
    th = math.radians(v["yaw_deg"])
    m.pose.orientation.z = math.sin(th / 2)
    m.pose.orientation.w = math.cos(th / 2)
    for _ in range(3):
        m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.3)
    print("目標已送出")
    return 0


def cmd_del(args):
    if not args:
        print("要給名字")
        return 1
    d = load()
    if args[0] not in d:
        print("沒有這個航點:%s" % args[0])
        return 1
    del d[args[0]]
    save_all(d)
    print("已刪除 %s" % args[0])
    return 0


def main():
    cmds = {"list": cmd_list, "save": cmd_save, "init": cmd_init,
            "goto": cmd_goto, "del": cmd_del}
    if len(sys.argv) < 2 or sys.argv[1] not in cmds:
        print(__doc__)
        return 1
    return cmds[sys.argv[1]](sys.argv[2:])


if __name__ == "__main__":
    sys.exit(main())
