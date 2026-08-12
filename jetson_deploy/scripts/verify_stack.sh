#!/usr/bin/env bash
# 對照 jetson_deploy/README.md 的架構,逐項驗證實機。
#
# ── 為什麼要有這支 ────────────────────────────────────────────────
# 這個專案踩過太多次「看起來對但實際不對」:
#
#   pgrep 全綠但 /scan 完全沒資料              行程活著 ≠ 有在動
#   ros2 param set 成功但行為沒變              節點只在建構時讀一次
#   設定檔改好了但跑著的行程是一小時前啟動的      存檔 ≠ 生效
#   ros2 topic list 列得出來但 echo 收不到      daemon 快取過期
#   ros2 topic echo /tf --once 抓不到底盤的     /tf 有多個發布者,--once 只抓一則
#
# 所以這支一律用「實際收到多少則訊息」當判準,不看行程、不看設定檔。
#
# 用法:
#     bash ~/slam2d/verify_stack.sh
#     bash ~/slam2d/verify_stack.sh 20      每項取樣 20 秒(預設 12)
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash 2>/dev/null
source ~/chassis_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0

SEC="${1:-12}"
PASS=0; FAIL=0
ok()   { PASS=$((PASS+1)); printf "  \033[32m✓\033[0m %s\n" "$1"; }
bad()  { FAIL=$((FAIL+1)); printf "  \033[31m✗\033[0m %s\n" "$1"; }

echo "════════ 1. TF 樹 ════════"
python3 - "$SEC" <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, HistoryPolicy,
                       ReliabilityPolicy)
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

SEC = float(sys.argv[1])
rclpy.init(); n = Node("verify_tf"); buf = Buffer(); TransformListener(buf, n)
child, rate = {}, {}


def cb(m, dyn):
    for t in m.transforms:
        child.setdefault(t.child_frame_id, set()).add(t.header.frame_id)
        if dyn:
            rate[t.child_frame_id] = rate.get(t.child_frame_id, 0) + 1


# /tf_static 是 TRANSIENT_LOCAL,/tf 是 VOLATILE。QoS 給錯就收不到而且不報錯。
n.create_subscription(
    TFMessage, "/tf_static", lambda m: cb(m, False),
    QoSProfile(depth=200, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST))
n.create_subscription(
    TFMessage, "/tf", lambda m: cb(m, True),
    QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.VOLATILE,
               history=HistoryPolicy.KEEP_LAST))

t0 = time.time()
while time.time() - t0 < SEC:
    rclpy.spin_once(n, timeout_sec=0.1)
dt = time.time() - t0

dup = {c: sorted(p) for c, p in child.items() if len(p) > 1}
roots = sorted(set(p for ps in child.values() for p in ps) - set(child))

print("  樹:")
for c, ps in sorted(child.items()):
    hz = rate.get(c, 0) / dt
    tag = "  %.1f Hz" % hz if hz > 0.2 else "  static"
    print("    %-28s <- %-16s%s" % (c, ", ".join(sorted(ps)), tag))
print()
if dup:
    print("  \033[31m✗\033[0m 雙父節點:%s" % dup)
    print("      tf2 不會報錯,它會在兩個答案之間隨機翻轉 —— "
          "症狀看起來像里程計在飄")
else:
    print("  \033[32m✓\033[0m 沒有雙父節點")
print("  %s 樹根 %s" % ("\033[32m✓\033[0m" if len(roots) == 1 else "\033[31m✗\033[0m", roots))

# README 說的那條鏈,每一段都要查得到
print()
print("  座標查詢:")
for a, b in [("map", "base_footprint"), ("map", "base_link"),
             ("map", "box_link"), ("map", "body"), ("map", "camera_link"),
             ("map", "wheel_front_left_link")]:
    try:
        t = buf.lookup_transform(a, b, rclpy.time.Time())
        v = t.transform.translation
        print("    \033[32m✓\033[0m %-8s -> %-24s (%+.3f, %+.3f, %+.3f)"
              % (a, b, v.x, v.y, v.z))
    except Exception as e:
        print("    \033[31m✗\033[0m %-8s -> %-24s %s" % (a, b, str(e)[:40]))
PY

echo
echo "════════ 2. 資料流 ════════"
# 期望頻率。低於 60% 就當不合格 —— WiFi 抖動容得下,整條斷掉容不下。
check_hz() {   # check_hz <topic> <期望Hz> <說明>
    local t=$1 want=$2 desc=$3
    local got
    got=$(timeout $((SEC > 10 ? 10 : SEC)) ros2 topic hz "$t" 2>&1 \
          | grep -oE "average rate: [0-9.]+" | head -1 | grep -oE "[0-9.]+")
    if [ -z "$got" ]; then
        bad "$(printf '%-26s 沒有資料   %s' "$t" "$desc")"
    elif [ "$(echo "$got >= $want * 0.6" | bc -l 2>/dev/null)" = "1" ]; then
        ok "$(printf '%-26s %6.2f Hz  (期望 ~%s)' "$t" "$got" "$want")"
    else
        bad "$(printf '%-26s %6.2f Hz  太低,期望 ~%s   %s' "$t" "$got" "$want" "$desc")"
    fi
}

echo "  ── 光達鏈 ──"
check_hz /livox/imu            200 "Mid-360 的 IMU"
check_hz /cloud_registered_body 10 "FAST-LIO 輸出,/scan 的來源"
check_hz /Odometry              10 "FAST-LIO 位姿,EKF 的光達那一路"
check_hz /scan                  10 "SLAM 和 Nav2 真正看到的東西"

echo "  ── 底盤 ──"
check_hz /odom                  10 "輪速里程計,EKF 的另一路"
check_hz /joint_states          10 "輪子角度,rsp 靠它算輪子的 TF"
check_hz /battery_state         10 "電量"

echo "  ── 融合與建圖 ──"
check_hz /odom_wheel_cov        10 "odom_cov_relay 補過共變異數的輪速"
check_hz /odom_lidar_cov        10 "同上,光達"
check_hz /odometry/filtered     30 "EKF 融合結果"

echo
echo "  ── /map(latched,車停著不重發,所以不量頻率)──"
if timeout 15 ros2 topic echo /map --once --field info > /tmp/_m 2>/dev/null \
   && grep -q width /tmp/_m; then
    ok "/map  $(grep -E '^(width|height|resolution)' /tmp/_m | tr '\n' ' ')"
else
    bad "/map 收不到 —— slam_toolbox 沒起來,或還沒建出地圖"
fi

echo
echo "════════ 3. 跑著的行程用的是哪份設定 ════════"
# ★ 查行程不查檔案。2026-08-11 就是設定檔改了但行程沒重啟,
#   slam_toolbox 用著舊的 base_frame 跑了一個多小時。
p() {   # p <節點> <參數> <期望值>
    local got
    got=$(timeout 10 ros2 param get "$1" "$2" 2>/dev/null | grep -oE "[^ ]+$")
    if [ "$got" = "$3" ]; then ok "$(printf '%-22s %-16s = %s' "$1" "$2" "$got")"
    else bad "$(printf '%-22s %-16s = %s   期望 %s' "$1" "$2" "${got:-查不到}" "$3")"; fi
}
p /slam_toolbox   base_frame   base_footprint
p /slam_toolbox   odom_frame   multi_odom
p /ekf_filter_node base_link_frame base_footprint
p /ekf_filter_node world_frame  multi_odom
p /chassis_driver publish_tf   False

echo
echo "════════ 4. FAST-LIO 健康度 ════════"
# 發散時 /Odometry 會跑到幾千公里外,而且 log 會刷 "No Effective Points"
SYNC=$(grep -ic "not Synced" /tmp/fastlio.log 2>/dev/null || echo 0)
NOEF=$(grep -ic "No Effective" /tmp/fastlio.log 2>/dev/null || echo 0)
[ "$SYNC" = "0" ] && ok "IMU/光達同步警告 0 次" \
    || bad "IMU/光達不同步 $SYNC 次 —— CPU 跟不上,IMU 佇列積壓,狀態會發散"
[ "$NOEF" = "0" ] && ok "No Effective Points 0 次" \
    || bad "No Effective Points $NOEF 次"

POS=$(timeout 12 ros2 topic echo /Odometry --once --field pose.pose.position 2>/dev/null \
      | grep -oE "^[xyz]: -?[0-9.e+-]+" | head -3 | tr '\n' ' ')
echo "  位姿 $POS"
BIG=$(echo "$POS" | grep -oE "[0-9]+\.[0-9]+" | sort -rn | head -1)
[ -n "$BIG" ] && [ "$(echo "$BIG < 1000" | bc -l 2>/dev/null)" = "1" ] \
    && ok "數值合理" || bad "數值異常 —— FAST-LIO 可能已發散,重啟 startall.sh"

echo
echo "════════ 結果 ════════"
printf "  通過 %d   失敗 %d\n" "$PASS" "$FAIL"
[ "$FAIL" -eq 0 ] && echo "  架構跟 README 一致" \
    || echo "  ★ 有項目不符,對照 jetson_deploy/README.md 的「改了什麼就要重啟什麼」"
exit $([ "$FAIL" -eq 0 ] && echo 0 || echo 1)
