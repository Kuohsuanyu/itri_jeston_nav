#!/usr/bin/env bash
# 感測器座標樹 + 里程計融合。
#
#   bash sensor_tree.sh fused     輪速 + 光達,EKF 融合  (預設)
#   bash sensor_tree.sh lidar     只有光達,底盤沒開也能跑
#
# ── 為什麼要有 box_link ──────────────────────────────────────────
# 感測器鎖在理線盒裡,不是鎖在車上。盒子在車上挪 2 公分,光達和相機就一起
# 挪 2 公分。把「盒子在車上哪裡」和「感測器在盒子裡哪裡」拆成兩段:
#
#     base_link ──(上車量,會變)── box_link ──(已量好,剛性不變)── body
#                                                              └── camera_link
#
# calib_box.py 本來就是以**盒底安裝面**為基準量的,所以 box_link 定在盒底
# 中心 —— 量到什麼就存什麼,不做多餘換算。
#
# ── 為什麼座標系全部改名 ─────────────────────────────────────────
# 樹莓派的 bringup 同時發 odom -> base_footprint 和 base_footprint -> base_link,
# 而且改不了。任何撞到那些名字的發布者都會讓某個座標系有兩個父節點 ——
# tf2 **不報錯**,只會在兩個答案之間隨機翻轉。實測同一個查詢隔五分鐘得到
# x=+0.831 和 x=+0.202。用自己的名字就從架構上避開了。
#
#     底盤那棵(它發它的,我們只讀 /odom 的數值,不查它的 TF):
#         odom → base_footprint → base_link → 四個輪子
#
#     我們這棵:
#         map → multi_odom → box_link → body / camera_link
#
# ── FAST-LIO 的 /tf 必須關掉 ─────────────────────────────────────
# 它會發 camera_init -> body,那會讓 body 多一個父節點。startall.sh 用
# -r /tf:=/tf_fastlio_unused 把它靜音。它的位姿改從 /Odometry 進 EKF。
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

MODE="${1:-fused}"
D=~/slam2d

# ============ 盒底 -> 光達(calib_box.py 2026-08-10 實測)============
# Z 0.2005 就是「光達離盒底」的量測值,不是疊出來的。
# pitch 0.5181 = 29.69 度,IMU 重力 / 地面法線 / 相機 pitch 三者交叉驗證過。
BODY_X=0.0250
BODY_Y=-0.0000
BODY_Z=0.2005
BODY_ROLL=0.0112
BODY_PITCH=0.5181
BODY_YAW=0.0000

# ============ 盒底 -> 相機 ============
CAM_X=0.0892
CAM_Y=-0.0454
CAM_Z=0.1190
CAM_ROLL=0.0092
CAM_PITCH=-0.0278
CAM_YAW=0.0154
# =====================================================================

pub() {   # pub parent child x y z roll pitch yaw
    setsid nohup ros2 run tf2_ros static_transform_publisher \
        --x "$3" --y "$4" --z "$5" --roll "$6" --pitch "$7" --yaw "$8" \
        --frame-id "$1" --child-frame-id "$2" \
        > "/tmp/tf_$2.log" 2>&1 < /dev/null &
    printf "  %-12s -> %-12s (%s, %s, %s)  rpy(%s, %s, %s)\n" \
        "$1" "$2" "$3" "$4" "$5" "$6" "$7" "$8"
}

echo "=== 模式:$MODE ==="

# 只殺我們自己發的。樹莓派的發布者不在這台機器上,殺不到也不該殺。
for ch in box_link body camera_link base_lidar camera_init base_link; do
    pkill -f "static_transform_publisher.*--child-frame-id $ch" 2>/dev/null
done
pkill -f odom_cov_relay.py 2>/dev/null
pkill -f "ekf_node" 2>/dev/null
sleep 2

echo "--- 感測器安裝(兩種模式都一樣)---"
pub box_link body        "$BODY_X" "$BODY_Y" "$BODY_Z" "$BODY_ROLL" "$BODY_PITCH" "$BODY_YAW"
pub box_link camera_link "$CAM_X"  "$CAM_Y"  "$CAM_Z"  "$CAM_ROLL"  "$CAM_PITCH"  "$CAM_YAW"

echo "--- 里程計 ---"
case "$MODE" in
fused)
    echo "  EKF 融合:輪速 + 光達 -> multi_odom → box_link"
    cd "$D" && setsid nohup python3 odom_cov_relay.py \
        > /tmp/odom_relay.log 2>&1 < /dev/null &
    sleep 4
    pgrep -f odom_cov_relay.py > /dev/null \
        && echo "  odom_cov_relay OK" \
        || { echo "  odom_cov_relay DEAD"; tail -10 /tmp/odom_relay.log; exit 1; }

    setsid nohup ros2 run robot_localization ekf_node \
        --ros-args --params-file "$D/ekf_multi.yaml" \
        -r __node:=ekf_filter_node \
        > /tmp/ekf.log 2>&1 < /dev/null &
    sleep 8
    pgrep -f ekf_node > /dev/null \
        && echo "  ekf_node OK" \
        || { echo "  ekf_node DEAD"; tail -12 /tmp/ekf.log; exit 1; }
    ODOM_FRAME=multi_odom
    ;;
lidar)
    echo "  只有光達:EKF 單來源 -> multi_odom → box_link"
    # 一樣用 EKF,只是輪速那一路沒資料。sensor_timeout 會讓它自己忽略,
    # 不用另外寫一支節點,也不用改任何下游設定。
    cd "$D" && setsid nohup python3 odom_cov_relay.py \
        > /tmp/odom_relay.log 2>&1 < /dev/null &
    sleep 4
    setsid nohup ros2 run robot_localization ekf_node \
        --ros-args --params-file "$D/ekf_multi.yaml" \
        -r __node:=ekf_filter_node \
        > /tmp/ekf.log 2>&1 < /dev/null &
    sleep 8
    pgrep -f ekf_node > /dev/null && echo "  ekf_node OK" || echo "  ekf_node DEAD"
    ODOM_FRAME=multi_odom
    ;;
*)
    echo "  未知模式,用 fused 或 lidar"
    exit 1
    ;;
esac
sleep 4

echo
echo "=== 驗證 ==="
python3 - <<'PY'
import math, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

def q2R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

rclpy.init(); n = Node("tree_verify"); buf = Buffer(); TransformListener(buf, n)
child = {}; acc = []; cnt = {"w": 0, "l": 0, "f": 0}
def cb(m):
    for t in m.transforms:
        child.setdefault(t.child_frame_id, set()).add(t.header.frame_id)
q = QoSProfile(depth=200, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               history=HistoryPolicy.KEEP_LAST)
n.create_subscription(TFMessage, "/tf_static", cb, q)
n.create_subscription(TFMessage, "/tf", cb, 50)
n.create_subscription(Imu, "/livox/imu", lambda m: acc.append(
    [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]),
    qos_profile_sensor_data)
n.create_subscription(Odometry, "/odom_wheel_cov", lambda m: cnt.__setitem__("w", cnt["w"]+1), 20)
n.create_subscription(Odometry, "/odom_lidar_cov", lambda m: cnt.__setitem__("l", cnt["l"]+1), 20)
n.create_subscription(Odometry, "/odometry/filtered", lambda m: cnt.__setitem__("f", cnt["f"]+1), 20)
t0 = time.time()
while time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.1)

dup = {c: sorted(p) for c, p in child.items() if len(p) > 1}
print("  雙父節點: %s" % (dup if dup else "沒有"))
print("  樹:")
for c, ps in sorted(child.items()):
    print("    %-26s <- %s" % (c, ", ".join(sorted(ps))))
print()
print("  EKF 輸入 輪速 %.1f Hz   光達 %.1f Hz   輸出 %.1f Hz"
      % (cnt["w"]/12.0, cnt["l"]/12.0, cnt["f"]/12.0))
print()
for a, b in [("multi_odom", "box_link"), ("box_link", "body"), ("map", "box_link")]:
    try:
        t = buf.lookup_transform(a, b, rclpy.time.Time())
        v = t.transform.translation
        print("  %-12s -> %-12s (%+.4f, %+.4f, %+.4f)" % (a, b, v.x, v.y, v.z))
    except Exception as e:
        print("  %-12s -> %-12s FAIL %s" % (a, b, str(e)[:50]))

if len(acc) > 50:
    up = np.mean(acc, axis=0); up /= np.linalg.norm(up)
    try:
        t = buf.lookup_transform("box_link", "body", rclpy.time.Time())
        qq = t.transform.rotation
        u = q2R(qq.x, qq.y, qq.z, qq.w) @ up
        d = math.degrees(math.acos(min(1, max(-1, u[2]))))
        print("  重力在 box_link 裡離垂直 %.2f 度  %s"
              % (d, "OK" if d < 3 else "還是斜的"))
    except Exception as e:
        print("  重力檢查失敗:", str(e)[:60])
PY
