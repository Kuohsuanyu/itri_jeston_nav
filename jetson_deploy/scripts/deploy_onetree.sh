#!/usr/bin/env bash
# 把「兩棵樹併成一棵」這次的改動佈署到 Jetson,並改掉只存在於 Jetson 的
# slam_params.yaml。
#
# 在 Jetson 上跑:
#     bash ~/slam2d/deploy_onetree.sh
#
# 前置條件(在樹莓派上做,這支檢查不到):
#     chassis_bringup/config/vehicle_param_DD-M.yaml
#         publish_tf: true   ->   false
#     然後重啟 bringup。
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

D=~/slam2d
S="$D/slam_params.yaml"

echo "=== 1/3 slam_params.yaml ==="
[ -f "$S" ] || { echo "  ✗ 找不到 $S"; exit 1; }
cp "$S" "$S.bak.$(date +%Y%m%d-%H%M%S)"
before=$(grep -E "odom_frame|base_frame" "$S" || true)
sed -i -E 's/^(\s*odom_frame:\s*).*/\1multi_odom/;   s/^(\s*base_frame:\s*).*/\1base_footprint/' "$S"
echo "  改前: $(echo "$before" | tr '\n' ' ')"
echo "  改後: $(grep -E 'odom_frame|base_frame' "$S" | tr '\n' ' ')"

echo
echo "=== 2/3 確認底盤已經停發 odom -> base_footprint ==="
# 這是整件事的前提。底盤還在發的話 base_footprint 會有兩個父節點,
# tf2 不報錯,只會在兩個答案之間隨機翻轉 —— 症狀是里程計看起來在飄。
if timeout 8 ros2 topic echo /tf --once 2>/dev/null \
     | grep -q 'child_frame_id: base_footprint'; then
    echo "  ✗ 底盤還在發。到樹莓派改 publish_tf: false 並重啟 bringup。"
    exit 1
fi
echo "  OK,沒有人發 base_footprint 的 TF"

# /odom topic 必須還活著 —— 關掉的是 TF 不是資料。
if timeout 8 ros2 topic hz /odom 2>/dev/null | head -1 | grep -q rate; then
    echo "  OK,/odom topic 照常有資料"
else
    echo "  ✗ /odom 沒資料 —— 是不是連 bringup 都停了?"
    exit 1
fi

echo
echo "=== 3/3 重啟整條鏈 ==="
bash "$D/start_slam2d.sh"

echo
echo "=== 驗收 ==="
python3 - <<'PY'
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

rclpy.init(); n = Node("onetree_verify"); buf = Buffer(); TransformListener(buf, n)
child = {}
def cb(m):
    for t in m.transforms:
        child.setdefault(t.child_frame_id, set()).add(t.header.frame_id)
q = QoSProfile(depth=200, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               history=HistoryPolicy.KEEP_LAST)
n.create_subscription(TFMessage, "/tf_static", cb, q)
n.create_subscription(TFMessage, "/tf", cb, 50)
t0 = time.time()
while time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.1)

dup = {c: sorted(p) for c, p in child.items() if len(p) > 1}
roots = sorted(set(p for ps in child.values() for p in ps) - set(child))
print("  雙父節點: %s" % (dup if dup else "沒有"))
print("  樹根: %s   %s" % (roots,
      "只有一個 = 一棵樹 ✓" if len(roots) == 1 else "<-- 超過一個,樹還是斷的"))
print("  樹:")
for c, ps in sorted(child.items()):
    print("    %-26s <- %s" % (c, ", ".join(sorted(ps))))
print()
for a, b in [("map", "base_footprint"), ("multi_odom", "base_footprint"),
             ("base_footprint", "base_link"), ("base_link", "box_link"),
             ("box_link", "body"), ("base_footprint", "body")]:
    try:
        t = buf.lookup_transform(a, b, rclpy.time.Time())
        v = t.transform.translation
        print("  %-16s -> %-16s (%+.4f, %+.4f, %+.4f)" % (a, b, v.x, v.y, v.z))
    except Exception as e:
        print("  %-16s -> %-16s FAIL %s" % (a, b, str(e)[:44]))
PY
