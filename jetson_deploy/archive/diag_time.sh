#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0
echo "=== 系統負載 ==="
uptime
nproc
top -bn1 | head -12
echo
echo "=== mid360 設定 ==="
for f in ~/ws_livox/install/fast_lio/share/fast_lio/config/mid360.yaml \
         ~/ws_livox/src/fast_lio/config/mid360.yaml; do
  [ -f "$f" ] && { echo "--- $f ---"; grep -nE "time_sync|timestamp|blind|filter_size|point_filter|lid_topic|imu_topic|extrinsic" "$f"; }
done
echo "--- MID360_config.json ---"
find ~/ws_livox -name "MID360_config.json" 2>/dev/null | head -2 | while read f; do echo "$f"; cat "$f"; done
echo
echo "=== 訊息時戳 vs ROS 時鐘 ==="
python3 - <<'PY'
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
rclpy.init(); n=Node("ts")
rows={}
def stamp(m): return m.header.stamp.sec + m.header.stamp.nanosec*1e-9
n.create_subscription(Imu,"/livox/imu",lambda m: rows.__setitem__("imu",(stamp(m), n.get_clock().now().nanoseconds*1e-9)),qos_profile_sensor_data)
try:
    from livox_ros_driver2.msg import CustomMsg
    n.create_subscription(CustomMsg,"/livox/lidar",lambda m: rows.__setitem__("lidar",(stamp(m), n.get_clock().now().nanoseconds*1e-9)),qos_profile_sensor_data)
    have=True
except Exception as e:
    print("  (載不到 CustomMsg:%s)"%str(e)[:60]); have=False
t0=time.time()
while time.time()-t0<8 and len(rows)<(2 if have else 1):
    rclpy.spin_once(n,timeout_sec=0.05)
for k,(s,now) in rows.items():
    print("  %-6s header.stamp %.3f   收到當下 ROS 時鐘 %.3f   落後 %+.3f s"%(k,s,now,now-s))
if "imu" in rows and "lidar" in rows:
    print("  ==> IMU 與光達的 header.stamp 相差 %+.3f s"%(rows["imu"][0]-rows["lidar"][0]))
print("  牆鐘 time.time() = %.3f"%time.time())
PY
