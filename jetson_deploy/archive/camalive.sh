#!/usr/bin/env bash
# Before restructuring the architecture: is the depth topic actually carrying
# data right now? "Publisher count: 2" included the zenoh bridge echoing the
# topic back, so a dead camera would still look like it has a publisher.
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

echo "=== 相機行程 ==="
pgrep -af realsense2_camera_node | head -2 | sed 's/^/  /' || echo "  DEAD"
echo "=== cam_web 的實際頻率(host 端獨立驗證) ==="
curl -sS --max-time 5 http://127.0.0.1:8092/stats.json 2>/dev/null | sed 's/^/  /' || echo "  cam_server 沒跑"

echo
echo "=== host 端直接量深度 topic(30 秒) ==="
python3 - <<'PY'
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
n_ = {"depth": 0, "info": 0}
rclpy.init(); nd = Node("cam_alive")
def mk(k):
    def f(m): n_[k] += 1
    return f
nd.create_subscription(Image, "/camera/camera/depth/image_rect_raw", mk("depth"), qos_profile_sensor_data)
nd.create_subscription(CameraInfo, "/camera/camera/depth/camera_info", mk("info"), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 30:
    rclpy.spin_once(nd, timeout_sec=0.1)
dt = time.time() - t0
for k, v in n_.items():
    print("    %-6s %4d 則 (%.1f Hz)" % (k, v, v/dt))
PY

echo
echo "=== 誰在發布這個 topic ==="
timeout 25 ros2 topic info -v /camera/camera/depth/image_rect_raw 2>/dev/null \
  | grep -E "Node name|Endpoint type" | paste - - | sed 's/^/  /'
