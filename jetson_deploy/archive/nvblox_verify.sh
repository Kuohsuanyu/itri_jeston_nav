#!/usr/bin/env bash
# Is nvblox actually consuming depth and producing a map?
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

echo "=== nvblox 的錯誤/警告 ==="
docker logs nvblox 2>&1 | grep -iE "error|warn|fail|could not|unavailable|dropp" \
  | tail -12 | sed 's/^/  /' || echo "  (無)"

echo
echo "=== nvblox 發布的 topic ==="
timeout 25 ros2 topic list 2>/dev/null | grep -i nvblox | sed 's/^/  /' || echo "  (還沒出現)"

echo
echo "=== 實際輸出頻率(45 秒) ==="
python3 - <<'PY'
import time, rclpy
from rclpy.node import Node
from nvblox_msgs.msg import Mesh, DistanceMapSlice
try:
    HAVE = True
except Exception:
    HAVE = False
PY
python3 - <<'PY'
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
n = Node("nvblox_probe")
cnt = {}
import rosidl_runtime_py.utilities as u
names = dict(n.get_topic_names_and_types())
targets = [t for t in names if "nvblox" in t]
for t in targets:
    try:
        cls = u.get_message(names[t][0])
    except Exception:
        continue
    cnt[t] = 0
    def mk(k):
        def f(m): cnt[k] += 1
        return f
    n.create_subscription(cls, t, mk(t), 5)

print("  監看 %d 個 nvblox topic" % len(cnt), flush=True)
t0 = time.time()
while time.time() - t0 < 45:
    rclpy.spin_once(n, timeout_sec=0.1)
dt = time.time() - t0
if not cnt:
    print("  沒有找到 nvblox topic")
for k in sorted(cnt):
    print("    %-45s %4d 則 (%.1f Hz)" % (k, cnt[k], cnt[k]/dt))
PY

echo
echo "=== 資源 ==="
docker stats nvblox --no-stream --format "  CPU {{.CPUPerc}}  MEM {{.MemUsage}}" 2>/dev/null
cat /proc/loadavg | awk '{print "  loadavg "$1}'
free -m | head -2 | sed 's/^/  /'
