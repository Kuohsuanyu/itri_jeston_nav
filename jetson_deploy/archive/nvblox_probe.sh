#!/usr/bin/env bash
# Probe from INSIDE the container: nvblox_msgs only exists in its workspace.
docker run --rm \
    --network host --ipc=host --privileged --runtime nvidia \
    -e ROS_DOMAIN_ID=0 \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/fastdds_udp_only.xml \
    -v /mnt/ssd/ws:/workspaces/isaac_ros-dev -v /dev:/dev \
    --workdir /workspaces/isaac_ros-dev \
    --entrypoint /bin/bash isaac_ros_dev-aarch64:nvblox -c '
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 - <<PY
import time, rclpy
from rclpy.node import Node
import rosidl_runtime_py.utilities as u

rclpy.init()
n = Node("probe")
time.sleep(3)
names = dict(n.get_topic_names_and_types())
watch = ["/nvblox_node/mesh", "/nvblox_node/static_map_slice",
         "/nvblox_node/static_esdf_pointcloud", "/nvblox_node/tsdf_layer",
         "/nvblox_node/static_occupancy_grid"]
cnt = {}
for t in watch:
    if t not in names: continue
    try: cls = u.get_message(names[t][0])
    except Exception: continue
    cnt[t] = 0
    def mk(k):
        def f(m): cnt[k] += 1
        return f
    n.create_subscription(cls, t, mk(t), 5)

print("  監看 %d 個輸出 topic,等 40 秒" % len(cnt), flush=True)
t0 = time.time()
while time.time() - t0 < 40:
    rclpy.spin_once(n, timeout_sec=0.1)
dt = time.time() - t0
print()
print("  === nvblox 輸出 ===")
for k in watch:
    if k in cnt:
        print("    %-42s %4d 則 (%.2f Hz)" % (k.split(\"/\")[-1], cnt[k], cnt[k]/dt))
    else:
        print("    %-42s  未發布" % k.split(\"/\")[-1])
PY
'
