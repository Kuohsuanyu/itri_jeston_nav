#!/usr/bin/env bash
# Topic names visible but zero data == discovery (UDP multicast) works while
# the data path does not. Fast DDS prefers shared memory on the same host,
# so the prime suspect is /dev/shm not really being shared with the container.
IMG=isaac_ros_dev-aarch64:latest
WS=/mnt/ssd/ws

echo "=== host 端 /dev/shm ==="
df -h /dev/shm | tail -1 | sed 's/^/  /'
ls /dev/shm | head -8 | sed 's/^/  /'
echo "  fastrtps 相關檔案數: $(ls /dev/shm 2>/dev/null | grep -c fastrtps)"

echo
echo "=== 容器內 /dev/shm(--ipc=host) ==="
docker run --rm --network host --ipc=host --privileged --runtime nvidia \
    -e ROS_DOMAIN_ID=0 -v /dev:/dev \
    --entrypoint /bin/bash "$IMG" -c '
        df -h /dev/shm | tail -1 | sed "s/^/  /"
        ls /dev/shm 2>/dev/null | head -8 | sed "s/^/  /"
        echo "  fastrtps 相關檔案數: $(ls /dev/shm 2>/dev/null | grep -c fastrtps)"
    '

echo
echo "=== 測試:強制關掉 SHM,只走 UDP ==="
cat > /tmp/fastdds_udp_only.xml <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_only</transport_id>
        <type>UDPv4</type>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="udp_participant" is_default_profile="true">
      <rtps>
        <userTransports><transport_id>udp_only</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
XML

docker run --rm --network host --ipc=host --privileged --runtime nvidia \
    -e ROS_DOMAIN_ID=0 \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_udp_only.xml \
    -v /tmp/fastdds_udp_only.xml:/tmp/fastdds_udp_only.xml:ro \
    -v /dev:/dev -v "$WS":/workspaces/isaac_ros-dev \
    --entrypoint /bin/bash "$IMG" -c '
source /opt/ros/humble/setup.bash
python3 - <<PY
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

n_ = {"depth":0, "odom":0}
rclpy.init(); node = Node("udp_probe")
def bump(k):
    def f(m): n_[k] += 1
    return f
node.create_subscription(Image, "/camera/camera/depth/image_rect_raw", bump("depth"), qos_profile_sensor_data)
node.create_subscription(Odometry, "/Odometry", bump("odom"), 10)
buf = Buffer(); TransformListener(buf, node)
t0 = time.time()
while time.time() - t0 < 35:
    rclpy.spin_once(node, timeout_sec=0.1)
dt = time.time() - t0
for k, v in n_.items():
    print("    %-6s %5d 則  (%.1f Hz)" % (k, v, v/dt))
try:
    t = buf.lookup_transform("odom", "camera_depth_optical_frame", rclpy.time.Time())
    v = t.transform.translation
    print("    [OK]   odom -> camera_depth_optical_frame (%.3f, %.3f, %.3f)" % (v.x,v.y,v.z))
except Exception as e:
    print("    [FAIL] TF:", str(e)[:60])
PY
'
