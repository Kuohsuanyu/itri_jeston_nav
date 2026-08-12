#!/usr/bin/env bash
# Topic names being visible proves discovery works, not that data flows.
# That exact distinction bit us on the WSL side earlier ("名字看得到、資料進不來"),
# so measure actual message arrival inside the container with a long window.
IMG=isaac_ros_dev-aarch64:latest
WS=/mnt/ssd/ws

docker run --rm \
    --network host --ipc=host --privileged \
    --runtime nvidia \
    -e ROS_DOMAIN_ID=0 \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "$WS":/workspaces/isaac_ros-dev \
    -v /dev:/dev \
    --entrypoint /bin/bash \
    "$IMG" -c '
source /opt/ros/humble/setup.bash
python3 - <<PY
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

n_ = {"depth":0, "color":0, "dinfo":0, "cinfo":0, "odom":0}
rclpy.init()
node = Node("container_probe")
def bump(k):
    def f(m): n_[k] += 1
    return f
node.create_subscription(Image, "/camera/camera/depth/image_rect_raw", bump("depth"), qos_profile_sensor_data)
node.create_subscription(Image, "/camera/camera/color/image_raw", bump("color"), qos_profile_sensor_data)
node.create_subscription(CameraInfo, "/camera/camera/depth/camera_info", bump("dinfo"), qos_profile_sensor_data)
node.create_subscription(CameraInfo, "/camera/camera/color/camera_info", bump("cinfo"), qos_profile_sensor_data)
node.create_subscription(Odometry, "/Odometry", bump("odom"), 10)
buf = Buffer(); TransformListener(buf, node)

print("  等待資料(45 秒,這台探索延遲一向很長)...", flush=True)
t0 = time.time()
while time.time() - t0 < 45:
    rclpy.spin_once(node, timeout_sec=0.1)
dt = time.time() - t0

print()
print("  === 容器內實際收到的訊息 ===")
for k, v in n_.items():
    print("    %-8s %5d 則  (%.1f Hz)" % (k, v, v/dt))

print()
print("  === TF 鏈路 ===")
for a, b in [("odom","base_link"), ("odom","camera_depth_optical_frame"),
             ("odom","camera_color_optical_frame"), ("map","odom")]:
    try:
        t = buf.lookup_transform(a, b, rclpy.time.Time())
        v = t.transform.translation
        print("    [OK]   %-6s -> %-30s (%.3f, %.3f, %.3f)" % (a,b,v.x,v.y,v.z))
    except Exception as e:
        print("    [FAIL] %-6s -> %-30s %s" % (a,b,str(e)[:45]))
PY
'
