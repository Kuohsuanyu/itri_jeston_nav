#!/usr/bin/env bash
# Bring the whole stack up, then measure the sensor-vs-host clock offset.
# ASCII only: the base64|bash pipeline mangles CJK.
#
# 2026-08-08: nvblox removed. The D435 is now a COLOR SOURCE only -- the lidar
# supplies all geometry, and lidar_web/server.py projects the cloud into the
# color image to paint it. align_depth stays enabled because the aligned depth
# frame is what the occlusion test reads (without it, a wall's color gets
# painted onto whatever is behind the wall).
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

start_capped() {
    unit="$1"; cap="$2"; log="$3"; shift 3
    systemctl --user reset-failed "$unit.scope" 2>/dev/null
    if command -v systemd-run > /dev/null 2>&1; then
        systemd-run --user --scope -p MemoryMax="$cap" --unit="$unit" --quiet \
          "$@" > "$log" 2>&1 < /dev/null &
    else
        setsid nohup "$@" > "$log" 2>&1 < /dev/null &
    fi
}

echo "[0/8] make sure no Isaac container is holding the camera"
for c in nvblox nvblox_mesh_web; do
    docker inspect "$c" > /dev/null 2>&1 && docker rm -f "$c" > /dev/null 2>&1 \
        && echo "  removed stale container: $c"
done

echo
echo "[1/8] wait for lidar NIC"
for i in $(seq 1 30); do
    C=$(cat /sys/class/net/enP8p1s0/carrier 2>/dev/null || echo 0)
    N=$(ip -4 -o addr show enP8p1s0 2>/dev/null | grep -c "192.168.0.100")
    [ "$C" = "1" ] && [ "$N" -ge 1 ] && { echo "  ready after ${i}s"; break; }
    sleep 1
done
ping -c1 -W2 192.168.0.50 > /dev/null 2>&1 && echo "  lidar responds" || echo "  WARN: lidar no ping"

echo "[2/8] livox driver"
setsid nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  > /tmp/livox.log 2>&1 < /dev/null &
sleep 12
grep -q "Init lds lidar fail" /tmp/livox.log && echo "  ERROR: bind failed" || echo "  ok"

echo "[3/8] FAST-LIO (3G cap, publishes camera_init -> body)"
# ros2 run, NOT ros2 launch -- keeps the params-file path explicit.
# (odom_lidar / base_lidar) -- see lidar_odom_tf.sh.
#
# We avoid the old base_link collision by renaming our frames instead
# RC and publishes nothing to ROS. Its /tf must stay ON.
# FAST-LIO IS the odometry source now: the chassis is driven by its own physical
# it silently alternates between the two answers.
FASTLIO_CFG=/home/andykuo/ws_livox/install/fast_lio/share/fast_lio/config/mid360.yaml
start_capped fastlio-run 3G /tmp/fastlio.log \
  ros2 run fast_lio fastlio_mapping --ros-args \
    --params-file "$FASTLIO_CFG" -p use_sim_time:=false \
    -r /tf:=/tf_fastlio_unused
sleep 18

# Camera comes up BEFORE the web viewer now. server.py latches the color
# intrinsics on the first CameraInfo; starting it first just means the first
# few seconds of cloud arrive uncolored.
echo "[4/8] RealSense D435 (640x480 @15, aligned depth)"
pkill -f realsense2_camera_node 2>/dev/null; sleep 2
setsid nohup ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=true enable_color:=true \
    enable_infra1:=false enable_infra2:=false \
    depth_module.depth_profile:=640x480x15 \
    rgb_camera.color_profile:=640x480x15 \
    pointcloud.enable:=false align_depth.enable:=true \
    > /tmp/realsense.log 2>&1 < /dev/null &
sleep 22
pgrep -f realsense2_camera_node > /dev/null && echo "  camera ok" || echo "  camera DEAD"

# base_link -> camera_link. ICP-calibrated 2026-08-07, verified twice.
# Without this TF the cloud simply stays uncolored -- nothing else breaks.
echo "[5/8] camera extrinsic"
# camera_extrinsic.sh 已停用:相機外參改由 robot_tf.sh 以
# box_link -> camera_link 發布。留著它會讓 camera_link 有兩個
# 父節點(base_link 和 box_link),整棵樹裂成兩半。

echo "[6/8] 3D viewer :8080 (colored)"
pkill -9 -f "python3 server.py" 2>/dev/null; sleep 1
cd ~/lidar_web && setsid nohup python3 server.py > /tmp/webviewer.log 2>&1 < /dev/null &
sleep 5
cd ~
grep -i "查不到\|Traceback\|Error" /tmp/webviewer.log | head -3 | sed 's/^/  /'

echo "[7/8] 2D mapping + map viewer :8090"
bash ~/slam2d/start_slam2d.sh > /tmp/slam2d_start.log 2>&1
pkill -9 -f "python3 map_server.py" 2>/dev/null; sleep 1
cd ~/slam2d && setsid nohup python3 map_server.py > /tmp/mapweb.log 2>&1 < /dev/null &
sleep 4
cd ~

echo "[8/8] zenoh bridge + camera web viewer :8092"
setsid nohup env RUST_LOG=info zenoh-bridge-ros2dds \
  -l tcp/0.0.0.0:7447 --no-multicast-scouting \
  --pub-max-frequency "/cloud_registered=3.0" \
  > /tmp/zenoh.log 2>&1 < /dev/null &
pkill -9 -f "python3 cam_server.py" 2>/dev/null; sleep 1
cd ~/cam_web && setsid nohup python3 cam_server.py > /tmp/cam_web.log 2>&1 < /dev/null &
sleep 8
cd ~

echo
echo "=== status ==="
for p in livox_ros_driver2_node fastlio_mapping "python3 server.py" \
         "python3 map_server.py" async_slam_toolbox_node \
         pointcloud_to_laserscan zenoh-bridge-ros2dds \
         realsense2_camera_node "python3 cam_server.py"; do
    pgrep -f "$p" > /dev/null && echo "  [OK]   $p" || echo "  [DEAD] $p"
done
echo "--- camera stream rates ---"
curl -sS http://127.0.0.1:8092/stats.json 2>/dev/null | sed 's/^/  /'; echo
free -m | head -2 | sed 's/^/  /'

echo
echo "=============== colorization preconditions ==============="
# Coloring needs exactly three things. If all three are OK, the 'colored'
# percentage in the browser will climb as the camera sweeps the room.
python3 - <<'PY'
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener

rclpy.init()
n = Node("color_precheck")
buf = Buffer(); TransformListener(buf, n)
got = {}
n.create_subscription(Image, "/camera/camera/color/image_raw",
                      lambda m: got.setdefault("color", m), qos_profile_sensor_data)
n.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw",
                      lambda m: got.setdefault("adepth", m), qos_profile_sensor_data)
n.create_subscription(CameraInfo, "/camera/camera/color/camera_info",
                      lambda m: got.setdefault("info", m), qos_profile_sensor_data)
n.create_subscription(PointCloud2, "/cloud_registered",
                      lambda m: got.setdefault("cloud", m), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 20 and len(got) < 4:
    rclpy.spin_once(n, timeout_sec=0.2)

c = got.get("cloud")
world = c.header.frame_id if c else "?"
print("  1. cloud       : %s  (world frame = %s)"
      % ("OK" if c else "NO DATA", world))
print("  2. color image : %s   aligned depth: %s   intrinsics: %s"
      % ("OK" if "color" in got else "NO DATA",
         "OK" if "adepth" in got else "MISSING (occlusion test off)",
         "OK" if "info" in got else "NO DATA"))

ok = False
for src in (world, "odom", "camera_init", "map"):
    if src == "?":
        continue
    try:
        t = buf.lookup_transform("camera_color_optical_frame", src, rclpy.time.Time())
        v = t.transform.translation
        print("  3. TF %s -> camera_color_optical_frame : OK  (%.3f, %.3f, %.3f)"
              % (src, v.x, v.y, v.z))
        ok = True
        break
    except Exception:
        pass
if not ok:
    print("  3. TF -> camera_color_optical_frame : FAIL")
    print("     cloud will stay grey. check /tmp/cam_extrinsic.log")
PY

echo
echo "=============== clock offset ==============="
python3 - <<'PY'
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

got = {}
rclpy.init()
n = Node("clock_check")
n.create_subscription(Imu, "/livox/imu", lambda m: got.setdefault("imu", m), qos_profile_sensor_data)
n.create_subscription(Odometry, "/Odometry", lambda m: got.setdefault("odom", m), 10)
t0 = time.time()
while time.time() - t0 < 15 and len(got) < 2:
    rclpy.spin_once(n, timeout_sec=0.2)
host = time.time()
print("  host clock            : %.3f" % host)
for k, label in (("imu", "/livox/imu    stamp"), ("odom", "/Odometry     stamp")):
    m = got.get(k)
    if m is None:
        print("  %-22s: NO DATA" % label); continue
    s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    print("  %-22s: %.3f   offset %+.3f s" % (label, s, s - host))
PY
