#!/usr/bin/env bash
# Bring the whole stack up, then measure the sensor-vs-host clock offset.
# ASCII only: the base64|bash pipeline mangles CJK.
#
# 2026-08-08: nvblox removed. The D435 is now a COLOR SOURCE only -- the lidar
# supplies all geometry.
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

# Camera:only depth + color, no pointcloud —— 那個在 CPU 上組點雲是白做工。
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

# ★ 2026-08-12 拿掉三個網頁檢視器(:8080 點雲 / :8090 地圖 / :8092 相機)。
#
# 它們吃掉 Orin Nano 一大塊 CPU —— 三個 python3 加起來近 100%,
# 加上 livox 驅動 85%、FAST-LIO 43%,六核心的 load average 衝到 5.4。
# 後果不只是慢:核心來不及處理網路封包,實測 RX dropped 5289 個,
# 從筆電 ping Jetson 變成 26% 遺失、最大 1096 ms。
#
# 而 RViz 已經涵蓋它們全部的功能(點雲、地圖、相機影像、送目標點),
# 而且算繪在筆電上,不佔 Jetson。
#
# 需要看點雲/影像用 RViz,它算繪在筆電上,不佔 Jetson。

# ★ 這支只負責**感測器層**:光達驅動 + FAST-LIO + 相機。
#   TF / EKF / SLAM / bridge 由 bringup_all.sh 依序帶起,不要在這裡重複起,
#   兩支腳本都起同樣的東西會互相 pkill,實測會卡住。

echo "=== status ==="
for p in livox_ros_driver2_node fastlio_mapping async_slam_toolbox_node \
         pointcloud_to_laserscan zenoh-bridge-ros2dds realsense2_camera_node; do
    pgrep -f "$p" > /dev/null && echo "  [OK]   $p" || echo "  [DEAD] $p"
done
free -m | head -2 | sed 's/^/  /'

echo

echo
echo "感測器層完成。TF / SLAM / bridge 請用 bringup_all.sh。"