#!/usr/bin/env bash
# Run nvblox in the Isaac ROS container against the host's RealSense + FAST-LIO.
#
# Architecture:
#   Mid-360 -> FAST-LIO -> TF (odom -> base_link)      [host]
#   D435    -> realsense2_camera -> depth/color        [host]
#                     |
#                     v
#                  nvblox (GPU)                        [container]
#                     |-> 彩色 mesh
#                     '-> 2D ESDF slice -> Nav2 local costmap
#
# The UDP-only DDS profile is mandatory: without it the container sees every
# topic name but receives zero messages (Fast DDS shared memory does not work
# across the container boundary even with --ipc=host).
#
# 用法: start_nvblox.sh [depth_only]
IMG=isaac_ros_dev-aarch64:nvblox
WS=/mnt/ssd/ws
MODE=${1:-color}

USE_COLOR=true
[ "$MODE" = "depth_only" ] && USE_COLOR=false

echo "=== 前置檢查 ==="
for p in fastlio_mapping realsense2_camera_node; do
    pgrep -f "$p" > /dev/null && echo "  [OK]   $p" || { echo "  [DEAD] $p — 先跑 startall.sh"; exit 1; }
done
[ -f "$WS/fastdds_udp_only.xml" ] || { echo "  找不到 DDS profile"; exit 1; }
echo "  模式: $MODE (use_color=$USE_COLOR)"

docker rm -f nvblox 2>/dev/null || true

echo
echo "=== 啟動 nvblox ==="
docker run -d --name nvblox \
    --network host --ipc=host --privileged --runtime nvidia \
    -e ROS_DOMAIN_ID=0 \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/fastdds_udp_only.xml \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v "$WS":/workspaces/isaac_ros-dev \
    -v /dev:/dev \
    --workdir /workspaces/isaac_ros-dev \
    --entrypoint /bin/bash \
    "$IMG" -c "
source /opt/ros/humble/setup.bash
source install/setup.bash
# nvblox 3.x 支援多相機,訂閱的是**全域**名稱 /camera_0/...,
# 不是 /nvblox_node/... 。用錯前綴的話節點照跑、照發布空地圖,
# 但一幀深度都不會進來(時間統計裡完全沒有 depth/integrate 就是這個症狀)。
# 名稱以 \`ros2 node info /nvblox_node\` 的 Subscribers 為準。
ros2 run nvblox_ros nvblox_node --ros-args \
    --params-file /workspaces/isaac_ros-dev/nvblox_d435.yaml \
    -p use_color:=$USE_COLOR \
    -r /camera_0/depth/image:=/camera/camera/depth/image_rect_raw \
    -r /camera_0/depth/camera_info:=/camera/camera/depth/camera_info \
    -r /camera_0/color/image:=/camera/camera/color/image_raw \
    -r /camera_0/color/camera_info:=/camera/camera/color/camera_info
" > /dev/null

sleep 25
echo "  容器狀態: $(docker inspect -f '{{.State.Status}}' nvblox 2>/dev/null)"
echo
echo "=== log ==="
docker logs nvblox 2>&1 | tail -20 | sed 's/^/  /'
