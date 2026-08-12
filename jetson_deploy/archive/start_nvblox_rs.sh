#!/usr/bin/env bash
# RealSense + nvblox 在同一個 component container 裡跑。
#
# host 端的相機驅動必須先停 —— USB 裝置只能被開啟一次。
# 停掉之後 host 的 cam_web_viewer(:8092)仍然可以訂閱容器發布的
# /camera0/camera/... topic,只是 topic 名稱換了。
IMG=isaac_ros_dev-aarch64:nvblox-rs
WS=/mnt/ssd/ws

echo "=== 停掉 host 的相機驅動(USB 不能兩邊搶) ==="
pkill -f realsense2_camera_node 2>/dev/null && echo "  已停" || echo "  本來就沒跑"
sleep 4

echo "=== 前置檢查 ==="
pgrep -f fastlio_mapping > /dev/null && echo "  [OK] FAST-LIO(提供位姿)" \
    || { echo "  [DEAD] FAST-LIO — 先跑 startall.sh"; exit 1; }
pgrep -f "static_transform_publisher.*camera_link" > /dev/null \
    && echo "  [OK] base_link -> camera_link 外參" \
    || echo "  ⚠ 外參 TF 沒在跑,nvblox 會查不到相機位姿"

docker rm -f nvblox 2>/dev/null || true

echo
echo "=== 啟動 container(realsense + nvblox 同行程) ==="
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
ros2 launch /workspaces/isaac_ros-dev/nvblox_rs.launch.py
" > /dev/null

sleep 40
echo "  容器狀態: $(docker inspect -f '{{.State.Status}}' nvblox 2>/dev/null)"

echo
echo "=== 有沒有 depth 整合(關鍵指標) ==="
docker logs nvblox 2>&1 | grep -iE "depth/integrate|mesh/integrate|error|fail" | tail -10 | sed 's/^/  /'

echo
echo "=== 相機是否在容器裡起來 ==="
docker logs nvblox 2>&1 | grep -iE "RealSense Node Is Up|Open profile|device" | tail -6 | sed 's/^/  /'
