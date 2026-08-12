#!/usr/bin/env bash
# Can the container see the host's ROS topics?
#
# This decides the whole architecture: camera driver + FAST-LIO stay on the
# host, nvblox runs in the container. run_dev.sh uses --network host and
# --ipc=host and forwards ROS_DOMAIN_ID, so it should work -- but DDS across
# a container boundary has bitten us before, so verify rather than assume.
set -e
IMG=isaac_ros_dev-aarch64:latest
WS=/mnt/ssd/ws

echo "=== host 端目前發布的 topic ==="
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0
timeout 20 ros2 topic list 2>/dev/null | grep -E "camera|cloud_registered|^/tf|Odometry" | sed 's/^/  /'

echo
echo "=== 容器內看到什麼 ==="
docker run --rm \
    --network host --ipc=host --privileged \
    --runtime nvidia \
    -e ROS_DOMAIN_ID=0 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "$WS":/workspaces/isaac_ros-dev \
    -v /dev:/dev \
    --entrypoint /bin/bash \
    "$IMG" -c '
        source /opt/ros/humble/setup.bash
        echo "  ROS_DISTRO=$ROS_DISTRO  DOMAIN=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
        echo "  --- topic list ---"
        timeout 25 ros2 topic list 2>/dev/null | sed "s/^/    /"
        echo "  --- 深度影像實際頻率 ---"
        timeout 20 ros2 topic hz /camera/camera/depth/image_rect_raw 2>/dev/null | head -2 | sed "s/^/    /"
        echo "  --- TF 查得到嗎 ---"
        timeout 20 ros2 run tf2_ros tf2_echo odom camera_depth_optical_frame 2>&1 \
          | grep -v "Waiting for transform" | head -4 | sed "s/^/    /"
        echo "  --- GPU 看得到嗎 ---"
        nvidia-smi -L 2>/dev/null | sed "s/^/    /" || echo "    (nvidia-smi 在 Jetson 上不適用,改看 CUDA)"
        python3 -c "import os; print(\"    /dev/nvhost-gpu:\", os.path.exists(\"/dev/nvhost-gpu\"))"
    '
