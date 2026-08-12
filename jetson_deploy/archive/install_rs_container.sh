#!/usr/bin/env bash
# Install realsense2_camera INSIDE the Isaac container and commit.
#
# The camera has to move into the container: nvblox's NITROS subscriber only
# activates after negotiating with a NITROS-aware publisher, which the host's
# plain realsense2_camera never provides. Same apt version as the host (4.58.2),
# so behaviour is identical -- only the process boundary changes.
set -e
NAME=isaac_rs_setup
BASE=isaac_ros_dev-aarch64:nvblox
NEW=isaac_ros_dev-aarch64:nvblox-rs

docker rm -f "$NAME" 2>/dev/null || true

echo "=== 在容器裡安裝 realsense2_camera ==="
docker run --name "$NAME" --network host --privileged --runtime nvidia \
    -v /mnt/ssd/ws:/workspaces/isaac_ros-dev \
    --entrypoint /bin/bash "$BASE" -c '
set -e
apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs 2>&1 | tail -5
echo "--- 版本 ---"
dpkg -l | grep realsense | awk "{print \"  \"\$2\" \"\$3}"
echo "--- composable plugin 有註冊嗎 ---"
grep -r "RealSenseNodeFactory" /opt/ros/humble/share/realsense2_camera/ 2>/dev/null | head -2 | sed "s/^/  /"
'

echo
echo "=== commit -> $NEW ==="
docker commit "$NAME" "$NEW" | sed 's/^/  /'
docker rm -f "$NAME" > /dev/null
docker image ls | grep isaac | sed 's/^/  /'
