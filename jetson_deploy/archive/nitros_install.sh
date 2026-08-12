#!/usr/bin/env bash
# Install the prebuilt NITROS/GXF apt packages and bake them into the image.
#
# nvblox_ros/package.xml requires isaac_ros_managed_nitros,
# isaac_ros_nitros_image_type, isaac_ros_nitros_camera_info_type and
# isaac_ros_gxf. All exist as prebuilt 3.2.5 debs inside the container, so
# building NITROS (and GXF) from source is unnecessary.
#
# The container runs with --rm, so install in a NAMED container and
# `docker commit` it -- far quicker than rebuilding the Dockerfile chain.
set -e
NAME=isaac_nitros_setup
IMG=isaac_ros_dev-aarch64:latest
NEWIMG=isaac_ros_dev-aarch64:nvblox

docker rm -f "$NAME" 2>/dev/null || true

echo "=== 在具名容器裡安裝 NITROS / GXF ==="
docker run --name "$NAME" \
    --network host --privileged --runtime nvidia \
    -v /mnt/ssd/ws:/workspaces/isaac_ros-dev \
    --entrypoint /bin/bash "$IMG" -c '
set -e
apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    ros-humble-isaac-ros-gxf \
    ros-humble-isaac-ros-nitros \
    ros-humble-isaac-ros-managed-nitros \
    ros-humble-isaac-ros-nitros-image-type \
    ros-humble-isaac-ros-nitros-camera-info-type 2>&1 | tail -8
echo "--- 安裝結果 ---"
ls /opt/ros/humble/share | grep -iE "nitros|gxf" | sed "s/^/  /"
'

echo
echo "=== commit 成新映像 $NEWIMG ==="
docker commit "$NAME" "$NEWIMG" | sed 's/^/  /'
docker rm -f "$NAME" > /dev/null
docker image ls | grep isaac | sed 's/^/  /'
df -h /mnt/ssd | tail -1 | sed 's/^/  /'
