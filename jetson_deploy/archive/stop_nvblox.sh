#!/usr/bin/env bash
# Tear down the Isaac ROS / nvblox side. ASCII only (base64|bash mangles CJK).
#
# WHY: nvblox contributed nothing to this build. Nav2 plans on the 2D map that
# comes from the lidar; nvblox's ESDF was never wired into the costmap. Its only
# unique output was a colored mesh, and that is now done far more cheaply by
# projecting the lidar cloud into the D435 color image (lidar_web/server.py).
#
# The container ALSO owns the RealSense: start_nvblox_rs.sh runs the camera
# driver inside it at namespace /camera0/camera. So this must run before the
# host-side realsense2_camera can claim the USB device again.
#
# NOT deleted: /mnt/ssd/ws and the isaac_ros_dev image. Rebuilding those cost
# most of a day. Keep them in case dynamic obstacle avoidance is wanted later.

echo "=== containers before ==="
docker ps -a --format '  {{.Names}}  {{.Status}}' 2>/dev/null

for c in nvblox nvblox_mesh_web isaac_ros_dev-aarch64-container; do
    if docker inspect "$c" > /dev/null 2>&1; then
        echo "  removing $c"
        docker rm -f "$c" > /dev/null 2>&1
    fi
done

# The RealSense USB device stays claimed for a moment after the container dies.
sleep 3

echo
echo "=== containers after ==="
docker ps -a --format '  {{.Names}}  {{.Status}}' 2>/dev/null
echo "  (none listed above = all clear)"

echo
echo "=== USB: is the D435 free? ==="
lsusb | grep -i intel | sed 's/^/  /' || echo "  WARN: no Intel USB device found"
fuser -v /dev/bus/usb/*/* 2>/dev/null | head -5 | sed 's/^/  /'

echo
echo "=== memory freed ==="
free -m | head -2 | sed 's/^/  /'
