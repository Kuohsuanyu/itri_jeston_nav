#!/usr/bin/env bash
# Build nvblox_ros inside the Isaac ROS container.
#
# The base image is prebuilt, but nvblox itself compiles CUDA from source.
# Parallelism is capped at 2: nvcc is memory hungry and this board has 8 GB
# and a history of OOM kills. Slower is better than a build that dies at 80%.
LOG=/mnt/ssd/ws/nvblox_build.log

echo "=== 停掉光達相關服務(編譯期間 CPU 會飽和,會讓 FAST-LIO 發散) ==="
for p in fastlio_mapping async_slam_toolbox_node pointcloud_to_laserscan \
         "python3 server.py" "python3 map_server.py" "python3 cam_server.py" \
         zenoh-bridge-ros2dds realsense2_camera_node livox_ros_driver2; do
    pkill -9 -f "$p" 2>/dev/null && echo "  停 $p" || true
done
sleep 5
cat /proc/loadavg | awk '{print "  loadavg "$1}'
free -m | head -2 | sed 's/^/  /'

echo
echo "=== 開始編譯 $(date) ===" | tee "$LOG"

docker run --rm \
    --network host --ipc=host --privileged --runtime nvidia \
    -e ROS_DOMAIN_ID=0 \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /mnt/ssd/ws:/workspaces/isaac_ros-dev \
    -v /dev:/dev \
    --workdir /workspaces/isaac_ros-dev \
    --entrypoint /bin/bash \
    isaac_ros_dev-aarch64:nvblox -c '
set -o pipefail
source /opt/ros/humble/setup.bash

echo "--- rosdep 補相依 ---"
rosdep update --rosdistro humble 2>&1 | tail -2
rosdep install --from-paths src --ignore-src -y --rosdistro humble \
    --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" \
    2>&1 | tail -6

echo
echo "--- colcon build(並行度 2) ---"
colcon build \
    --symlink-install \
    --packages-up-to nvblox_ros nvblox_examples_bringup \
    --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+ 2>&1
' >> "$LOG" 2>&1

RC=$?
echo "=== 編譯結束 exit=$RC  $(date) ===" | tee -a "$LOG"
tail -25 "$LOG"
echo
echo "--- 產出 ---"
ls -1 /mnt/ssd/ws/install 2>/dev/null | head -20 | sed 's/^/  /'
df -h /mnt/ssd | tail -1 | sed 's/^/  /'
exit $RC
