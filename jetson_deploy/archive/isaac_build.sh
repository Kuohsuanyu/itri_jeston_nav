#!/usr/bin/env bash
# Build the Isaac ROS dev image (aarch64.ros2_humble).
#
# run_dev.sh hardcodes /bin/bash at the end so it is interactive only.
# This calls the same underlying builder that run_dev.sh line 205 uses,
# so the resulting image is identical -- just runnable in the background.
#
# The RealSense layer is deliberately NOT included: the camera driver runs on
# the host (apt realsense2_camera 4.58.2) and the container uses --network host
# + --ipc=host, so nvblox in the container can subscribe to the host's topics.
# That also sidesteps NVIDIA's librealsense version pin.
# NVIDIA 的腳本用 tput 做彩色輸出。背景執行沒有 TTY、$TERM 是空的,
# tput 會失敗並把整個腳本帶掉(實測 exit 2)。給它一個 TERM 就好。
export TERM=${TERM:-xterm-256color}

LOG=/mnt/ssd/ws/build.log
cd /mnt/ssd/ws/src/isaac_ros_common/scripts

echo "start $(date)" | tee "$LOG"
df -h /mnt/ssd | tail -1 | tee -a "$LOG"
free -m | head -2 | tee -a "$LOG"

./build_image_layers.sh \
    --image_key "aarch64.ros2_humble" \
    --image_name "isaac_ros_dev-aarch64" >> "$LOG" 2>&1
RC=$?

echo "=== exit $RC  $(date) ===" | tee -a "$LOG"
docker image ls | grep -i isaac | tee -a "$LOG"
df -h /mnt/ssd | tail -1 | tee -a "$LOG"
exit $RC
