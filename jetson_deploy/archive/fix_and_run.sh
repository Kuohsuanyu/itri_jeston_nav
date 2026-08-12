#!/usr/bin/env bash
# realsense2_camera 的 composable 版本需要 libdiagnostic_updater.so,
# 但 apt 沒把它當硬相依拉進來 —— 只有在 dlopen 的時候才會發現。
set -e
docker rm -f isaac_fix 2>/dev/null || true

echo "=== 補裝 diagnostic_updater ==="
docker run --name isaac_fix --network host \
    --entrypoint /bin/bash isaac_ros_dev-aarch64:nvblox-rs -c '
apt-get update -qq 2>/dev/null
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    ros-humble-diagnostic-updater ros-humble-diagnostic-msgs 2>&1 | tail -3
if ls /opt/ros/humble/lib/libdiagnostic_updater.so >/dev/null 2>&1; then
    echo "  libdiagnostic_updater.so 就位"
else
    echo "  找不到 libdiagnostic_updater.so:"
    find / -name "libdiagnostic_updater*" 2>/dev/null | head -3
fi
'

docker commit isaac_fix isaac_ros_dev-aarch64:nvblox-rs | sed 's/^/  /'
docker rm -f isaac_fix > /dev/null

echo
bash ~/start_nvblox_rs.sh
