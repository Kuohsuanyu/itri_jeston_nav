#!/usr/bin/env bash
# 核心報 -71 (EPROTO) / -32 (EPIPE),是 USB 傳輸層錯誤。
# 彩色能開、深度不能,而深度多了一個紅外投影器(雷射)的電流。
#
# 決定性測試:關掉投影器 + 最低解析度。
#   能開  -> 電流/頻寬邊界問題,可用降規格繞過
#   不能開 -> 實體連線(線材/接頭)問題,軟體救不了
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

docker rm -f nvblox 2>/dev/null || true
pkill -f realsense2_camera_node 2>/dev/null || true
sleep 5

run_test() {
    DESC="$1"; W="$2"; H="$3"; F="$4"; EMIT="$5"
    echo
    echo "=== $DESC ==="
    pkill -f realsense2_camera_node 2>/dev/null || true
    sleep 4
    setsid nohup ros2 launch realsense2_camera rs_launch.py \
        enable_depth:=true enable_color:=false \
        enable_infra1:=false enable_infra2:=false \
        depth_module.depth_profile:="${W}x${H}x${F}" \
        depth_module.emitter_enabled:="$EMIT" \
        pointcloud.enable:=false align_depth.enable:=false \
        > /tmp/rs_test.log 2>&1 < /dev/null &
    sleep 22

    if grep -qi "stream start failure\|hardware error" /tmp/rs_test.log; then
        echo "  ✗ 啟動失敗(hardware error)"
    else
        echo "  啟動沒報錯"
    fi

    N=$(timeout 15 python3 - <<PY
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
c = [0]
rclpy.init(); n = Node("t")
n.create_subscription(Image, "/camera/camera/depth/image_rect_raw",
                      lambda m: c.__setitem__(0, c[0]+1), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.1)
print(c[0])
PY
)
    echo "  12 秒內收到 ${N:-0} 幀深度"
}

# emitter 0 = 關閉投影器,1 = 開啟
run_test "最低負載 + 投影器關閉" 424 240 6 0
run_test "最低負載 + 投影器開啟" 424 240 6 1
run_test "我們要用的設定 640x480x15 + 投影器開啟" 640 480 15 1

echo
echo "=== 測試期間的核心錯誤 ==="
echo 2919 | sudo -S -p "" dmesg 2>/dev/null | grep -iE "uvcvideo|usb 2-1.3" | tail -8 | sed 's/^/  /'
pkill -f realsense2_camera_node 2>/dev/null || true
