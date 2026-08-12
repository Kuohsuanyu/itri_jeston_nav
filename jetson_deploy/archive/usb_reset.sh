#!/usr/bin/env bash
# "Depth stream start failure ... Hardware Error" 通常不是相機壞了,
# 而是前一個持有者沒把 USB 裝置乾淨釋放就又被開啟。
# 反覆重啟容器很容易踩到(這一輪重開了四五次)。
#
# 順序:停所有持有者 -> USB 層 reset -> 確認裝置回來 -> 再啟動
PW=2919
sudo_() { echo "$PW" | sudo -S -p "" "$@"; }

echo "=== 停掉所有可能持有相機的東西 ==="
docker rm -f nvblox 2>/dev/null && echo "  停 nvblox 容器" || true
pkill -f realsense2_camera_node 2>/dev/null && echo "  停 host realsense" || true
pkill -9 -f "python3 cam_server.py" 2>/dev/null && echo "  停 cam_server" || true
sleep 5

echo
echo "=== USB 裝置現況 ==="
lsusb | grep -i intel | sed 's/^/  /'
for d in /sys/bus/usb/devices/*/; do
    if [ -f "$d/idVendor" ] && [ "$(cat $d/idVendor 2>/dev/null)" = "8086" ]; then
        DEV=$(basename "$d")
        echo "  裝置 $DEV  speed=$(cat $d/speed 2>/dev/null)"
        echo "=== reset $DEV ==="
        echo "$DEV" | sudo_ tee /sys/bus/usb/drivers/usb/unbind > /dev/null 2>&1 || true
        sleep 3
        echo "$DEV" | sudo_ tee /sys/bus/usb/drivers/usb/bind > /dev/null 2>&1 || true
        sleep 5
    fi
done

echo
echo "=== reset 後 ==="
lsusb | grep -i intel | sed 's/^/  /' || echo "  ✗ 裝置不見了"
ls /dev/video* 2>/dev/null | tr '\n' ' ' | sed 's/^/  /'; echo

echo
echo "=== 確認相機可用 ==="
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
timeout 30 rs-enumerate-devices 2>&1 | grep -E "Name|Firmware|Usb Type" | sed 's/^/  /' \
    || echo "  ✗ rs-enumerate-devices 讀不到"
