#!/usr/bin/env bash
# 正確的 sudo 寫入 sysfs 寫法:
#   echo PW | sudo -S sh -c 'echo VALUE > /path'
# sudo 從 stdin 讀密碼,實際寫入在 sh -c 裡面完成,兩者不搶 stdin。
#
# 錯誤寫法(我犯過兩次):
#   echo VALUE | sudo -S tee /path      <- VALUE 被密碼蓋掉,寫進去的是空的
PW=2919

DEV=""
for d in /sys/bus/usb/devices/*/; do
    if [ -f "$d/idVendor" ] && [ "$(cat $d/idVendor 2>/dev/null)" = "8086" ]; then
        DEV=$(basename "$d")
    fi
done
[ -z "$DEV" ] && { echo "找不到 Intel USB 裝置"; exit 1; }
echo "=== 目標裝置: $DEV ==="

echo "--- 斷電 ---"
echo "$PW" | sudo -S -p "" sh -c "echo 0 > /sys/bus/usb/devices/$DEV/authorized"
sleep 4
lsusb | grep -ci "8086" | sed 's/^/  Intel 裝置數: /'

echo "--- 上電 ---"
echo "$PW" | sudo -S -p "" sh -c "echo 1 > /sys/bus/usb/devices/$DEV/authorized"
sleep 8

echo
echo "=== reset 後 ==="
lsusb | grep -i intel | sed 's/^/  /' || echo "  ✗ 裝置沒回來"
ls /dev/video* 2>/dev/null | tr '\n' ' ' | sed 's/^/  /'; echo

echo
echo "=== 相機是否可正常開啟 ==="
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
timeout 40 rs-enumerate-devices 2>&1 \
  | grep -E "Name |Firmware Version|Usb Type Descriptor" | sed 's/^/  /' \
  || echo "  ✗ 仍然讀不到"
