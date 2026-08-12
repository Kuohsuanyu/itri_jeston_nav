#!/usr/bin/env bash
# 啟動底盤測試介面(idempotent —— 已經在跑就不重複啟動)
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash 2>/dev/null
source ~/chassis_ws/install/setup.bash     # chassis_msgs 的自訂型別
export ROS_DOMAIN_ID=0

if pgrep -f "chassis_console.py" > /dev/null 2>&1; then
    echo "  已在執行(PID $(pgrep -f chassis_console.py | head -1))"
    exit 0
fi

cd ~/chassis_test
setsid nohup python3 chassis_console.py > /tmp/chassis_console.log 2>&1 < /dev/null &
sleep 4
pgrep -f chassis_console.py > /dev/null && echo "  已啟動" || { echo "  ✗ 啟動失敗"; tail -20 /tmp/chassis_console.log; }
