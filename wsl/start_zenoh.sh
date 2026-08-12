#!/usr/bin/env bash
# 啟動 WSL 端的 zenoh bridge(idempotent —— 已經在跑就不重複啟動)
#
# 不能用 set -u:ROS 的 setup.bash 會讀未初始化變數。
source "$HOME/lidar_view/ros_env.sh"

LOG=/tmp/zenoh_wsl.log

if pgrep -f "zenoh-bridge-ros2dds" > /dev/null 2>&1; then
    echo "  zenoh bridge 已在執行(PID $(pgrep -f zenoh-bridge-ros2dds | head -1))"
    exit 0
fi

if ! command -v zenoh-bridge-ros2dds > /dev/null 2>&1; then
    echo "  ✗ 找不到 zenoh-bridge-ros2dds"
    echo "    安裝方式見 wsl/README_zenoh.md"
    exit 1
fi

echo "  啟動 zenoh bridge:domain $ROS_DOMAIN_ID -> $ZENOH_JETSON"

# --no-multicast-scouting:區網上沒有別的 zenoh 節點,關掉省得亂連。
# 連線是 WSL 主動撥出的 TCP,所以完全不需要 Windows 入站防火牆規則。
RUST_LOG=info setsid nohup zenoh-bridge-ros2dds \
    -e "$ZENOH_JETSON" --no-multicast-scouting \
    > "$LOG" 2>&1 < /dev/null &

# 等連線建立。TCP 撥出很快,但 DDS 端的路由建立要幾秒。
for i in $(seq 1 15); do
    sleep 1
    if grep -q "Route Subscriber" "$LOG" 2>/dev/null; then
        echo "  已連上,開始建立 topic 路由"
        exit 0
    fi
done

echo "  ⚠ 15 秒內沒看到路由建立,log:$LOG"
tail -5 "$LOG"
