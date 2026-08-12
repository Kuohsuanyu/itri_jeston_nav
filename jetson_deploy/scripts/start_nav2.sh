#!/usr/bin/env bash
# 只啟動 Nav2 的全域路徑規劃(planner_server + global_costmap)。
#
# 沒接底盤,所以不需要 controller_server / bt_navigator / behavior_server。
# 由 map_server.py 直接呼叫 ComputePathToPose action 取得路徑,畫在網頁上。
#
# 相依:slam_toolbox 要先在跑(提供 /map 和 map->odom),
#       pointcloud_to_laserscan 要先在跑(提供 /scan)。
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

D=~/slam2d
LOG=/tmp/nav2.log
: > "$LOG"

echo "[前置檢查]"
if ! pgrep -f async_slam_toolbox_node > /dev/null; then
    echo "  ✗ slam_toolbox 沒在跑 —— 沒有 /map 和 map->odom,規劃一定失敗"
    exit 1
fi
timeout 12 ros2 topic info /scan 2>/dev/null | grep -q "Publisher count: 1" \
    && echo "  /scan OK" || echo "  ⚠ /scan 可能沒有 publisher"

pkill -f "nav2_planner|planner_server"        2>/dev/null
pkill -f lifecycle_manager                    2>/dev/null
systemctl --user reset-failed 2>/dev/null
sleep 3

# costmap 會依地圖大小配置記憶體,包個上限以防萬一
echo "[1/2] planner_server + global_costmap"
if command -v systemd-run > /dev/null 2>&1; then
    systemd-run --user --scope -p MemoryMax=1G --unit=nav2-planner --quiet \
      ros2 run nav2_planner planner_server \
      --ros-args --params-file "$D/nav2_planner.yaml" \
      >> "$LOG" 2>&1 < /dev/null &
else
    setsid nohup ros2 run nav2_planner planner_server \
      --ros-args --params-file "$D/nav2_planner.yaml" \
      >> "$LOG" 2>&1 < /dev/null &
fi
sleep 6

# lifecycle_manager 負責把 planner_server 從 unconfigured 帶到 active。
# 少了它,節點會起來但 action server 永遠不出現 —— 這是最常見的卡點。
echo "[2/2] lifecycle_manager(負責 configure + activate)"
setsid nohup ros2 run nav2_lifecycle_manager lifecycle_manager \
  --ros-args -r __node:=lifecycle_manager_planner \
  --params-file "$D/nav2_planner.yaml" \
  >> "$LOG" 2>&1 < /dev/null &
sleep 12

echo
echo "=== 狀態 ==="
pgrep -f planner_server   > /dev/null && echo "  [OK]   planner_server"   || echo "  [DEAD] planner_server"
pgrep -f lifecycle_manager > /dev/null && echo "  [OK]   lifecycle_manager" || echo "  [DEAD] lifecycle_manager"

echo "  生命週期狀態: $(timeout 10 ros2 lifecycle get /planner_server 2>&1 | head -1)"

echo
echo "=== action server 出現了嗎(這是能不能規劃的關鍵) ==="
timeout 15 ros2 action list 2>/dev/null | grep -i compute_path && echo "  ✓ 可以規劃" \
    || echo "  ✗ 還沒出現 —— 看 $LOG"

echo
echo "=== log 尾巴 ==="
tail -12 "$LOG"
