#!/usr/bin/env bash
# 清空 2D 地圖,重新開始蒐集。
#
# slam_toolbox 2.6.10 沒有提供 reset 服務(只有 clear_changes,那是清手動修正
# 不是清地圖),所以唯一可靠的做法就是把節點重啟。
# static TF 和 pointcloud_to_laserscan 不用動,重啟它們反而會讓 TF 斷一下。
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

pkill -f async_slam_toolbox_node 2>/dev/null
systemctl --user reset-failed 2>/dev/null
sleep 3

# 記憶體上限:位姿圖失控時只殺 slam_toolbox,不要再讓全域 OOM killer
# 掃到整塊板子(2026-08-05 就是這樣被殺的,連帶影響其他服務)。
systemd-run --user --scope -p MemoryMax=1500M --unit=slam2d-run --quiet \
  ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args --params-file "$HOME/slam2d/slam_params.yaml" \
  >> /tmp/slam2d.log 2>&1 < /dev/null &

sleep 8
pgrep -f async_slam_toolbox_node > /dev/null && echo OK || echo FAILED
