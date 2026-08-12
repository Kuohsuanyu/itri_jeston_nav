#!/usr/bin/env bash
# 開啟 RViz2 顯示 Jetson 上 FAST-LIO 的建圖結果
#
# 注意:這裡不能用 set -u。ROS 的 /opt/ros/humble/setup.bash 內部會讀取
# AMENT_TRACE_SETUP_FILES 等未初始化變數,開了 nounset 會讓腳本一啟動就中止。
source "$HOME/lidar_view/ros_env.sh"

JETSON=192.168.40.98

echo "=============================================="
echo "  FAST-LIO 遠端檢視  (WSL2 + RViz2 + zenoh)"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  (Jetson 在 domain 0,靠 zenoh 橋接)"
echo "=============================================="

echo "[1/4] 檢查與 Jetson 的連線"
if ping -c 2 -W 2 "$JETSON" > /dev/null 2>&1; then
    echo "      Jetson $JETSON 有回應"
else
    echo "      ✗ ping 不到 $JETSON"
    echo "        確認板子已上電、且與這台電腦在同一個網路"
    exit 1
fi

echo "[2/4] 啟動 zenoh bridge"
"$HOME/lidar_view/start_zenoh.sh"

echo "[3/4] 等待 topic 出現(最多 25 秒)"
found=0
for i in $(seq 1 25); do
    if ros2 topic list 2>/dev/null | grep -q "/scan"; then
        found=1; break
    fi
    sleep 1
done

if [ "$found" = "1" ]; then
    echo "      找到 /scan"
    ros2 topic list 2>/dev/null | sed 's/^/        /'
else
    echo "      ✗ 25 秒內沒看到 /scan"
    echo "        Jetson 上的 FAST-LIO 可能還沒啟動 —— 先在控制台按「一鍵啟動」"
    echo "        也確認 Jetson 端的 zenoh bridge 有在跑(見 README_zenoh.md)"
    echo "        (仍會開啟 RViz,topic 出現後畫面會自己跑起來)"
fi

echo "[4/4] 啟動 RViz2"
# 走 rviz_launch.sh:WSLg 上建 render window 會間歇性失敗,那支會重試
# 可以帶自己的設定檔:start_rviz.sh ~/lidar_view/my.rviz
# 不帶就用 live.rviz。live.rviz 是版控裡的正本,你在介面上調的東西
# 要 File -> Save Config As 存成別的檔名才留得住。
exec bash "$HOME/lidar_view/rviz_launch.sh" "${1:-$HOME/lidar_view/live.rviz}"
