#!/usr/bin/env bash
# 2D occupancy map:FAST-LIO -> /scan -> slam_toolbox -> /map
#
#   map ─(slam_toolbox)─ multi_odom ─(EKF)─ base_footprint ─(底盤rsp)─ base_link
#                                                                          └─ box_link ─ body
#
# TF 全部由 robot_tf.sh 統一發布。這裡只負責 /scan 和 slam_toolbox。
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

D=~/slam2d
LOG=/tmp/slam2d.log
: > "$LOG"

# ── ★ 先停定位模式,兩者互斥 ─────────────────────────────────────
# map -> multi_odom 只能有一個發布者。slam_toolbox 和 AMCL 都要發那一段,
# 兩個都跑的話 tf2 **不會報錯** —— 它會交替採用兩個答案,車的位姿以幾十 Hz
# 在兩處之間翻轉。slam_toolbox 拿到的掃描位置全是亂的,地圖根本建不起來。
#
# 2026-08-13 實測就是這樣:切到建圖模式後 AMCL 沒被停掉,
#     map -> multi_odom  57.1 Hz     (單一發布者應該是 20 Hz 上下)
#     /map  113 x 232 格,量三次完全沒長大
# 而且 /map 也有兩個發布者(slam_toolbox 的新圖 + map_server 的舊圖),
# RViz 顯示的是先到的那個,看起來就像「二維地圖沒有建出來」。
#
# start_localization.sh 一直都會先停 slam_toolbox,但反方向漏了 ——
# 這裡補上,讓兩支腳本對稱。
echo "[0/3] 停掉定位模式(AMCL / map_server 跟 slam_toolbox 互斥)"
STOPPED=0
for p in "nav2_amcl/amcl" "nav2_map_server/map_server" "lifecycle_manager.*localization"; do
    pkill -f "$p" 2>/dev/null && { echo "    停 $p"; STOPPED=1; }
done
[ "$STOPPED" = "1" ] && sleep 4 || echo "    本來就沒跑"

echo "[1/3] TF:robot_tf.sh(EKF + base_link -> box_link -> body / camera_link)"
# 方向一律是「往下接」,不要反過來發 body -> base_link。
# base_link 的父節點是樹莓派 robot_state_publisher 在發的,反過來發等於再
# 宣告一次 base_link 是自己的子節點 —— tf2 對同一個子節點只留一筆 static,
# 誰後發誰贏而且不報錯。實測兩次相同查詢隔五分鐘得到 x=+0.831 和 x=+0.202。
# ★ 不要用 `| sed` 縮排。robot_tf.sh 會 spawn 一堆背景行程,只要其中
#   任何一個還握著 pipe 的寫入端,sed 就等不到 EOF,整條腳本永遠卡住。
#   2026-08-12 實測卡了 7 分鐘,而 robot_tf.sh 本身早就結束了。
#   寫檔案再 cat 出來就沒有這個問題。
bash "$D/robot_tf.sh" > /tmp/robot_tf.log 2>&1
sed 's/^/    /' /tmp/robot_tf.log

# ★ 高度帶和 range_min 都在 start_scan.sh 裡 —— 建圖和定位共用同一支,
#   不要在這裡複製一份。兩份會漂,而且漂了不會有任何錯誤訊息。
# ★ 不要 `| sed` 縮排:那支會 spawn 背景行程,pipe 等不到 EOF 就卡死。
echo "[2/3] /scan"
bash "$D/start_scan.sh" > /tmp/start_scan.log 2>&1
sed 's/^/  /' /tmp/start_scan.log

# 記憶體上限:位姿圖失控時只殺 slam_toolbox,不要讓全域 OOM killer
# 掃到整塊板子(2026-08-05 就是這樣被殺的)。
echo "[3/3] slam_toolbox (async, 1.5GB 上限)"
pkill -f async_slam_toolbox_node 2>/dev/null; sleep 1
systemctl --user reset-failed slam2d-run.scope 2>/dev/null
if command -v systemd-run > /dev/null 2>&1; then
    systemd-run --user --scope -p MemoryMax=1500M --unit=slam2d-run --quiet \
      ros2 run slam_toolbox async_slam_toolbox_node \
      --ros-args --params-file "$D/slam_params.yaml" \
      >> "$LOG" 2>&1 < /dev/null &
else
    setsid nohup ros2 run slam_toolbox async_slam_toolbox_node \
      --ros-args --params-file "$D/slam_params.yaml" \
      >> "$LOG" 2>&1 < /dev/null &
fi
sleep 12

echo
echo "=== status ==="
for p in static_transform_publisher pointcloud_to_laserscan_node async_slam_toolbox_node; do
  n=$(pgrep -fc "$p")
  [ "$n" -gt 0 ] && echo "  [OK]   $p (x$n)" || echo "  [DEAD] $p"
done

echo
echo "=== TF chain map -> base_footprint ==="
timeout 12 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 \
  | grep -vE "Waiting for transform|signal_handler" | head -4

echo
echo "=== /scan ==="
timeout 12 ros2 topic hz /scan 2>&1 | head -3

echo
echo "=== /map ==="
timeout 20 ros2 topic echo /map --once --field info 2>&1 | head -12
