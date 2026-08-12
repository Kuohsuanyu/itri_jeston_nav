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

# ---- /scan 的高度帶 ----------------------------------------------------
# ★ 2026-08-11:target_frame 從 box_link 改成 base_footprint。
#   兩棵樹併成一棵之後 base_footprint 查得到了,而它**就在地面上**(z=0),
#   所以高度帶可以直接寫離地高度,不用再減任何偏移 —— 少一個換算就少一個
#   改了一半的機會。舊值 -0.3542 / 1.0458 是「離地 0.10/1.50 減掉 box_link
#   離地 0.4542」,換算對了但完全看不出來在幹嘛。
MIN_H=0.10
MAX_H=1.50
echo "  /scan 高度帶 $MIN_H ~ $MAX_H  (相對 base_footprint,就是離地高度)"

# ---- range_min:把車子自己濾掉 ------------------------------------------
# 高度帶(離地 0.10~1.50)會**涵蓋車體自己的上蓋**
# —— 上蓋離地 0.4562,正好落在帶子中間。光達裝在上蓋上方 0.199 m,俯視 37 度
# 時打得到上蓋外緣(斜距約 0.56 m,超過 FAST-LIO 的 blind 0.5),那圈點會變成
# 一圈假障礙物,Nav2 會以為自己被包圍。
#
# 車體 1.04 x 0.78 m,從 base_footprint 算半對角線 sqrt(0.52^2+0.39^2)=0.65 m,
# 所以 range_min 取 0.70 就切乾淨。代價是 0.70 m 內看不到東西,但那已經在
# Nav2 的 footprint 裡面了,本來就不該靠 /scan 處理。
RANGE_MIN=0.70

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

# 用 body-frame 的點雲,不能用 /cloud_registered(那是世界座標系,
# pointcloud_to_laserscan 需要感測器座標系的雲)。
echo "[2/3] pointcloud_to_laserscan  /cloud_registered_body -> /scan"
pkill -f pointcloud_to_laserscan_node 2>/dev/null; sleep 1
setsid nohup ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/cloud_registered_body \
  -r scan:=/scan \
  -p target_frame:=base_footprint \
  -p transform_tolerance:=0.20 \
  -p queue_size:=30 \
  -p min_height:=$MIN_H \
  -p max_height:=$MAX_H \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.0087 \
  -p scan_time:=0.1 \
  -p range_min:=$RANGE_MIN \
  -p range_max:=40.0 \
  -p use_inf:=true \
  >> "$LOG" 2>&1 < /dev/null &
sleep 4

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
