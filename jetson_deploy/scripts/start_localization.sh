#!/usr/bin/env bash
# 載入既有地圖並定位 —— 回答「我現在在地圖的哪裡」。
#
#   bash start_localization.sh                        用最新的地圖
#   bash start_localization.sh ~/maps/map_xxx.yaml    指定地圖
#   bash start_localization.sh <地圖> <x> <y> <yaw度>  順便給初始位置
#
# ── 跟建圖模式的關係 ──────────────────────────────────────────────
#   建圖  slam_toolbox   一邊建 /map 一邊發 map -> multi_odom
#   定位  map_server     發固定的 /map
#         amcl           發 map -> multi_odom
#
# ★ 兩者互斥。map -> multi_odom 只能有一個發布者 —— 兩個都跑的話
#   base_footprint 到 map 的路徑會有兩個答案,tf2 不報錯只會隨機翻轉,
#   症狀是車在地圖上瞬移。所以這支會先把 slam_toolbox 停掉。
#
# ── 前提 ────────────────────────────────────────────────────────
#   startall.sh 和 robot_tf.sh 已經跑過(要有 /scan 和 multi_odom -> base_footprint)
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

D=~/slam2d
MAP="${1:-$(ls -t ~/maps/*.yaml 2>/dev/null | head -1)}"
IX="$2"; IY="$3"; IYAW="$4"
LOG=/tmp/localization.log
: > "$LOG"

[ -f "$MAP" ] || { echo "✗ 找不到地圖:$MAP"; echo "  ~/maps 裡有:"; ls -1 ~/maps/*.yaml 2>/dev/null | sed 's/^/    /'; exit 1; }
echo "=== 地圖:$MAP ==="
sed 's/^/    /' "$MAP"
PGM=$(dirname "$MAP")/$(grep "^image:" "$MAP" | awk '{print $2}')
[ -f "$PGM" ] && echo "    尺寸 $(head -c 20 "$PGM" | tr '\n' ' ' | awk '{print $2" x "$3}') 格" || echo "    ✗ 找不到 $PGM"

echo
echo "[1/5] 停掉 slam_toolbox(它跟 AMCL 都要發 map -> multi_odom)"
pkill -f async_slam_toolbox_node 2>/dev/null && echo "    已停" || echo "    本來就沒跑"
pkill -f "nav2_map_server|map_server --ros-args" 2>/dev/null
pkill -f "amcl --ros-args" 2>/dev/null
pkill -f "lifecycle_manager.*localization" 2>/dev/null
sleep 3

echo
echo "[2/5] 前置檢查"
fail=0
hz() { timeout 8 ros2 topic hz "$1" 2>&1 | grep -oE "average rate: [0-9.]+" | head -1 | grep -oE "[0-9.]+"; }
S=$(hz /scan)
[ -n "$S" ] && echo "    /scan  $S Hz" || { echo "    ✗ /scan 沒資料 —— 先跑 start_slam2d.sh"; fail=1; }
if timeout 10 ros2 run tf2_ros tf2_echo multi_odom base_footprint 2>&1 | grep -q Translation; then
    echo "    multi_odom -> base_footprint OK"
else
    echo "    ✗ 查不到 multi_odom -> base_footprint —— EKF 沒起來"; fail=1
fi
[ "$fail" = "1" ] && { echo; echo "前置條件不足,中止"; exit 1; }

echo
echo "[3/5] map_server"
setsid nohup ros2 run nav2_map_server map_server --ros-args \
    --params-file "$D/localization.yaml" \
    -p yaml_filename:="$MAP" \
    >> "$LOG" 2>&1 < /dev/null &
sleep 5

echo "[4/5] amcl"
setsid nohup ros2 run nav2_amcl amcl --ros-args \
    --params-file "$D/localization.yaml" \
    >> "$LOG" 2>&1 < /dev/null &
sleep 5

echo "[5/5] lifecycle_manager(把兩個節點推到 active)"
# ★ Nav2 的節點都是 lifecycle node,起來之後停在 unconfigured,
#   不 configure + activate 的話它們什麼都不做,而且**不會報錯** ——
#   看起來行程活著、topic 也列得出來,但 /map 是空的、AMCL 不發 TF。
setsid nohup ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -p node_names:="[map_server, amcl]" \
    -p autostart:=true \
    -p bond_timeout:=0.0 \
    -r __node:=lifecycle_manager_localization \
    >> "$LOG" 2>&1 < /dev/null &
sleep 12

echo
echo "=== 狀態 ==="
for p in "map_server" "amcl" "lifecycle_manager"; do
    n=$(pgrep -fc "$p" 2>/dev/null | head -1)
    [ "$n" -gt 0 ] && echo "    [OK]   $p" || echo "    [DEAD] $p"
done

# 初始位置。AMCL 不知道你從哪開始 —— 沒給的話粒子散在整張圖上,
# 在長廊這種到處長得一樣的環境幾乎收斂不了。
if [ -n "$IX" ] && [ -n "$IY" ]; then
    echo
    echo "=== 設定初始位置 ($IX, $IY, ${IYAW:-0}°) ==="
    Y=$(python3 -c "import math;print(math.sin(math.radians(${IYAW:-0})/2))")
    W=$(python3 -c "import math;print(math.cos(math.radians(${IYAW:-0})/2))")
    timeout 20 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
      "{header: {frame_id: map}, pose: {pose: {position: {x: $IX, y: $IY, z: 0.0},
        orientation: {z: $Y, w: $W}},
        covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                     0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}" 2>&1 | tail -1
    sleep 4
fi

echo
echo "=== 驗證 ==="
echo "--- /map ---"
timeout 20 ros2 topic echo /map --once --field info 2>&1 | grep -E "width|height|resolution" | sed 's/^/    /'
echo "--- map -> base_footprint(這就是「我在地圖的哪裡」)---"
timeout 15 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 \
  | grep -vE "Waiting for transform|signal_handler" | head -4 | sed 's/^/    /'
echo "--- AMCL 的位姿估計 ---"
timeout 15 ros2 topic echo /amcl_pose --once --field pose.pose.position 2>&1 | head -4 | sed 's/^/    /'

echo
cat <<'MSG'
=== 接下來 ===
沒給初始位置、或位置不準的話,在 RViz 用工具列的「2D Pose Estimate」
在地圖上點一下車的實際位置、拖出朝向。AMCL 會收斂。

RViz 要看的:
    Map        /map                 灰底的既有地圖
    Scan       /scan                紅點。★ 紅點要貼在地圖的牆上,
                                      沒貼上就是定位還沒收斂
    PoseWithCovariance  /amcl_pose  估計位置 + 不確定度橢圓
    PoseArray  /particle_cloud      粒子雲。散開 = 還不確定,聚攏 = 收斂了

要回到建圖模式:bash ~/slam2d/start_slam2d.sh
MSG
