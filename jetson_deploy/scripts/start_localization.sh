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
#   startall.sh(FAST-LIO)和 robot_tf.sh(EKF -> multi_odom)已經跑過。
#   /scan 不用先起 —— 沒有的話這支會自己叫 start_scan.sh。
#
# ── 初始位置 ────────────────────────────────────────────────────
#   這支**一定會**發初始位置,預設是地圖原點 map(0,0,0) —— 也就是當初
#   開始建圖的那個實體定點。標準流程:車停回那裡 -> 啟動 -> 直接就定位好。
#
#   不發的話 AMCL(set_initial_pose: false)不會發 map -> multi_odom,
#   RViz 的 Fixed Frame 是 map 就整個畫面錯誤 —— 而且會變成死結:
#   要在地圖上點位置得先看得到地圖,要看得到地圖又要先有位置。
#
#   車停在別處的話:
#       RViz 工具列「2D Pose Estimate」在圖上點實際位置、拖出朝向
#       或  bash start_localization.sh <地圖> <x> <y> <yaw度>
#       或  python3 waypoint.py init <航點名>
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

D=~/slam2d
MAP="${1:-$(ls -t ~/maps/*.yaml 2>/dev/null | head -1)}"
LOG=/tmp/localization.log
: > "$LOG"

# ── 初始位置:預設就是地圖原點 ────────────────────────────────────
# ★ 「初始位置」是地上的一個**實體定點** —— 就是當初開始建圖的那個位置。
#   地圖是以那裡為原點長出來的,所以車停在那裡時,map(0,0,0) 就是正解。
#
#   流程固定成:把車停回初始位置 -> 啟動 -> 直接就定位好了,不用再點。
#
#   為什麼要有預設值:AMCL 的 set_initial_pose 是 false,沒收到 /initialpose
#   之前它**不發 map -> multi_odom**。而 RViz 的 Fixed Frame 是 map,
#   查不到 map 就整個畫面都是錯誤狀態 —— 於是變成死結:要在地圖上點位置,
#   得先看得到地圖;要看得到地圖,又要先有位置。給預設值就解開了。
#
#   車**不是**停在初始位置的話,啟動後在 RViz 用 2D Pose Estimate 改,
#   或 bash start_localization.sh <地圖> <x> <y> <yaw度>
IX="${2:-0.0}"; IY="${3:-0.0}"; IYAW="${4:-0.0}"

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
if [ -z "$S" ]; then
    # ★ /scan 的產生器在 start_scan.sh,建圖和定位共用。
    #   bringup_all.sh 的 loc 模式不跑 start_slam2d.sh,所以在這裡自己起 ——
    #   2026-08-12 就是因為沒人起它,定位整條中止、map -> multi_odom 接不上。
    echo "    /scan 沒資料,啟動 pointcloud_to_laserscan"
    bash "$D/start_scan.sh" > /tmp/start_scan.log 2>&1
    sed 's/^/      /' /tmp/start_scan.log
    S=$(hz /scan)
fi
[ -n "$S" ] && echo "    /scan  $S Hz" || { echo "    ✗ /scan 還是沒資料 —— FAST-LIO 起來了嗎"; fail=1; }
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

echo
echo "=== 設定初始位置 map($IX, $IY, $IYAW°) ==="
if [ "$IX" = "0.0" ] && [ "$IY" = "0.0" ] && [ "$IYAW" = "0.0" ]; then
    echo "    預設值 = 地圖原點 = 建圖起點那個實體定點。"
    echo "    ★ 前提是車確實停在那裡。不是的話用 RViz 的 2D Pose Estimate 改。"
fi
# 共變異數給小的 = 「我確定在這裡」,不是「大概在附近」。
# 這條走廊 81 公尺、門洞週期性重複,給大的話粒子會散開然後鎖到隔壁門洞 ——
# 實測沿走廊滑動同一幀掃描,相距十幾公尺的三個位置分數一模一樣(0.0500)。
Y=$(python3 -c "import math;print(math.sin(math.radians($IYAW)/2))")
W=$(python3 -c "import math;print(math.cos(math.radians($IYAW)/2))")
timeout 20 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: $IX, y: $IY, z: 0.0},
    orientation: {z: $Y, w: $W}},
    covariance: [0.10,0,0,0,0,0, 0,0.10,0,0,0,0, 0,0,0,0,0,0,
                 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.03]}}" 2>&1 | tail -1
sleep 6

# ★ 驗證 map -> multi_odom 真的出現了。AMCL 的 /initialpose 訂閱是 RELIABLE,
#   而且它要收到下一幀 /scan 才會開始發 TF —— 沒驗證的話會以為設好了,
#   到 RViz 才發現 Fixed Frame [map] does not exist。
if timeout 12 ros2 run tf2_ros tf2_echo map multi_odom 2>&1 | grep -q Translation; then
    echo "    ✓ map -> multi_odom 已建立,TF 樹接起來了"
else
    echo "    ✗ map -> multi_odom 還是沒有 —— AMCL 沒吃到初始位置"
    echo "      檢查 /scan 有沒有在發,以及 ros2 lifecycle get /amcl 是不是 active"
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
初始位置已經設好(預設 = 地圖原點 = 建圖起點)。車停在別處的話,
在 RViz 用工具列的「2D Pose Estimate」點實際位置、拖出朝向。

RViz 要看的:
    Map        /map                 灰底的既有地圖
    Scan       /scan                紅點。★ 紅點要貼在地圖的牆上,
                                      沒貼上就是定位還沒收斂
    PoseWithCovariance  /amcl_pose  估計位置 + 不確定度橢圓
    PoseArray  /particle_cloud      粒子雲。散開 = 還不確定,聚攏 = 收斂了

要回到建圖模式:bash ~/slam2d/start_slam2d.sh
MSG
