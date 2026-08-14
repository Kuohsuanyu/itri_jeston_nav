#!/usr/bin/env bash
# 完整導航:在 RViz 點目標點,車自己規劃路線並開過去。
#
#   bash ~/slam2d/start_nav.sh          啟動
#   bash ~/slam2d/start_nav.sh stop     停掉(車會立刻停)
#   bash ~/slam2d/start_nav.sh dry      只起規劃器,不接速度 —— 看路徑但車不動
#
# ★★ 這會讓車**真的移動**。人要在旁邊,手放在遙控的停止鍵上。★★
#
# ── 前提 ────────────────────────────────────────────────────────
# 必須先在定位模式而且定位是準的:
#     bash ~/slam2d/robot.sh use ~/maps/<地圖>.yaml
#     python3 ~/slam2d/check_localization.py     命中率 > 70%
#
# 定位不準的話 Nav2 會規劃到錯的地方,而那時車是真的會動的。
#
# ── 已知的安全限制,啟動前務必知道 ──────────────────────────────
# 1. **車前 70 公分是盲區**(/scan 的 range_min = 0.70)。人走到很近時
#    反而從 costmap 上消失。速度設在 0.10 m/s 就是為了讓盲區不會變成撞擊。
# 2. 光達裝在盒子上、離地 0.73 m,高度帶切 0.10~1.50 m。
#    **比 10 公分矮的東西(門檻、腳、地上的線)看不到。**
# 3. 沒有後方感測。allow_reversing 已關,但復原行為的 backup 仍可能倒退
#    —— 那是最後手段,正常不會觸發。
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/slam2d/robot_env.sh
export ROS_DOMAIN_ID=0

D=~/slam2d
LOG=/tmp/nav2.log

NODES="controller_server planner_server behavior_server bt_navigator velocity_smoother"

stop_all() {
    echo "=== 停止導航 ==="
    # ★ 先送零速度再殺節點。反過來的話最後一筆非零速度會留在底盤上,
    #   車會繼續滑行到超時才停。
    timeout 8 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
        > /dev/null 2>&1
    echo "  已送零速度"
    pkill -f goal_to_plan.py 2>/dev/null && echo "  停 goal_to_plan"
    for n in $NODES lifecycle_manager_navigation; do
        pkill -f "$n" 2>/dev/null && echo "  停 $n"
    done
    sleep 3
    timeout 8 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
        > /dev/null 2>&1
    echo "  再送一次零速度(保險)"
}

case "${1:-start}" in
stop) stop_all; exit 0 ;;
esac

DRY=0
[ "${1:-}" = "dry" ] && DRY=1

echo "=== 前置檢查 ==="
FAIL=0
pgrep -f "nav2_amcl/amcl" > /dev/null \
    && echo "  ✓ AMCL 在跑" \
    || { echo "  ✗ AMCL 沒在跑 —— 先 bash ~/slam2d/robot.sh use <地圖>"; FAIL=1; }
pgrep -f async_slam_toolbox_node > /dev/null \
    && { echo "  ✗ slam_toolbox 在跑 —— 導航要用定位模式,不是建圖模式"; FAIL=1; }

H=$(timeout 10 ros2 topic hz /scan 2>&1 | grep -oE "average rate: [0-9.]+" | head -1 | grep -oE "[0-9.]+")
[ -n "$H" ] && echo "  ✓ /scan $H Hz" || { echo "  ✗ /scan 沒資料"; FAIL=1; }

if timeout 12 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -q Translation; then
    P=$(timeout 12 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -m1 Translation | sed 's/.*\[/[/')
    echo "  ✓ 我在地圖的 $P"
else
    echo "  ✗ 查不到 map -> base_footprint —— AMCL 收到初始位置了嗎"; FAIL=1
fi

# ★ 底盤要真的收得到速度指令。這條斷了的話 Nav2 會以為自己在開,
#   而車一動也不動 —— 而且 Nav2 不會報錯,只會超時後嘗試復原行為。
if timeout 15 ros2 node info /chassis_driver 2>/dev/null | grep -q "/cmd_vel"; then
    echo "  ✓ 底盤有訂閱 /cmd_vel"
else
    echo "  ⚠ 查不到底盤的 /cmd_vel 訂閱(可能只是 daemon 快取過期)"
    echo "     確認:ros2 node info /chassis_driver --no-daemon | grep cmd_vel"
fi

[ "$FAIL" = "1" ] && { echo; echo "前置條件不足,中止"; exit 1; }

echo
echo "=== 停掉舊的導航節點 ==="
stop_all > /dev/null 2>&1
sleep 2
: > "$LOG"

echo
if [ "$DRY" = "1" ]; then
    echo "=== 乾跑模式:只起規劃器,速度指令不會送到底盤 ==="
    echo "  車不會動。可以在 RViz 看規劃出來的路徑對不對。"
    NODES="planner_server"
    START="planner_server"
    DRY_BRIDGE=1
else
    echo "=== 完整導航(★ 車會動)==="
    echo "  速度上限 0.10 m/s / 0.15 rad/s —— 走路速度的五分之一"
    START="$NODES"
fi

for n in $START; do
    case "$n" in
      controller_server) PKG=nav2_controller ;;
      planner_server)    PKG=nav2_planner ;;
      behavior_server)   PKG=nav2_behaviors ;;
      bt_navigator)      PKG=nav2_bt_navigator ;;
      velocity_smoother) PKG=nav2_velocity_smoother ;;
    esac
    setsid nohup ros2 run "$PKG" "$n" --ros-args --params-file "$D/nav2_nav.yaml" \
        >> "$LOG" 2>&1 < /dev/null &
    sleep 3
done
sleep 5

# lifecycle_manager 把節點從 unconfigured 帶到 active。
# ★ 少了它,節點會起來但 action server 永遠不出現 —— 在 RViz 點目標點
#   完全沒反應,而且沒有任何錯誤訊息。這是最常見的卡點。
LIST=$(echo "$START" | tr ' ' ',' | sed 's/,/, /g')
setsid nohup ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -r __node:=lifecycle_manager_navigation \
    -p "node_names:=[$LIST]" -p autostart:=true -p bond_timeout:=0.0 \
    >> "$LOG" 2>&1 < /dev/null &
sleep 14

# ★ 乾跑模式要自己接 /goal_pose。RViz 的「2D Goal Pose」只是把目標發到
#   /goal_pose,它不會叫任何人規劃 —— 完整模式是 bt_navigator 訂閱它然後
#   跑行為樹。乾跑刻意不起 bt_navigator(那樣才沒有任何節點會發 /cmd_vel),
#   所以要補一個只呼叫 ComputePathToPose 的橋接。
if [ "${DRY_BRIDGE:-0}" = "1" ]; then
    pkill -f goal_to_plan.py 2>/dev/null; sleep 1
    cd "$D" && setsid nohup python3 goal_to_plan.py         > /tmp/goal_to_plan.log 2>&1 < /dev/null &
    sleep 4
    pgrep -f goal_to_plan.py > /dev/null         && echo "  goal_to_plan OK(/goal_pose -> /plan)"         || { echo "  goal_to_plan DEAD"; tail -5 /tmp/goal_to_plan.log; }
fi

echo
echo "=== 狀態 ==="
for n in $START; do
    S=$(timeout 10 ros2 lifecycle get "/$n" 2>&1 | head -1)
    printf "  %-20s %s\n" "$n" "$S"
done

echo
echo "=== action server 出現了嗎 ==="
timeout 15 ros2 action list 2>/dev/null | grep -E "navigate_to_pose|follow_path|compute_path" | sed 's/^/  /' \
    || echo "  ✗ 沒有 —— lifecycle 沒 active,在 RViz 點目標點會沒反應"

if [ "$DRY" = "1" ]; then
cat <<'MSG'

=== 乾跑模式 ===
  在 RViz 工具列用 **2D Goal Pose** 點目標(不是 Nav2 Goal ——
  那是 nav2_rviz_plugins 額外裝的,這邊沒有)。
  路徑會畫在 /plan 上,車不會動。
  路徑看起來合理之後,再跑完整模式:
      bash ~/slam2d/start_nav.sh
MSG
else
cat <<'MSG'

═══ ★ 車現在會動 ★ ═══

  在 RViz 工具列點 "Nav2 Goal",在地圖上點目標位置、拖出朝向。

  隨時要停:
      bash ~/slam2d/start_nav.sh stop
  或在 RViz 按 Nav2 面板的 Cancel。

  ── 第一次請這樣測 ──────────────────────────────────────
  1. 目標點設在**兩三公尺外的空曠處**,不要一開始就跨越整條走廊
  2. 人站在車旁邊,手放在遙控的停止鍵上
  3. 看它會不會:原地轉向對準 -> 直線前進 -> 到點停下
  4. 再測「前面站人」:它應該減速停住,人離開後繼續

  ── 安全限制 ────────────────────────────────────────────
  車前 70 公分是盲區(/scan 的 range_min)。人站太近反而看不到。
  比 10 公分矮的東西看不到(高度帶從離地 0.10 m 才開始)。
MSG
fi
