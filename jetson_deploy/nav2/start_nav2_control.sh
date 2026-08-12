#!/usr/bin/env bash
# Nav2 control layer. ASCII only (base64|bash mangles CJK).
#
#   bash ~/nav2/start_nav2_control.sh            start
#   bash ~/nav2/start_nav2_control.sh --dry      start with cmd_vel NOT connected
#
# --dry runs everything except the twist_mux output remap, so /cmd_vel is never
# published. Use it for the first bring-up: you can watch /cmd_vel_nav to see
# what Nav2 WOULD command, with zero chance of the wheels moving.
#
# Prerequisites that must already be running (startall.sh):
#   livox driver, FAST-LIO, pointcloud_to_laserscan, slam_toolbox
# and planner_server from start_nav2.sh.
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

D=~/nav2
LOG=/tmp/nav2_control.log
: > "$LOG"

DRY=0
[ "$1" = "--dry" ] && DRY=1

echo "=== preflight ==="
fail=0
# /scan must be FLOWING -- a controller with a stale scan drives into things.
if timeout 6 ros2 topic hz /scan 2>/dev/null | head -1 | grep -q rate; then
    echo "  [OK]   /scan"
else
    echo "  [FAIL] /scan  no data"
    fail=1
fi
# /map must EXIST, not tick. slam_toolbox only republishes when new keyframes
# arrive, so a parked robot legitimately shows zero rate on a perfectly good
# map. Checking hz here fails on a stationary vehicle -- which is exactly the
# state you are in when you start the stack.
if timeout 15 ros2 topic echo /map --once --field info.width > /dev/null 2>&1; then
    echo "  [OK]   /map"
else
    echo "  [FAIL] /map  not published"
    fail=1
fi
# Nav2 asks TF for map -> base_footprint every control cycle. If it is not there the
# controller starts, accepts a goal, and then fails on every tick with a
# transform error -- which looks like a Nav2 bug but is not.
if timeout 6 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -q Translation; then
    echo "  [OK]   TF map -> base_footprint"
else
    echo "  [FAIL] TF map -> base_footprint"
    fail=1
fi
if [ "$fail" = "1" ]; then
    echo
    echo "  preflight failed. Fix the above before starting the controller."
    echo "  A controller that cannot see /scan will happily drive into things."
    exit 1
fi

echo
echo "[1/4] twist_mux"
pkill -f twist_mux 2>/dev/null; sleep 1
# The mux output always goes to /cmd_vel so MANUAL DRIVING KEEPS WORKING.
# --dry instead diverts only the Nav2 branch: velocity_smoother publishes to
# /cmd_vel_nav_dry, which twist_mux does not read. Nav2 computes a full plan and
# commands velocities you can watch, and none of it can reach the wheels.
# (Diverting the mux output instead would also kill teleop -- which is exactly
#  when you most want to be able to drive away from something.)
OUT="/cmd_vel"
if [ "$DRY" = "1" ]; then
    NAV_OUT="/cmd_vel_nav_dry"
    BEH_OUT="/cmd_vel_behavior_dry"
    echo "  DRY RUN: Nav2 -> $NAV_OUT, recovery -> $BEH_OUT"
    echo "           neither is wired to the mux. Teleop still works."
else
    NAV_OUT="/cmd_vel_nav"
    BEH_OUT="/cmd_vel_behavior"
fi
setsid nohup ros2 run twist_mux twist_mux \
    --ros-args --params-file "$D/twist_mux.yaml" \
    -r cmd_vel_out:="$OUT" \
    >> "$LOG" 2>&1 < /dev/null &
sleep 3

echo "[2/4] controller_server + local_costmap"
pkill -f "nav2_controller" 2>/dev/null; sleep 1
setsid nohup ros2 run nav2_controller controller_server \
    --ros-args --params-file "$D/nav2_control.yaml" \
    -r cmd_vel:=/cmd_vel_smoother_in \
    >> "$LOG" 2>&1 < /dev/null &
sleep 4

echo "[3/4] velocity_smoother + behavior_server"
pkill -f velocity_smoother 2>/dev/null
pkill -f behavior_server 2>/dev/null; sleep 1
setsid nohup ros2 run nav2_velocity_smoother velocity_smoother \
    --ros-args --params-file "$D/nav2_control.yaml" \
    -r cmd_vel:=/cmd_vel_smoother_in -r cmd_vel_smoothed:="$NAV_OUT" \
    >> "$LOG" 2>&1 < /dev/null &
# behavior_server 預設直接發 /cmd_vel,繞過 twist_mux。不 remap 的話,
# dry run 時 spin 復原照樣轉得動輪子 —— 這是安全漏洞,不是效能問題。
setsid nohup ros2 run nav2_behaviors behavior_server \
    --ros-args --params-file "$D/nav2_control.yaml" \
    -r cmd_vel:="$BEH_OUT" \
    >> "$LOG" 2>&1 < /dev/null &
sleep 4

echo "[4/4] bt_navigator + lifecycle"
pkill -f bt_navigator 2>/dev/null; sleep 1
setsid nohup ros2 run nav2_bt_navigator bt_navigator \
    --ros-args --params-file "$D/nav2_control.yaml" \
    >> "$LOG" 2>&1 < /dev/null &
sleep 4

pkill -f "lifecycle_manager.*control" 2>/dev/null; sleep 1
setsid nohup ros2 run nav2_lifecycle_manager lifecycle_manager \
    --ros-args -r __node:=lifecycle_manager_control \
    -p use_sim_time:=false -p autostart:=true -p bond_timeout:=12.0 \
    -p "node_names:=[controller_server, behavior_server, bt_navigator, velocity_smoother]" \
    >> "$LOG" 2>&1 < /dev/null &
sleep 12

echo
echo "=== status ==="
for p in twist_mux controller_server velocity_smoother behavior_server bt_navigator lifecycle_manager; do
    pgrep -f "$p" > /dev/null && echo "  [OK]   $p" || echo "  [DEAD] $p"
done

echo
echo "=== lifecycle states (must all be active) ==="
for n in controller_server behavior_server bt_navigator velocity_smoother; do
    s=$(timeout 6 ros2 lifecycle get /$n 2>/dev/null | head -1)
    printf "  %-20s %s\n" "$n" "${s:-no response}"
done

echo
echo "=== cmd_vel chain ==="
echo "  teleop  -> /cmd_vel_teleop  priority 100"
echo "  recovery-> $BEH_OUT  priority 20"
echo "  nav2    -> $NAV_OUT     priority 10"
echo "  output  -> $OUT"
ros2 topic info "$OUT" 2>/dev/null | sed 's/^/  /'

echo
echo "=== errors in log ==="
grep -iE "error|fail|exception" "$LOG" | grep -v "failure_tolerance" | tail -8 | sed 's/^/  /'

echo
echo "Send a goal:"
echo "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\"
echo "    \"{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}\""
echo
echo "Watch what it would command (safe, does not move anything):"
echo "  ros2 topic echo $NAV_OUT"
echo
echo "Software stop:"
echo "  ros2 topic pub -1 /emergency_stop std_msgs/Bool '{data: true}'"
