#!/usr/bin/env bash
# 模式 A:純本機看模型 —— 不連 Jetson,車子沒開也能跑。
#
# 用來做什麼:
#   看 URDF 長對不對、量測值貼上去之後感測器擺的位置合不合理、
#   以及最重要的 —— 把 TF 樹畫出來看清楚每個 frame 在哪。
#
# 起三支:
#   robot_state_publisher       讀 URDF -> 發 /tf_static(固定關節)和 /robot_description
#   joint_state_publisher_gui   四個輪子是 continuous joint,沒人餵 /joint_states
#                               robot_state_publisher 就不發輪子的 TF,輪子會消失。
#                               這支開一個滑桿視窗餵假的角度,順便可以轉輪子玩。
#   rviz2
#
# ★ 跟 Jetson 完全隔離:ROS_DOMAIN_ID=9,而且不啟動 zenoh bridge。
#   這很重要 —— 這支發的 /tf_static 裡有 base_link -> box_link,
#   實機 fused 模式下那一段是**故意不發**的(box_link 的父節點是 EKF 的
#   multi_odom)。要是漏到 Jetson 那邊,box_link 就變成雙父節點,
#   tf2 不報錯,只會在兩個答案之間隨機翻轉。所以這裡刻意換一個 domain。
set -e

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=9
export ROS_LOCALHOST_ONLY=1          # 雙保險:封包不出這台機器
export QT_QPA_PLATFORM=xcb
export AMENT_PREFIX_PATH="$HOME/qbot_ws/install/chassis_description:$AMENT_PREFIX_PATH"
[ -d "/run/user/$(id -u)" ] && chmod 700 "/run/user/$(id -u)" 2>/dev/null || true

URDF="$HOME/lidar_view/qbot_view.urdf"
if [ ! -f "$URDF" ]; then
    echo "✗ 找不到 $URDF —— 先跑 setup_model.sh"
    exit 1
fi

echo "=============================================="
echo "  QBOT 模型檢視  (本機,domain $ROS_DOMAIN_ID,不連 Jetson)"
echo "=============================================="

cleanup() {
    kill $RSP $JSP 2>/dev/null || true
}
trap cleanup EXIT

echo "[1/3] robot_state_publisher"
# 用位置參數傳檔案路徑,不要用 --ros-args -p robot_description:="$(cat ...)"。
# 後者是官方文件常見寫法,但 URDF 有換行,rcl 的參數解析器會在第一個換行
# 就斷掉:「Couldn't parse parameter override rule」。位置參數沒這問題。
ros2 run robot_state_publisher robot_state_publisher "$URDF" \
    > /tmp/rsp.log 2>&1 &
RSP=$!
sleep 2
kill -0 $RSP 2>/dev/null || { echo "  DEAD"; tail -20 /tmp/rsp.log; exit 1; }
echo "      OK (PID $RSP)"

echo "[2/3] joint_state_publisher_gui  —— 滑桿視窗,拉了輪子會轉"
ros2 run joint_state_publisher_gui joint_state_publisher_gui \
    > /tmp/jsp.log 2>&1 &
JSP=$!
sleep 2

echo "[3/3] RViz2"
echo
echo "  Fixed Frame 預設 base_footprint(地面)。想只看感測器就改成 box_link。"
echo "  左邊 TF -> Tree 展開,可以看到整棵樹的父子關係。"
echo
rviz2 -d "$HOME/lidar_view/model.rviz"
