#!/usr/bin/env bash
# 從零把整條鏈帶起來 —— Jetson 重開機之後跑這一支就好。
#
#   bash ~/slam2d/bringup_all.sh            建圖模式(slam_toolbox)
#   bash ~/slam2d/bringup_all.sh loc        定位模式(既有地圖 + AMCL)
#   bash ~/slam2d/bringup_all.sh loc <地圖.yaml>
#
# ── 順序有陷阱,這是把它們固定下來 ──────────────────────────────
#
# 1. 底盤要**先**在線
#      bridge 是按「探索到的 ROS 節點」建路由的。底盤還沒起來時啟動,
#      它探索不到 /chassis_driver 和 /robot_state_publisher,那兩個節點
#      發的東西(輪子的 TF、/odom、/joint_states)就一律不轉發到 WSL,
#      而且不會有任何錯誤。
#
# 2. publish_tf 要關,而且**每次底盤重開機都要重關**
#      那個參數不保存,重開機回到參數檔的 true。不關的話 base_footprint
#      會有兩個父節點(odom 和 multi_odom)—— tf2 不報錯,只會在兩個答案
#      之間隨機翻轉,症狀是車在地圖上亂跳、輪子接不上。
#      2026-08-12 一天之內因此出事兩次。
#      ★ 常常要下兩次:第一次回 "Node not found"(節點探索還沒完成)。
#
# 3. FAST-LIO 的 /tf 一定要靜音
#      它發 camera_init -> body,而我們也發 box_link -> body。
#      用 ros2 launch 起 FAST-LIO 的話沒有 remap,body 就有兩個父節點。
#      startall.sh 用 ros2 run 加 -r /tf:=/tf_fastlio_unused。
#
# 4. bridge 最後起
#      要在上面全部就緒之後,它才探索得到所有節點。
set -e
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/chassis_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0

MODE="${1:-map}"
MAPFILE="$2"
# 位址一律從 robot_env.sh 來 —— 那裡是正本,不要在這裡寫死。
source ~/slam2d/robot_env.sh

say()  { echo; echo "═══ $* ═══"; }
ok()   { printf "  \033[32m✓\033[0m %s\n" "$1"; }
bad()  { printf "  \033[31m✗\033[0m %s\n" "$1"; }
hz()   { timeout "${2:-8}" ros2 topic hz "$1" 2>&1 | grep -oE "average rate: [0-9.]+" \
         | head -1 | grep -oE "[0-9.]+"; }

say "1/6 等底盤上線"
# bridge 和 publish_tf 都依賴底盤已經在線,所以這一步是硬性前提。
for i in $(seq 1 30); do
    ping -c 1 -W 1 "$BASE_IP" > /dev/null 2>&1 && break
    [ "$i" = "1" ] && echo "  等待 $BASE_IP ..."
    sleep 3
done
if ping -c 2 -W 2 "$BASE_IP" > /dev/null 2>&1; then
    R=$(ping -c 10 -q "$BASE_IP" 2>/dev/null | tail -1 | sed -E 's|.*= [0-9.]+/([0-9.]+)/[0-9.]+/([0-9.]+) ms|平均 \1 ms  抖動 \2 ms|')
    ok "底盤 $BASE_IP 在線($(ip route get "$BASE_IP" 2>/dev/null | grep -oE 'dev [a-zA-Z0-9]+')) $R"
else
    bad "底盤 $BASE_IP 90 秒內沒上線"
    # ★ 2026-08-13 改走有線之後最常見的失誤:交換器沒接、或樹莓派的
    #   有線介面沒設 IP。先確認是「底盤沒開」還是「只是有線那條不通」。
    if ping -c 2 -W 2 "$BASE_IP_WIFI" > /dev/null 2>&1; then
        echo "     但無線 $BASE_IP_WIFI 通得到 —— 底盤是開著的,是**有線這條**不通。"
        echo "     檢查:交換器的線、樹莓派上 ip addr show(要有 $BASE_IP)"
        echo "     臨時要用無線跑的話:"
        echo "       BASE_IP=$BASE_IP_WIFI bash ~/slam2d/bringup_all.sh $MODE $MAPFILE"
    else
        echo "     無線 $BASE_IP_WIFI 也不通 —— 底盤應該是沒開機。"
        echo "     沒有底盤的話可以只跑感測器:"
        echo "       bash ~/slam2d/startall.sh && bash ~/slam2d/robot_tf.sh standalone"
    fi
    exit 1
fi
O=$(hz /odom 10)
[ -n "$O" ] && ok "/odom $O Hz" || { bad "/odom 沒資料 —— 底盤的 bringup 起來了嗎"; exit 1; }

say "2/6 感測器 + FAST-LIO"
# startall.sh 內含 -r /tf:=/tf_fastlio_unused,不要改用 ros2 launch 起 FAST-LIO
bash ~/slam2d/startall.sh > /tmp/bringup_startall.log 2>&1
tail -6 /tmp/bringup_startall.log | sed 's/^/    /'
for p in livox_ros_driver2_node fastlio_mapping; do
    pgrep -f "$p" > /dev/null && ok "$p" || { bad "$p 沒起來"; tail -12 /tmp/bringup_startall.log; exit 1; }
done
# 確認 remap 真的有帶上 —— 沒有的話 body 會有兩個父節點
ps -eo cmd | grep "[f]astlio_mapping" | grep -q tf_fastlio_unused \
    && ok "FAST-LIO 的 /tf 已靜音" \
    || bad "FAST-LIO 沒有 /tf remap —— body 會有兩個父節點!"

say "2b/6 把底盤這條逼到有線"
# 樹莓派兩張網卡都開著就會同時公告有線和無線兩個 DDS locator,對端挑哪一條
# 不保證。實測流量真的被切成兩半(有線 1015 / 無線 697 個封包 / 8 秒),
# 而無線是 218 ms 延遲、215 ms 抖動 —— 有線只有 0.296 / 0.032。
# wired_only.sh 在 Jetson 這端把無線那個位址變成死路,底盤完全不用改。
# ★ ip route / iptables 不持久,所以每次啟動都要重下。
bash ~/slam2d/wired_only.sh on 2>&1 | sed 's/^/    /'

say "3/6 關掉底盤的 TF 廣播"
# ★ 常常第一次會 "Node not found",那是節點探索還沒完成,不是真的失敗。
ros2 daemon stop > /dev/null 2>&1; sleep 3
DONE=0
for i in 1 2 3 4; do
    R=$(timeout 25 ros2 param set /chassis_driver publish_tf false 2>&1 | tail -1)
    echo "    第 $i 次:$R"
    echo "$R" | grep -q successful && { DONE=1; break; }
    sleep 6
done
[ "$DONE" = "1" ] && ok "已關閉" || {
    bad "關不掉 —— 會有雙父節點。到底盤上重啟 bringup 並帶:"
    echo "     ros2 launch chassis_bringup bringup.launch.py localization_mode:=external_takeover"
    exit 1
}

say "4/6 TF + EKF + /scan + 建圖或定位"
if [ "$MODE" = "loc" ]; then
    bash ~/slam2d/robot_tf.sh fused > /tmp/bringup_tf.log 2>&1
    tail -8 /tmp/bringup_tf.log | sed 's/^/    /'
    bash ~/slam2d/start_localization.sh $MAPFILE 2>&1 | tail -14 | sed 's/^/    /'
else
    bash ~/slam2d/start_slam2d.sh 2>&1 | tail -16 | sed 's/^/    /'
fi

say "5/6 zenoh bridge(最後起,才探索得到全部節點)"
bash ~/slam2d/start_zenoh_jetson.sh 2>&1 | grep -E "啟動|OK|路由|則|✗" | sed 's/^/    /'
grep -oE "Discovered ROS Node /[a-z_0-9/]+" /tmp/zenoh_jetson.log 2>/dev/null \
    | grep -iE "chassis|robot_state" | sort -u | sed 's/^/    /' \
    && ok "bridge 探索到底盤的節點" \
    || bad "bridge 沒探索到底盤 —— WSL 會少輪子的 TF"

say "6/6 驗收"
bash ~/slam2d/verify_stack.sh 10 2>&1 | tail -40

echo
if [ "$MODE" = "loc" ]; then
cat <<'MSG'
═══ 定位模式:還要給初始位置 ═══
  已有航點:  python3 ~/slam2d/waypoint.py init <名字>
  沒有航點:  在 RViz 用 2D Pose Estimate 點車的實際位置
  確認:      python3 ~/slam2d/check_localization.py    命中率要 > 70%
MSG
else
cat <<'MSG'
═══ 建圖模式 ═══
  用遙控慢慢繞,地圖會在 /map 上長出來
  存檔:  bash ~/slam2d/robot.sh save [名字]

  ★ 地圖原點 map(0,0) = 你**按下建圖時車停的位置**,不是圖片角落。
    那個點就是以後的「回家」定點 —— 值得在地上做個實體標記。
MSG
fi
echo
echo "筆電那邊:開啟RViz.bat"
