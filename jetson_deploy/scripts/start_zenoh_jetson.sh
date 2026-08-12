#!/usr/bin/env bash
# Jetson 端的 zenoh bridge —— WSL 的 RViz 靠這條 TCP 連過來。
#
# ── 為什麼要節流 ───────────────────────────────────────────────
# 預設會把 domain 0 的**每一條** topic 都橋過去,包含三條 RealSense 影像
# (640x480、15 Hz、深度是 16 bit):
#     color            640*480*3  * 15 ≈  13 MB/s
#     depth            640*480*2  * 15 ≈   9 MB/s
#     aligned_depth    640*480*2  * 15 ≈   9 MB/s
#     /livox/lidar     每則兩萬點  * 10 ≈  10 MB/s
# 加起來遠超過 WiFi 實際吞吐。結果是**所有東西**一起塞車,
# 而 RViz 最需要的 /tf 是小訊息,卻跟影像排同一條隊。
#
# 症狀:RViz 出現
#     Message Filter dropping message: frame 'base_footprint' at time ...
#     reason 'discarding message because the queue is full'
#     reason 'the timestamp on the message is earlier than all the data in
#             the transform cache'
# 看起來像 TF 樹壞掉,實際上樹是好的,只是 /tf 晚了十幾秒才到,
# RViz 想內插的時間點早就滾出快取了。
#
# ── 做法 ────────────────────────────────────────────────────
# v1.9.0 的 CLI 沒有 --allow / --deny(那是設定檔才有的),但
# --pub-max-frequency 吃正規表達式而且可以重複,效果一樣夠:
# 把大訊息壓到「看得到就好」的頻率,小訊息維持全速。
#
# 維持全速的(RViz 真正需要的):
#     /tf  /tf_static  /scan  /map  /odom  /odometry/filtered  /robot_description
# ★★ 啟動順序很重要:一定要在**底盤上線之後**才啟動這支。
#
# zenoh-bridge-ros2dds 是按「探索到的 ROS 節點」建路由的,不是按 topic。
# 底盤還沒起來時啟動,它探索不到 /chassis_driver 和 /robot_state_publisher,
# 那兩個節點發的東西(輪子的 TF、/odom、/joint_states)就一律不轉發 ——
# 而且不會有任何錯誤,只是 WSL 那邊少東西。
#
# 2026-08-12 實測:
#   底盤離線時啟動 bridge  ->  Discovered 清單全是 Jetson 本機的,WSL 沒有輪子
#   底盤在線時啟動 bridge  ->  /chassis_driver /robot_state_publisher 都探索到,
#                              WSL 拿到完整的 13 個 frame
#
# 底盤重開機之後也要重跑這支。
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# ★ 單播探索。沒有這個的話 bridge 探索不到底盤(樹莓派)的節點,
#   就不會幫 /joint_states /odom /battery_state /chassis/* 建路由 ——
#   WiFi 的 multicast 不可靠,而 bridge 是按「探索到的節點」建路由的。
#   2026-08-11 實測:資料收得到,但 bridge 一條底盤的路由都沒建。
export FASTRTPS_DEFAULT_PROFILES_FILE=~/slam2d/fastdds_peers.xml

LOG=/tmp/zenoh_jetson.log

# ★ pkill -x 對 zenoh-bridge-ros2dds 沒用:Linux 的 comm 上限 15 字元,
#   實際只有 "zenoh-bridge-ro",精確比對永遠對不上。用 pgrep -f 抓 PID。
# ★ 一律先 SIGTERM 並**等它真的走完**,不要急著 -9。
#
# kill -9 會讓 zenoh session 沒機會關閉,對端(WSL)的路由就留成殘留。
# 累積下來的症狀是同一個 topic 出現好幾個發布者:
#     /scan 4 個、/tf_static 7 個、路由 669 條(正常約 190)
# RViz 同一幀掃描收到四次、TF 收到五次,直接卡死。
# 2026-08-12 實測,而且是我自己反覆 -9 造成的。
PIDS=$(pgrep -f zenoh-bridge-ros2dds || true)
if [ -n "$PIDS" ]; then
    echo "  停掉舊的 (PID $PIDS),等它關閉 session"
    kill $PIDS 2>/dev/null || true
    for i in $(seq 1 12); do
        sleep 1
        pgrep -f zenoh-bridge-ros2dds > /dev/null || break
    done
    if pgrep -f zenoh-bridge-ros2dds > /dev/null; then
        echo "  ⚠ 12 秒沒走,強制 —— 對端可能會留下殘留路由"
        kill -9 $(pgrep -f zenoh-bridge-ros2dds) 2>/dev/null || true
        sleep 3
    else
        echo "  已乾淨關閉"
    fi
    sleep 3   # 讓 DDS 的探索散播 unmatch
fi

echo "  啟動 bridge(監聽 tcp/0.0.0.0:7447)"
setsid nohup zenoh-bridge-ros2dds \
    -l tcp/0.0.0.0:7447 \
    --no-multicast-scouting \
    --pub-max-frequency '/camera/.*=0.2' \
    --pub-max-frequency '/livox/lidar=0.2' \
    --pub-max-frequency '/livox/imu=2.0' \
    --pub-max-frequency '/cloud_registered=1.0' \
    --pub-max-frequency '/cloud_registered_body=0.5' \
    --pub-max-frequency '/cloud_effected=0.1' \
    --pub-max-frequency '/Laser_map=0.1' \
    --pub-max-frequency '/slam_toolbox/.*=0.5' \
    --pub-max-frequency '/path=1.0' \
    > "$LOG" 2>&1 < /dev/null &

sleep 10
if pgrep -f zenoh-bridge-ros2dds > /dev/null; then
    echo "  OK  路由 $(grep -c 'Route' "$LOG" 2>/dev/null) 條"
else
    echo "  ✗ 沒起來"
    tail -8 "$LOG"
    exit 1
fi

# 埠被佔住是最常見的失敗:舊的沒殺乾淨,新的就啟不了但也不會大聲抱怨。
if grep -qi "Address already in use" "$LOG"; then
    echo "  ✗ 7447 被佔用 —— 舊的 bridge 還在"
    exit 1
fi

# 檢查有沒有製造重複發布者。bridge 應該只**轉發**,不該讓本機的 topic
# 多出發布者 —— 多出來就是殘留路由或迴圈,RViz 會收到重複資料而卡死。
sleep 5
echo "  發布者檢查(每個應該都是 1):"
BAD=0
for t in /scan /map /cloud_registered; do
    n=$(timeout 10 ros2 topic info "$t" 2>/dev/null | grep -oE "Publisher count: [0-9]+" | grep -oE "[0-9]+")
    printf "    %-20s %s
" "$t" "${n:-?}"
    [ "${n:-1}" -gt 1 ] && BAD=1
done
[ "$BAD" = "1" ] && {
    echo "  ⚠ 有重複發布者 —— 兩端都停掉(SIGTERM)再依序啟動:"
    echo "     WSL:  kill \$(pgrep -f zenoh-bridge-ros2dds)"
    echo "     Jetson: 再跑一次這支"
}
