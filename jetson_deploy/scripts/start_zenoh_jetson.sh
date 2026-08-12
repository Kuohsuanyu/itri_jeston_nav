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
PIDS=$(pgrep -f zenoh-bridge-ros2dds || true)
if [ -n "$PIDS" ]; then
    echo "  停掉舊的 (PID $PIDS)"
    kill $PIDS 2>/dev/null || true
    sleep 4
    PIDS=$(pgrep -f zenoh-bridge-ros2dds || true)
    [ -n "$PIDS" ] && { kill -9 $PIDS 2>/dev/null || true; sleep 2; }
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
