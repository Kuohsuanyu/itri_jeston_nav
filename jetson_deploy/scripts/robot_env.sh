#!/usr/bin/env bash
# 全機共用的位址設定 —— 只有這一份是正本。
#
#   source ~/slam2d/robot_env.sh
#
# ── 為什麼要抽出來 ──────────────────────────────────────────────
# 底盤 IP 原本散在 bringup_all.sh、base_off.sh、fastdds_peers.xml 三個地方。
# 換網段時只要漏改一個,症狀是「有時候找得到底盤、有時候找不到」——
# 而且不會有錯誤訊息,因為每支腳本自己那份都是合法的。
#
# ── 2026-08-13:底盤改走有線 ────────────────────────────────────
# 三台機器接在同一台交換器上,自成一個 192.168.0.x 的車內網路:
#
#     [switch]  光達 .0.50    Jetson .0.100(enP8p1s0)   樹莓派 .0.101
#     Jetson 的 WiFi .40.98 保留,只負責對筆電的 zenoh bridge
#
# 實測差異(Jetson -> 樹莓派,各 25~50 次):
#
#     有線 192.168.0.101     0% 遺失   平均 0.307 ms   mdev 0.060 ms
#     無線 192.168.40.160   60% 遺失   平均 307 ms     mdev 336 ms
#
# 抖動差 5000 倍。DDS 的參與者探索靠週期性公告維持,在 60% 丟包的鏈路上
# 就是我們追了好幾天的那些症狀:zenoh bridge 探索不到底盤節點、
# ros2 node list 看不到底盤、/chassis/motor_state 訂閱斷掉三小時不恢復。
#
# ★ 要真的走有線,樹莓派的 WiFi 必須關掉:
#       sudo ip link set wlan0 down
#   兩張網卡都開著時,它會同時公告 .0.101 和 .40.160 兩個 locator,
#   對端挑哪一個不保證 —— 改了 IP 卻還是走無線,而且看不出來。

# 底盤(樹莓派 itri-base)。要臨時切回無線測試:
#     BASE_IP=192.168.40.160 bash ~/slam2d/bringup_all.sh loc
BASE_IP="${BASE_IP:-192.168.0.101}"

# 底盤的無線位址,只在有線不通時當退路提示用
BASE_IP_WIFI="${BASE_IP_WIFI:-192.168.40.160}"

# 光達(Livox Mid-360)和 Jetson 在車內網路的位址
LIDAR_IP="${LIDAR_IP:-192.168.0.50}"
JETSON_WIRED_IP="${JETSON_WIRED_IP:-192.168.0.100}"

# 車內網路用的網卡。startall.sh 會等它拿到位址才啟動光達驅動。
WIRED_IF="${WIRED_IF:-enP8p1s0}"

export BASE_IP BASE_IP_WIFI LIDAR_IP JETSON_WIRED_IP WIRED_IF
