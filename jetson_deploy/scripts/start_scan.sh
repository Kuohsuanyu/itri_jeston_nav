#!/usr/bin/env bash
# 把 FAST-LIO 的點雲切成一層 2D 的 /scan。
#
#   /cloud_registered_body ──(pointcloud_to_laserscan)── /scan
#
# ── 為什麼要獨立成一支 ──────────────────────────────────────────
# 這段本來寫在 start_slam2d.sh 裡面,但 /scan 是**建圖和定位都要**的:
#
#     建圖  /scan -> slam_toolbox -> /map
#     定位  /scan -> AMCL         -> map -> multi_odom
#
# bringup_all.sh 的 loc 模式跳過 start_slam2d.sh,結果就是沒有人產生
# /scan,start_localization.sh 前置檢查失敗直接中止 —— AMCL 沒起來,
# map -> multi_odom 接不上,TF tree 斷成兩截。2026-08-12 實測。
#
# ★ 呼叫端不要用 `bash start_scan.sh | sed`。這支會 spawn 背景行程,
#   只要它還握著 pipe 的寫入端,sed 就等不到 EOF,整條腳本卡死。
#   寫檔案再 cat 出來。
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

LOG=/tmp/scan.log

# ---- 高度帶 ------------------------------------------------------------
# ★ target_frame 是 base_footprint,而它**就在地面上**(z=0),
#   所以這兩個值直接就是離地高度,不用再減任何偏移 ——
#   少一個換算就少一個改了一半的機會。
MIN_H=0.10
MAX_H=1.50

# ---- range_min:把車子自己濾掉 ------------------------------------------
# 高度帶(離地 0.10~1.50)會**涵蓋車體自己的上蓋** —— DD-M-HH 的上蓋
# 離地 0.5187,正好落在帶子中間。光達裝在上蓋上方,斜著打得到上蓋外緣,
# 那圈點會變成一圈假障礙物,Nav2 會以為自己被包圍。
#
# 外廓要算到**輪子外緣**,不是車體 mesh:
#   車體    ±0.5050 x ±0.3400
#   輪子    Y 外緣 ±0.3718(輪關節 0.31105 + 輪寬外側 0.0607)← 這個才是最寬處
#   半對角線 sqrt(0.505^2 + 0.3718^2) = 0.627 m
# 所以 range_min 取 0.70 切得乾淨(留 7 公分餘裕)。代價是 0.70 m 內看不到
# 東西,但那已經在 Nav2 的 footprint 裡面,本來就不該靠 /scan 處理。
#
# ★ 這個值改了 localization.yaml 的 laser_min_range 也要一起改,
#   否則 AMCL 會把「被濾掉所以是 inf」的方向當成真的沒有障礙物。
RANGE_MIN=0.70

echo "  /scan  高度帶 $MIN_H ~ $MAX_H m(離地),range_min $RANGE_MIN m"

# ---- 先等 FAST-LIO ------------------------------------------------------
# ★ FAST-LIO 起來之後還要做 IMU 靜止初始化 + 建 ikd-Tree,才開始發點雲。
#   實測從行程啟動到第一筆 /cloud_registered_body 要 30~60 秒,而
#   startall.sh 只 sleep 18。中間這段空窗期如果直接檢查 /scan,會得到
#   「行程活著但沒資料」然後整條定位鏈中止 —— 2026-08-12 連續失敗兩次,
#   而 15 秒後其實就正常了。
#
#   不能靠加大 sleep 解決:初始化時間跟現場環境有關(點太少會重試)。
#   等條件成立,不要等固定秒數。
echo -n "  等 /cloud_registered_body "
for i in $(seq 1 20); do
    if timeout 5 ros2 topic echo /cloud_registered_body --once > /dev/null 2>&1; then
        echo " 好了(${i} 次嘗試)"
        break
    fi
    echo -n "."
    [ "$i" = "20" ] && {
        echo " 逾時"
        echo "  ✗ FAST-LIO 沒有輸出點雲。看 /tmp/fastlio.log:"
        tail -5 /tmp/fastlio.log 2>/dev/null | cut -c1-110 | sed 's/^/      /'
        exit 1
    }
done

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
  > "$LOG" 2>&1 < /dev/null &
sleep 5

if pgrep -f pointcloud_to_laserscan_node > /dev/null; then
    H=$(timeout 10 ros2 topic hz /scan 2>&1 | grep -oE "average rate: [0-9.]+" | head -1 | grep -oE "[0-9.]+")
    if [ -n "$H" ]; then
        echo "  [OK]   /scan $H Hz"
        exit 0
    fi
    echo "  [WARN] 行程活著但 /scan 沒資料"
    echo "         最常見原因是 /cloud_registered_body 沒有 —— FAST-LIO 起來了嗎"
    tail -5 "$LOG" | sed 's/^/         /'
    exit 1
fi
echo "  [DEAD] pointcloud_to_laserscan 沒起來"
tail -8 "$LOG" | sed 's/^/         /'
exit 1
