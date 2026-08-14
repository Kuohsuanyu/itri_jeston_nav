#!/usr/bin/env bash
# 把 AMCL 的位置重設回一個已知的定點 —— 定位跑掉時的救援指令。
#
#   bash ~/slam2d/reset_pose.sh                 回到初始位置(地圖原點)
#   bash ~/slam2d/reset_pose.sh 充電站           回到具名航點
#   bash ~/slam2d/reset_pose.sh 12.5 -1.2 90    直接給 x y yaw(度)
#   bash ~/slam2d/reset_pose.sh save 充電站      把**目前**位置存成航點
#   bash ~/slam2d/reset_pose.sh list            列出所有航點
#   bash ~/slam2d/reset_pose.sh global          撒滿粒子讓它自己找(見下)
#
# ── 為什麼需要這支 ──────────────────────────────────────────────
# AMCL 現在是**純追蹤器**,不會自己找到自己。三個設定加起來:
#
#   recovery_alpha_slow/fast: 0.0   關掉隨機粒子注入。開著的話匹配不好時
#                                   會一直灌亂數,實測症狀是車停著但地圖上
#                                   的位置一直游移(2026-08-12)。
#   初始共變異數 0.10 / 0.03        刻意給得緊。這條走廊門洞週期性重複,
#                                   給鬆的話粒子散開會鎖到隔壁門洞。
#   update_min_d: 0.2               車停著時濾波器一次都不跑。
#
# 好處是追蹤很穩,代價是**掉了就回不來**。所以要有一個「回到原點」的動作:
# 把車開到地上那個已知的定點,下這道指令,位置就重新對齊。
#
# 這在這種環境裡比讓它自己猜可靠得多 —— 把同一幀掃描沿走廊滑動,相距
# 十幾公尺的三個位置分數一模一樣(都是 0.0500),那是環境的幾何歧義,
# 不是參數調得好不好的問題。
#
# ── global 模式 ────────────────────────────────────────────────
# 呼叫 /reinitialize_global_localization,把粒子撒滿整張地圖,然後要**推著
# 車走一段**才會收斂 —— 走過的門洞序列才是能分辨位置的資訊。車停著的話
# 它只會一直維持散開的狀態。
#
# 這條走廊上成功率不高(週期性重複),真的迷路時建議還是開回定點重設。
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

D=~/slam2d
say() { printf "\n\033[1m%s\033[0m\n" "$*"; }
ok()  { printf "  \033[32m✓\033[0m %s\n" "$1"; }
bad() { printf "  \033[31m✗\033[0m %s\n" "$1"; }

# ---- 前置:AMCL 要活著而且是 active ------------------------------------
if ! pgrep -f "nav2_amcl/amcl" > /dev/null; then
    bad "AMCL 沒在跑"
    echo "     先把定位堆疊帶起來:"
    echo "       bash ~/slam2d/bringup_all.sh loc ~/maps/<地圖>.yaml"
    exit 1
fi
LC=$(timeout 15 ros2 lifecycle get /amcl 2>&1)
case "$LC" in
    active*) ;;
    *) bad "AMCL 狀態是 $LC,不是 active"
       echo "     lifecycle_manager 可能掛了。重跑 start_localization.sh"
       exit 1 ;;
esac

case "$1" in
  list) exec python3 "$D/waypoint.py" list ;;
  save) shift; exec python3 "$D/waypoint.py" save "$@" ;;
  global)
    say "全域重定位:把粒子撒滿整張地圖"
    echo "  ★ 撒完之後要**推著車走一段**才會收斂。車停著的話它只會維持散開。"
    timeout 25 ros2 service call /reinitialize_global_localization \
        std_srvs/srv/Empty "{}" 2>&1 | tail -2 | sed 's/^/  /'
    echo
    echo "  推車走 10 公尺以上、經過幾個特徵不同的門洞,再驗證:"
    echo "    python3 ~/slam2d/check_localization.py"
    exit 0 ;;
esac

# ---- 決定要重設到哪 ----------------------------------------------------
if [ -z "$1" ]; then
    # 預設 = 地圖原點 = 當初開始建圖的那個實體定點
    X=0.0; Y=0.0; YAW=0.0; WHERE="初始位置(地圖原點)"
elif [ -n "$3" ]; then
    X="$1"; Y="$2"; YAW="$3"; WHERE="指定座標"
else
    # 具名航點 —— 交給 waypoint.py,它才是航點檔的唯一寫入者
    say "重設到航點「$1」"
    python3 "$D/waypoint.py" init "$1" || exit 1
    WHERE="航點 $1"
    X=""; # 已經發過了,下面跳過發布
fi

if [ -n "$X" ]; then
    say "重設到 $WHERE:map($X, $Y, $YAW°)"
    if [ -z "$1" ]; then
        echo "  這是當初開始建圖的那個實體定點。車要確實停在那裡,"
        echo "  不然是把錯的位置蓋成另一個錯的位置。"
    fi
    # 共變異數小 = 「我確定在這裡」。跟 start_localization.sh 用同一組值,
    # 兩邊不一致的話會出現「重設完的行為跟剛啟動時不一樣」這種難查的差異。
    S=$(python3 -c "import math;print(math.sin(math.radians($YAW)/2))")
    W=$(python3 -c "import math;print(math.cos(math.radians($YAW)/2))")
    timeout 20 ros2 topic pub --once /initialpose \
      geometry_msgs/msg/PoseWithCovarianceStamped \
      "{header: {frame_id: map}, pose: {pose: {position: {x: $X, y: $Y, z: 0.0},
        orientation: {z: $S, w: $W}},
        covariance: [0.10,0,0,0,0,0, 0,0.10,0,0,0,0, 0,0,0,0,0,0,
                     0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.03]}}" \
      2>&1 | tail -1 | sed 's/^/  /'
fi

sleep 5

# ---- 驗證 --------------------------------------------------------------
# ★ 發出去不等於生效。AMCL 要收到下一幀 /scan 才會重新發 map -> multi_odom,
#   而且如果 /scan 停了,它會安靜地什麼都不做。
say "驗證"
if timeout 12 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -q Translation; then
    P=$(timeout 12 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 \
        | grep -m1 Translation | sed 's/.*\[/[/')
    ok "map -> base_footprint $P"
else
    bad "查不到 map -> base_footprint"
    echo "     /scan 有在發嗎?ros2 topic hz /scan"
    exit 1
fi

echo
echo "  掃描和地圖的吻合度:"
timeout 70 python3 "$D/check_localization.py" 2>/dev/null \
    | sed -n '/幀 \//,$p' | sed 's/^/    /'

cat <<'MSG'

  命中率 > 70% 才算對齊。低於 50% 的話:
      車沒有真的停在那個定點       —— 對一下地上的標記
      這個定點跟建圖起點有偏差     —— 開到定點後用 RViz 的 2D Pose Estimate
                                      微調,再 bash reset_pose.sh save 初始位置
                                      以後就用 bash reset_pose.sh 初始位置
      車根本不在地圖範圍內(車庫)  —— 那不是定位問題,先開進走廊
MSG
