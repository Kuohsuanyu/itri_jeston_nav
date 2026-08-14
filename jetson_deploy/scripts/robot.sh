#!/usr/bin/env bash
# 車子的單一入口 —— 兩種模式,加上初始點的管理。
#
#   bash ~/slam2d/robot.sh map            建圖模式:邊走邊畫新地圖
#   bash ~/slam2d/robot.sh save [名字]    存地圖(建圖模式中)
#
#   bash ~/slam2d/robot.sh use [地圖]     使用地圖模式:在既有地圖上定位
#   bash ~/slam2d/robot.sh home [航點]    回到初始點(定位跑掉時用)
#   bash ~/slam2d/robot.sh mark <名字>    把目前位置存成航點
#
#   bash ~/slam2d/robot.sh status         現在是什麼模式、資料流正不正常
#   bash ~/slam2d/robot.sh stop           全部停掉
#   bash ~/slam2d/robot.sh view           開即時三維點雲(:8100)
#
# ── 兩種模式為什麼不能並存 ──────────────────────────────────────
#   建圖  slam_toolbox  一邊建 /map,一邊發 map -> multi_odom
#   定位  map_server    發固定的 /map
#         amcl         發 map -> multi_odom
#
# map -> multi_odom 只能有一個發布者。兩個都跑的話 tf2 **不會報錯**,
# 它會在兩個答案之間隨機翻轉,症狀是車在地圖上瞬移。所以切換模式時
# 一定要先把另一邊停掉 —— 底下的腳本都有做,不要手動同時起。
#
# ── ★ 初始點就是「開始建圖時車停的位置」 ────────────────────────
# 地圖的 map(0,0) 定義在你按下建圖時車所在的地方。所以:
#
#     1. 把車停在你要當定點的位置(地上做個實體標記)
#     2. robot.sh map        ← 從這裡開始建
#     3. 繞完 robot.sh save
#     4. 以後 robot.sh use,車開回那個標記,robot.sh home 就對齊了
#
# 順序反過來(先建圖再挑一個點當初始點)也可以,那時要:
#     車開到那個點 -> 在 RViz 用 2D Pose Estimate 對準 -> robot.sh mark 初始位置
#     以後用 robot.sh home 初始位置
# ★ 不能用 set -u。/opt/ros/humble/setup.bash 內部會讀 AMENT_TRACE_SETUP_FILES
#   等未初始化的變數,開了 nounset 這支腳本會在第一行 source 就中止,
#   訊息是 "AMENT_TRACE_SETUP_FILES: unbound variable" —— 看起來像 ROS 壞了。
D=~/slam2d
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source "$D/robot_env.sh"
export ROS_DOMAIN_ID=0

CMD="${1:-status}"
shift 2>/dev/null || true

hdr() { printf "\n\033[1m═══ %s ═══\033[0m\n" "$*"; }
ok()  { printf "  \033[32m✓\033[0m %s\n" "$1"; }
bad() { printf "  \033[31m✗\033[0m %s\n" "$1"; }
inf() { printf "    %s\n" "$1"; }

mode_now() {
    if   pgrep -f async_slam_toolbox_node > /dev/null; then echo map
    elif pgrep -f "nav2_amcl/amcl"        > /dev/null; then echo loc
    else echo none; fi
}

case "$CMD" in

map)
    hdr "建圖模式"
    if [ "$(mode_now)" = "loc" ]; then
        inf "目前是使用地圖模式,會先停掉 AMCL 和 map_server"
    fi
    echo "  ★ 車現在停的位置就是新地圖的原點 map(0,0)。"
    echo "    這個點以後是「回家」的定點,地上建議做個實體標記。"
    echo
    exec bash "$D/bringup_all.sh"
    ;;

save)
    hdr "存地圖"
    [ "$(mode_now)" = "map" ] || { bad "現在不是建圖模式"; inf "bash robot.sh map"; exit 1; }
    exec bash "$D/map_save.sh" "$@"
    ;;

use)
    hdr "使用地圖模式"
    MAP="${1:-$(ls -t ~/maps/*.yaml 2>/dev/null | head -1)}"
    if [ -z "$MAP" ] || [ ! -f "$MAP" ]; then
        bad "找不到地圖"
        inf "~/maps 裡有:"
        ls -1t ~/maps/*.yaml 2>/dev/null | sed 's/^/      /' || inf "  (空的,先 robot.sh map 建一張)"
        exit 1
    fi
    echo "  地圖 $MAP"
    echo "  ★ 啟動後會把定位設在地圖原點。車要停在建圖起點那個定點,"
    echo "    不然要用 RViz 的 2D Pose Estimate 指定實際位置。"
    echo
    exec bash "$D/bringup_all.sh" loc "$MAP"
    ;;

home)
    hdr "回到初始點"
    exec bash "$D/reset_pose.sh" "$@"
    ;;

mark)
    hdr "標記目前位置"
    [ $# -ge 1 ] || { bad "要給名字:robot.sh mark 初始位置"; exit 1; }
    exec bash "$D/reset_pose.sh" save "$@"
    ;;

view)
    hdr "即時三維點雲"
    pkill -f live_cloud.py 2>/dev/null; sleep 1
    cd "$D" || exit 1
    setsid nohup python3 live_cloud.py 8100 > /tmp/live_cloud.log 2>&1 < /dev/null &
    sleep 4
    if pgrep -f live_cloud.py > /dev/null; then
        ok "已啟動"
        grep -oE "http://[0-9.]+:[0-9]+" /tmp/live_cloud.log | sed 's/^/    /'
        inf "看完關掉:bash robot.sh stop view"
    else
        bad "起不來"; tail -5 /tmp/live_cloud.log | sed 's/^/    /'
    fi
    ;;

stop)
    hdr "停止"
    if [ "${1:-all}" = "view" ]; then
        pkill -f live_cloud.py && ok "即時點雲已停" || inf "本來就沒在跑"
        exit 0
    fi
    # ★ 一律 SIGTERM。kill -9 會讓 zenoh session 沒機會關閉,對端留下殘留
    #   路由,累積成「同一個 topic 好幾個發布者」,RViz 收到重複資料而卡死。
    for p in live_cloud.py async_slam_toolbox_node "nav2_amcl/amcl" \
             "nav2_map_server/map_server" lifecycle_manager \
             pointcloud_to_laserscan ekf_node odom_cov_relay tf_relay_wheels \
             tf_static_repeat fastlio_mapping livox_ros_driver2_node \
             realsense2_camera_node zenoh-bridge; do
        pkill -f "$p" 2>/dev/null && inf "SIGTERM $p"
    done
    sleep 6
    LEFT=$(pgrep -af "slam_toolbox|amcl|fastlio|livox_ros|zenoh-bridge" | head -5)
    [ -z "$LEFT" ] && ok "都停了" || { inf "還沒走完,強制:"; echo "$LEFT" | sed 's/^/      /'
        pkill -9 -f "slam_toolbox|amcl|fastlio|livox_ros" 2>/dev/null; }
    ;;

status)
    hdr "狀態"
    M=$(mode_now)
    case "$M" in
      map)  ok "建圖模式(slam_toolbox 在跑)" ;;
      loc)  ok "使用地圖模式(AMCL 在跑)"
            Y=$(ps -eo cmd | grep -oP 'yaml_filename:=\K\S+' | head -1)
            [ -n "$Y" ] && inf "地圖 $Y" ;;
      none) bad "沒有在跑" ; inf "建圖 robot.sh map / 定位 robot.sh use" ;;
    esac

    echo
    printf "  %-26s %s\n" "底盤 $BASE_IP" \
      "$(ping -c1 -W1 "$BASE_IP" >/dev/null 2>&1 && echo 在線 || echo '✗ 不通')"
    for t in /scan /odom /cloud_registered_body; do
        H=$(timeout 8 ros2 topic hz "$t" 2>&1 | grep -oE "average rate: [0-9.]+" \
            | head -1 | grep -oE "[0-9.]+")
        printf "  %-26s %s\n" "$t" "${H:-✗ 沒資料} ${H:+Hz}"
    done

    if [ "$M" != "none" ]; then
        echo
        if timeout 12 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -q Translation; then
            P=$(timeout 12 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 \
                | grep -m1 Translation | sed 's/.*\[/[/')
            ok "我在地圖的 $P"
        else
            bad "查不到 map -> base_footprint"
            [ "$M" = "loc" ] && inf "AMCL 還沒收到初始位置:bash robot.sh home"
            [ "$M" = "map" ] && inf "slam_toolbox 還沒收到第一幀 /scan"
        fi
    fi
    ;;

*)
    sed -n '2,30p' "$0" | sed 's/^# \?//'
    exit 1 ;;
esac
