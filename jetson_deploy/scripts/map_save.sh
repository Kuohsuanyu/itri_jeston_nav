#!/usr/bin/env bash
# 把建圖模式下的 /map 存成檔案。
#
#   bash ~/slam2d/map_save.sh              存成 ~/maps/map_<時間戳>
#   bash ~/slam2d/map_save.sh 一樓走廊      存成 ~/maps/一樓走廊
#
# 會產生三個檔:
#   <名字>.pgm    佔據格點陣圖
#   <名字>.yaml   解析度、原點、門檻值 —— start_localization.sh 讀這個
#   <名字>.posegraph + .data   slam_toolbox 的位姿圖,之後可以**接著建**
#
# ── 兩件容易搞錯的事 ────────────────────────────────────────────
#
# 1. **地圖原點 = 你開始建圖時車所在的位置**,不是地圖圖片的角落。
#    yaml 裡的 origin 是「圖片左下角在 map 座標系的位置」,那個值是負的,
#    正是因為原點在圖片中間某處。
#
#    所以流程要固定成:把車停在你要當「初始位置」的定點 -> 開始建圖 ->
#    繞完存檔。之後 `reset_pose.sh` 不帶參數就是回到那個點,永遠正確。
#
# 2. map_saver 訂閱 /map 要用 transient_local。slam_toolbox 是 latched
#    發布(只在地圖更新時發),用預設的 volatile 訂閱會**等到逾時然後
#    存出一張空圖**,而且不會報錯 —— 檔案有產生、大小正常、打開全是灰的。
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"
DIR=~/maps
mkdir -p "$DIR"
OUT="$DIR/$NAME"

echo "=== 前置檢查 ==="
if ! pgrep -f async_slam_toolbox_node > /dev/null; then
    echo "  ✗ slam_toolbox 沒在跑 —— 現在不是建圖模式"
    echo "    建圖:bash ~/slam2d/bringup_all.sh"
    exit 1
fi
echo "  slam_toolbox 在跑"

# /map 是 latched,ros2 topic hz 量不到穩定頻率,改用 echo 確認拿得到內容
INFO=$(timeout 25 ros2 topic echo /map --once --field info 2>/dev/null)
W=$(echo "$INFO" | grep -oP '^width:\s*\K[0-9]+')
H=$(echo "$INFO" | grep -oP '^height:\s*\K[0-9]+')
if [ -z "$W" ] || [ "$W" = "0" ]; then
    echo "  ✗ /map 沒有內容。slam_toolbox 收到 /scan 了嗎?"
    echo "    ros2 topic hz /scan"
    exit 1
fi
echo "  /map $W x $H 格"

echo
echo "=== 存 $OUT ==="
# ★ map_subscribe_transient_local 一定要 true,見檔頭第 2 點
timeout 60 ros2 run nav2_map_server map_saver_cli \
    -f "$OUT" --occ 0.65 --free 0.25 --fmt pgm \
    --ros-args -p map_subscribe_transient_local:=true 2>&1 \
    | grep -viE "^\[INFO\].*(Creating|Configuring|Activating)" | sed 's/^/  /'

if [ ! -f "$OUT.yaml" ] || [ ! -f "$OUT.pgm" ]; then
    echo "  ✗ 沒有產生檔案"
    exit 1
fi

echo
echo "=== 位姿圖(之後可以接著建)==="
# 存了這個,以後 slam_toolbox 可以 deserialize 回來繼續繞,不用從頭建。
timeout 40 ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: '$OUT'}" 2>&1 \
    | tail -2 | sed 's/^/  /'

echo
echo "=== 結果 ==="
ls -lh "$OUT".* 2>/dev/null | awk '{printf "  %-10s %s\n", $5, $9}'
echo
sed 's/^/  /' "$OUT.yaml"
RES=$(grep -oP '^resolution:\s*\K[0-9.]+' "$OUT.yaml")
echo
python3 - "$W" "$H" "$RES" <<'PY'
import sys
w, h, r = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
print("  涵蓋範圍 %.1f x %.1f 公尺" % (w * r, h * r))
PY

cat <<MSG

=== 接下來 ===
  用這張圖定位:
      bash ~/slam2d/bringup_all.sh loc $OUT.yaml

  ★ 地圖原點 map(0,0) 就是你**開始建圖時**車停的位置。
    把車開回那裡,bash ~/slam2d/reset_pose.sh 就能重設定位。
    那個點值得在地上做個實體標記。
MSG
