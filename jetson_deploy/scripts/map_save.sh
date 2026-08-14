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
# 2. ★ map_saver 的兩個參數都要改,預設值存不出來。實測 2026-08-14:
#
#    map_subscribe_transient_local 要 **false**(預設 true)
#      我原本以為 latched 發布非用 transient_local 不可,那是錯的。
#      zenoh bridge 會在 /map 註冊一個 VOLATILE 端點,訂閱端要求
#      TRANSIENT_LOCAL 時跟它不相容,rclcpp 就整個放棄:
#          [WARN]  offering incompatible QoS ... DURABILITY_QOS_POLICY
#          [ERROR] Failed to spin map subscription
#      -> 完全不產生檔案。
#      用 volatile 照樣拿得到 —— slam_toolbox 每 map_update_interval
#      (1.0 秒)就重發,不必依賴 latch。實測車靜止時 20 秒收到 19 則。
#
#    save_map_timeout 要 **20.0**(預設 2.0)
#      2 秒理論上夠(1 Hz),但加上訂閱配對和第一則的傳輸就很邊緣,
#      實測會間歇性 "Failed to spin map subscription"。給寬沒有代價。
#
# 3. ★ 存完一定要驗證圖不是空的。這是會安靜失敗的那種錯誤:
#    檔案有產生、大小正常、yaml 也對,打開卻整片灰。所以下面會實際解析
#    pgm 數格子,佔據格太少就當失敗。
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

verify_pgm() {   # 回傳 0 = 有內容
    python3 - "$1.pgm" <<'PY'
import sys
import numpy as np
try:
    d = open(sys.argv[1], "rb").read()
except OSError:
    print("    ✗ 讀不到檔案"); sys.exit(1)
i, tok = 2, []
while len(tok) < 3:
    while d[i:i + 1].isspace():
        i += 1
    if d[i:i + 1] == b"#":
        while d[i:i + 1] not in (b"\n", b""):
            i += 1
        continue
    s = i
    while not d[i:i + 1].isspace():
        i += 1
    tok.append(int(d[s:i]))
i += 1
w, h, _ = tok
a = np.frombuffer(d[i:i + w * h], dtype=np.uint8)
occ = int((a < 90).sum()); free = int((a > 220).sum())
print("    %d x %d 格   佔據 %d   空曠 %d   未知 %d"
      % (w, h, occ, free, len(a) - occ - free))
if occ < 50 or free < 500:
    print("    ✗ 幾乎是空的 —— 存出來的圖沒有用")
    sys.exit(1)
PY
}

echo
echo "=== 存 $OUT ==="
# ★ 兩個參數都是實測調出來的,不要改回預設:
#
#   map_subscribe_transient_local:=false
#       true 會撞上 zenoh bridge 在 /map 註冊的 VOLATILE 端點:
#         [WARN]  offering incompatible QoS ... DURABILITY_QOS_POLICY
#         [ERROR] Failed to spin map subscription
#       然後**完全不產生檔案**。
#
#   save_map_timeout:=20.0
#       預設 2.0 秒太短,實測會 "Failed to spin map subscription"。
#       /map 車靜止時仍以約 1 Hz 發布(20 秒 19 則),2 秒理論上夠,
#       但加上訂閱配對和第一則的傳輸就很邊緣 —— 給寬一點沒有代價,
#       收到就會馬上返回。
#
# ★ 不要用 /slam_toolbox/save_map 服務當備援。2026-08-14 實測它一律回
#   result=255,英數檔名和中文檔名都一樣 —— 這個版本的那個服務是壞的。
timeout 90 ros2 run nav2_map_server map_saver_cli \
    -f "$OUT" --occ 0.65 --free 0.25 --fmt pgm \
    --ros-args -p map_subscribe_transient_local:=false \
               -p save_map_timeout:=20.0 2>&1 \
    | grep -viE "lifecycle node launched|Waiting on external|design.ros2.org" \
    | sed 's/^/  /'

if [ ! -f "$OUT.yaml" ] || [ ! -f "$OUT.pgm" ] || ! verify_pgm "$OUT"; then
    echo
    echo "  ✗ 存不出有內容的地圖"
    echo "    確認 /map 真的有東西:"
    echo "      ros2 topic echo /map --once --field info"
    echo "    確認 slam_toolbox 還活著:"
    echo "      pgrep -af async_slam_toolbox_node"
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
