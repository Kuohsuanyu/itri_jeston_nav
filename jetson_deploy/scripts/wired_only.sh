#!/usr/bin/env bash
# 把「Jetson ↔ 底盤」這條逼到有線上,不動底盤任何設定。
#
#   bash ~/slam2d/wired_only.sh          套用(切斷通往底盤無線位址的路)
#   bash ~/slam2d/wired_only.sh off      還原
#   bash ~/slam2d/wired_only.sh status   看現在是什麼狀態
#
# ── 問題 ────────────────────────────────────────────────────────
# 樹莓派兩張網卡都開著,DDS 就會**同時公告兩個 locator**:
#     192.168.0.101    有線
#     192.168.40.160   無線
# 對端挑哪一條不保證,而且會隨時間漂。實測(各聽 8 秒,只過濾底盤的 IP):
#
#     enP8p1s0 有線   1015 個 DDS 封包
#     wlP1p1s0 無線    697 個 DDS 封包      ← 這些是問題來源
#
# 兩條的品質差三個數量級:
#
#     有線 192.168.0.101    0% 遺失   平均 0.296 ms   抖動 0.032 ms
#     無線 192.168.40.160   0% 遺失   平均 218 ms     抖動 215 ms
#
# 挑到無線,底盤的 /odom /joint_states 就延遲兩百毫秒且抖動同等級 ——
# 那是「有時候延遲」和「旋轉不穩」的直接來源。
#
# ── 做法 ────────────────────────────────────────────────────────
# 在 Jetson 上讓 192.168.40.160 變成走不通:
#
#   1. blackhole 路由    送出去的封包立刻回 EHOSTUNREACH,不是逾時。
#                        Fast DDS 會很快放棄那個 locator 改用有線的;
#                        用 DROP 的話它要等到逾時,反而更慢。
#   2. iptables 擋進來的  底盤從無線那張網卡送來的東西一律丟掉,
#                        免得單向路徑造成一半資料走無線一半走有線。
#
# 底盤完全不用改 —— 它照樣廣告兩個位址,只是其中一個對我們是死路。
#
# ★ 代價:有線斷掉時**不會自動退回無線**,底盤會整個失聯。
#   這是刻意的 —— 半通不通的無線比乾脆斷掉更難查。
#   有線真的壞了就跑 wired_only.sh off。
#
# ★ ip route 和 iptables 都**不持久**,重開機自動消失。
#   bringup_all.sh 會在啟動時呼叫這支,所以正常流程不用手動下。
#
# ── ★ 順序不能反 ────────────────────────────────────────────────
# DDS 的端點配對是**建立時決定的**。底盤的節點啟動時如果兩條都通,它挑了
# 無線就固定下來;之後才切斷無線,只會讓已配對的連線斷掉,**不會**自動
# 改用有線。2026-08-14 實測:套用後 /odom 立刻掉到 0 則,等 45 秒也沒回來,
# 還原後馬上恢復 139 則。
#
# 所以正確流程是:
#     1. 在 Jetson 套用這支            bash ~/slam2d/wired_only.sh on
#     2. 重啟底盤的 ROS(重開樹莓派或重跑它的 bringup)
#     3. 驗證                           bash ~/slam2d/wired_only.sh status
#
# 反過來做的話會直接把底盤資料切斷。
source ~/slam2d/robot_env.sh

# ── sudo ────────────────────────────────────────────────────────
# ★ 這支會被 bringup_all.sh 自動呼叫,那時是非互動的,sudo 讀不到密碼
#   就會**靜默失敗** —— 規則沒設成,但輸出看起來一切正常。
#   2026-08-14 就這樣白跑了一次。所以這裡要明確檢查並大聲報錯。
if sudo -n true 2>/dev/null; then
    SUDO="sudo -n"
else
    SUDO=""
fi

need_sudo() {
    [ -n "$SUDO" ] && return 0
    echo "  ✗ 沒有免密碼 sudo,規則設不上(非互動的 SSH 讀不到密碼)"
    echo
    echo "    裝一條只允許這兩個指令的規則就好:"
    echo "      bash ~/slam2d/wired_only.sh install-sudoers"
    echo
    echo "    或是自己在有終端機的 shell 裡跑:"
    echo "      sudo ip route add blackhole $BASE_IP_WIFI"
    echo "      sudo iptables -I INPUT 1 -s $BASE_IP_WIFI -j DROP"
    return 1
}

have_route() { ip route show | grep -q "blackhole $BASE_IP_WIFI"; }
have_rule()  { $SUDO iptables -C INPUT -s "$BASE_IP_WIFI" -j DROP 2>/dev/null; }

case "${1:-on}" in

install-sudoers)
    # 只放行這兩個指令,而且參數寫死 —— 不是給整個 ip / iptables 免密碼。
    F=/etc/sudoers.d/wired-only
    echo "=== 安裝 $F ==="
    echo "  只放行兩條指令,參數寫死,不是整個 ip/iptables 免密碼:"
    T=$(mktemp)
    cat > "$T" <<RULES
# 讓 wired_only.sh 可以在非互動的 SSH 下設定路由和防火牆規則。
# 只放行這幾條,參數完全寫死。
$USER ALL=(root) NOPASSWD: /usr/sbin/ip route add blackhole $BASE_IP_WIFI
$USER ALL=(root) NOPASSWD: /usr/sbin/ip route del blackhole $BASE_IP_WIFI
$USER ALL=(root) NOPASSWD: /usr/sbin/iptables -I INPUT 1 -s $BASE_IP_WIFI -j DROP
$USER ALL=(root) NOPASSWD: /usr/sbin/iptables -D INPUT -s $BASE_IP_WIFI -j DROP
$USER ALL=(root) NOPASSWD: /usr/sbin/iptables -C INPUT -s $BASE_IP_WIFI -j DROP
RULES
    sed 's/^/    /' "$T"
    echo
    echo "  需要你輸入一次密碼:"
    sudo install -m 0440 -o root -g root "$T" "$F" && rm -f "$T" || { rm -f "$T"; exit 1; }
    sudo visudo -c -f "$F" && echo "  ✓ 語法檢查通過" || { echo "  ✗ 語法錯,移除"; sudo rm -f "$F"; exit 1; }
    echo "  ✓ 裝好了。之後 bringup_all.sh 可以自動套用。"
    exit 0 ;;

status)
    echo "=== wired_only 狀態 ==="
    have_route && echo "  ✓ blackhole 路由 $BASE_IP_WIFI 已設" \
               || echo "  ✗ blackhole 路由 未設"
    have_rule  && echo "  ✓ iptables 阻擋 $BASE_IP_WIFI 已設" \
               || echo "  ✗ iptables 阻擋 未設"
    echo
    echo "  底盤有線 $BASE_IP:"
    ping -c 5 -q "$BASE_IP" 2>/dev/null | tail -2 \
      | sed -E 's/.*, ([0-9.]+)% packet loss.*= [0-9.]+\/([0-9.]+)\/[0-9.]+\/([0-9.]+) ms.*/    遺失 \1%  平均 \2 ms  抖動 \3 ms/'
    echo "  底盤無線 $BASE_IP_WIFI:"
    ping -c 3 -W 1 -q "$BASE_IP_WIFI" 2>/dev/null | tail -2 \
      | sed -E 's/.*, ([0-9.]+)% packet loss.*/    遺失 \1%(套用後應該是 100%)/' \
      || echo "    走不通(預期)"
    exit 0 ;;

off)
    echo "=== 還原:允許無線路徑 ==="
    need_sudo || exit 1
    have_route && { $SUDO ip route del blackhole "$BASE_IP_WIFI" && echo "  已移除 blackhole 路由"; } \
               || echo "  blackhole 路由本來就沒設"
    while have_rule; do
        $SUDO iptables -D INPUT -s "$BASE_IP_WIFI" -j DROP && echo "  已移除 iptables 規則"
    done
    echo
    echo "  ★ DDS 要重新探索才會用回無線 —— 重啟相關節點,或等它自己重試。"
    exit 0 ;;

on|"")
    echo "=== 把底盤這條逼到有線 ==="
    if ! ping -c 2 -W 2 "$BASE_IP" > /dev/null 2>&1; then
        echo "  ✗ 有線 $BASE_IP 不通 —— 現在切會讓底盤完全失聯,中止"
        echo "     先確認交換器的線,或用無線跑:"
        echo "       BASE_IP=$BASE_IP_WIFI bash ~/slam2d/bringup_all.sh loc"
        exit 1
    fi
    echo "  有線 $BASE_IP 通,可以切"
    need_sudo || exit 1

    have_route || $SUDO ip route add blackhole "$BASE_IP_WIFI"
    have_route && echo "  ✓ blackhole 路由 $BASE_IP_WIFI"

    have_rule || $SUDO iptables -I INPUT 1 -s "$BASE_IP_WIFI" -j DROP
    have_rule && echo "  ✓ iptables 丟棄來自 $BASE_IP_WIFI 的封包"

    echo
    echo "  ★ 現在去重啟底盤的 ROS(重開樹莓派,或重跑它的 bringup)。"
    echo "    不重啟的話已配對的連線會斷掉而且不會自己改走有線 ——"
    echo "    症狀是 /odom 立刻掉到 0 則。"
    echo
    echo "  驗證(8 秒):底盤的 DDS 流量應該只剩有線"
    for IF in "$WIRED_IF" wlP1p1s0; do
        N=$($SUDO timeout 8 tcpdump -i "$IF" -nn -q \
              "udp and (host $BASE_IP or host $BASE_IP_WIFI)" 2>/dev/null | wc -l)
        printf "    %-12s %5d 個封包%s\n" "$IF" "$N" \
            "$([ "$IF" = "wlP1p1s0" ] && { [ "$N" -lt 20 ] && echo "  ✓" || echo "  ★ 還有殘留,節點要重啟才會換 locator"; })"
    done
    echo
    echo "  底盤資料還在嗎:"
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=0
    for t in /odom /joint_states; do
        H=$(timeout 8 ros2 topic hz "$t" 2>&1 | grep -oE "average rate: [0-9.]+" \
            | head -1 | grep -oE "[0-9.]+")
        printf "    %-16s %s\n" "$t" "${H:-✗ 沒資料 —— 立刻跑 wired_only.sh off}"
    done
    ;;

*)
    sed -n '2,8p' "$0" | sed 's/^# \?//'
    exit 1 ;;
esac
