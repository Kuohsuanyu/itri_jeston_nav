#!/usr/bin/env bash
# 啟動 RViz2,處理 WSLg 上那個「建不出 render window」的老問題。
#
# 症狀:
#   InvalidParametersException: Window with name 'OgreWindow(0)' already exists
#   Unable to create the rendering window after 100 tries
#   Aborted (core dumped)
#
# ★ 那個訊息是**症狀不是原因**。第一次 createRenderWindow 因為別的理由失敗,
#   RViz 用同一個名字重試 100 次,第 2 次起才變成「名字已存在」。
#   真正的失敗原因被吃掉了,log 上看不到。
#
# 實測在 WSLg 上是間歇性的 —— 同樣的指令有時成,有時不成。所以策略是重試,
# 不是去猜。三階:
#   1. 原樣重試 3 次(通常第 2 次就過)
#   2. 軟體算繪 llvmpipe(硬體 GL 直通壞掉時)
#   3. 叫使用者 wsl --shutdown(WSLg 本身要重來)
#
# ★ 清殘留不要用 pkill -f rviz2 —— 執行這行的 shell,它自己的 cmdline 裡
#   就有 "rviz2" 這個字串,-f 會比對到而把自己殺掉。用 pgrep 取 PID 並
#   排除自己。同樣的坑在 zenoh-bridge-ros2dds 上也踩過。
source "$HOME/lidar_view/ros_env.sh"

CFG="${1:-$HOME/lidar_view/live.rviz}"

# 等到「一個 rviz2 都不在」為止才回傳。
#
# ★ 只 kill 完就馬上重開會失敗:崩潰的行程要寫 core dump,寫完之前
#   X 的視窗資源還沒放掉,新的 rviz2 就撞 OgreWindow(0) already exists。
#   2026-08-11 實測:kill 之後隔 3 秒重試仍會撞,要等到 pgrep 真的空了。
cleanup_stale() {
    local me=$$ pids i
    for i in 1 2 3 4 5 6 7 8 9 10; do
        pids=$(pgrep -x rviz2 2>/dev/null | grep -v "^${me}$" || true)
        [ -z "$pids" ] && { [ "$i" -gt 1 ] && echo "  視窗資源已釋放"; return 0; }
        if [ "$i" -eq 1 ]; then
            echo "  有其他 rviz2 在跑($pids),先關掉"
            kill $pids 2>/dev/null || true
        elif [ "$i" -eq 5 ]; then
            echo "  還不肯走,強制"
            kill -9 $pids 2>/dev/null || true
        fi
        sleep 2
    done
    echo "  ⚠ 10 秒後仍有 rviz2 存在,硬上會撞視窗名稱"
    return 1
}

attempt() {
    local label="$1"; shift
    echo "  嘗試:$label"
    cleanup_stale
    # exec 掉的話失敗就沒得重試,所以先跑在子行程觀察它活不活得過初始化。
    # RViz 建視窗失敗會在 10 秒內 abort;活過 12 秒就是真的起來了。
    env "$@" rviz2 -d "$CFG" &
    local pid=$!
    local i=0
    while [ $i -lt 12 ]; do
        sleep 1
        kill -0 $pid 2>/dev/null || return 1
        i=$((i + 1))
    done
    echo "  起來了(PID $pid)。關掉視窗即結束。"
    wait $pid
    return 0
}

echo "=== RViz2  設定檔 $CFG ==="

for n in 1 2 3; do
    attempt "第 $n 次(硬體 GL)" X=1 && exit 0
    echo "  失敗,等視窗資源釋放後重試"
    sleep 5
done

echo
echo "硬體 GL 三次都失敗,改用軟體算繪(會比較慢)"
attempt "llvmpipe" LIBGL_ALWAYS_SOFTWARE=1 && {
    echo
    echo "★ 硬體 GL 壞了但軟體可以。要固定下來的話,把這行的註解拿掉:"
    echo "    ~/lidar_view/ros_env.sh 裡的 export LIBGL_ALWAYS_SOFTWARE=1"
    exit 0
}

echo
echo "★ 兩種都失敗 —— WSLg 本身要重來:"
echo "    1. 關掉所有 WSL 視窗"
echo "    2. 在 Windows 的 PowerShell 執行:wsl --shutdown"
echo "    3. 重開 WSL,然後 ~/lidar_view/start_zenoh.sh"
exit 1
