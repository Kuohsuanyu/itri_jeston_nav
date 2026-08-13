#!/usr/bin/env bash
# 從 Jetson 關掉底盤(樹莓派)。
#
#   bash ~/slam2d/base_off.sh          10 秒後關機
#   bash ~/slam2d/base_off.sh 30       30 秒後
#   bash ~/slam2d/base_off.sh cancel   取消倒數中的關機
#
# ── 為什麼要這支而不是直接 ros2 service call ──────────────────────
# 1. 那串指令太長,而且 chassis_msgs 要先 source ~/chassis_ws/install
#    ——忘了 source 的錯誤訊息是「找不到型別」,看不出是環境問題。
# 2. 關機服務會做靜止檢查,但**它的檢查會誤判**:
#    2026-08-12 實測回 "motor_state is stale (3596.8s)",而那個 topic
#    其實一直在發 10 Hz —— 是底盤自己的訂閱斷了(同機 DDS 問題),
#    不是車在動。這支會先從 Jetson 自己讀一次 rpm 確認,再決定要不要用 force。
# 3. 關完自動 ping 確認真的斷線。回傳 success 不等於真的關掉 ——
#    sudoers 沒裝的話會排程成功但執行失敗,而且只在底盤的 log 裡才看得到。
source /opt/ros/humble/setup.bash
source ~/chassis_ws/install/setup.bash 2>/dev/null
source ~/slam2d/robot_env.sh          # BASE_IP 的正本
export ROS_DOMAIN_ID=0

CONFIRM="SHUTDOWN"      # 要跟底盤 system_param.yaml 的 confirm_code 一致

if [ "$1" = "cancel" ]; then
    echo "=== 取消倒數中的關機 ==="
    timeout 25 ros2 service call /system/shutdown_cancel std_srvs/srv/Trigger "{}" \
        2>&1 | tail -3
    exit 0
fi

DELAY="${1:-10}"

echo "=== 底盤在不在 ==="
if ! ping -c 2 -W 2 "$BASE_IP" > /dev/null 2>&1; then
    echo "  ✗ ping 不到 $BASE_IP"
    if ping -c 2 -W 2 "$BASE_IP_WIFI" > /dev/null 2>&1; then
        echo "    但無線 $BASE_IP_WIFI 通 —— 底盤開著,是有線那條不通。"
        echo "    要用無線關機:BASE_IP=$BASE_IP_WIFI bash ~/slam2d/base_off.sh"
    else
        echo "    無線也不通 —— 已經關了,或還沒開機"
    fi
    exit 1
fi
echo "  通($BASE_IP)"

echo
echo "=== 關機前記錄 ==="
timeout 10 ros2 topic echo /battery_state --once 2>/dev/null \
    | grep -E "^(voltage|current|percentage)" | sed 's/^/  /'

echo
echo "=== 從 Jetson 確認車是靜止的 ==="
# ★ 自己讀,不要只依賴底盤的靜止檢查 —— 它會因為訂閱斷掉而誤判成
#   "state unknown",那時車其實停得好好的。
MS=$(timeout 12 ros2 topic echo /chassis/motor_state --once 2>/dev/null)
if [ -z "$MS" ]; then
    echo "  ⚠ 讀不到 /chassis/motor_state。底盤的 driver 可能停了。"
    echo "    無法確認車是否靜止 —— 請目視確認之後再手動下:"
    echo "    ros2 service call /system/shutdown chassis_msgs/srv/Shutdown \\"
    echo "      \"{confirm: '$CONFIRM', delay_sec: $DELAY, reason: '手動', force: true}\""
    exit 1
fi
echo "$MS" | grep -E "rpm|alarm" | sed 's/^/  /'
LR=$(echo "$MS" | grep -oP 'left_rpm:\s*\K-?\d+' | head -1)
RR=$(echo "$MS" | grep -oP 'right_rpm:\s*\K-?\d+' | head -1)
if [ "${LR:-1}" != "0" ] || [ "${RR:-1}" != "0" ]; then
    echo "  ✗ 車還在動(左 $LR / 右 $RR rpm)—— 不關機"
    exit 1
fi
echo "  靜止確認"

echo
echo "=== 送出關機($DELAY 秒後)==="
# 先照正常流程(force: false),讓底盤自己也檢查一次。
OUT=$(timeout 40 ros2 service call /system/shutdown chassis_msgs/srv/Shutdown \
      "{confirm: '$CONFIRM', delay_sec: $DELAY, reason: '從 Jetson 關機', force: false}" 2>&1)
echo "$OUT" | tail -3 | sed 's/^/  /'

if echo "$OUT" | grep -q "success=False"; then
    echo
    if echo "$OUT" | grep -qi "stale\|state unknown"; then
        # 這是已知的誤判:底盤的 system_service 訂閱斷了,但車確實靜止
        # (上面已經從 Jetson 讀過 rpm 都是 0)。
        echo "  底盤說它不知道車的狀態,但我們剛確認過是靜止的 —— 改用 force"
        timeout 40 ros2 service call /system/shutdown chassis_msgs/srv/Shutdown \
          "{confirm: '$CONFIRM', delay_sec: $DELAY, reason: '從 Jetson 關機(force)', force: true}" \
          2>&1 | tail -3 | sed 's/^/  /'
    else
        echo "  ✗ 被拒絕,而且原因不是狀態未知。不強制。"
        exit 1
    fi
fi

echo
echo "=== 等待斷線(最多 60 秒)==="
for i in $(seq 1 12); do
    sleep 5
    # ★ 有線和無線都要斷才算真的關機。只斷有線可能只是網路線鬆了。
    if ! ping -c 2 -W 2 "$BASE_IP" > /dev/null 2>&1 &&
       ! ping -c 2 -W 2 "$BASE_IP_WIFI" > /dev/null 2>&1; then
        echo "  ✓ 第 $((i * 5)) 秒斷線 —— 已關機"
        exit 0
    fi
    echo "  第 $((i * 5)) 秒:還通"
done

echo
echo "  ✗ 60 秒後還連得上 —— 排程成功但沒真的關機"
echo "    最常見原因是 sudoers 沒裝,底盤的 log 會有:"
echo "        shutdown command exited with 1"
echo "    到底盤上裝:"
echo "        sudo install -m 0440 -o root -g root \\"
echo "          ~/<ws>/src/chassis-ros2-driver/chassis_system/deploy/chassis-shutdown.sudoers \\"
echo "          /etc/sudoers.d/chassis-shutdown"
echo "        sudo sed -i \"s/<USER>/\$USER/\" /etc/sudoers.d/chassis-shutdown"
echo "        sudo visudo -c"
exit 1
