#!/usr/bin/env bash
# 機器人座標樹 —— 感測器接在底盤的 base_link 下面。
#
#   bash robot_tf.sh fused        EKF 融合輪速 + 光達  (預設,最準)
#   bash robot_tf.sh chassis      只用輪速,光達在 SLAM 層修正
#   bash robot_tf.sh standalone   底盤沒開,只用光達
#
# ── 目標結構 ────────────────────────────────────────────────────
#   fused(預設。樹莓派 publish_tf: false,那一段由 EKF 接手)
#     map ─(slam)─ multi_odom ─(EKF)─ base_footprint ─(底盤rsp)─ base_link ─┬─ wheel_*
#                                                                            └─ box_link ─┬─ body
#   chassis(樹莓派 publish_tf: true,只用輪速)                                              └─ camera_link
#     map ─(slam)─ odom ─(輪速)───────  base_footprint ─(底盤rsp)─ base_link ─┬─ wheel_*
#                                                                            └─ box_link ─ …
#   standalone(底盤整台沒開,base_link 不存在)
#     map ─(slam)─ lidar_odom ─(FAST-LIO)───────────────────────────────── box_link ─ …
#
#   ★ 三種模式共同點:box_link 以下(body / camera_link)完全一樣,
#     那是校準出來的剛性關係,跟里程計無關。
#     不同的只有「base_footprint 或 box_link 的父節點是誰發的」。
#
#   ★ fused 為什麼要樹莓派關 TF:
#     slam_toolbox 只從 TF 讀里程計,沒有 odom topic 輸入。要讓它吃融合值,
#     融合值就必須「是」base_footprint 的父節點 —— 那個位置底盤原本佔著。
#     兩邊都發 = base_footprint 有兩個父節點,tf2 不報錯只會隨機翻轉。
#     所以是取代,不是並存。
#
#     Nav2 的 footprint 不受影響:footprint 是定義在 base_link 上的多邊形,
#     box_link 只是掛在它底下的一段偏移,底盤外形沒有變。
#
# ── 為什麼中間要插 box_link ─────────────────────────────────────
# 感測器鎖在理線盒裡,不是直接鎖在車上。把「盒子在車上哪裡」和「感測器在盒子
# 裡哪裡」拆成兩段,各自的來源完全不同:
#
#     base_link ──(上車後捲尺量,盒子挪動就要改)── box_link
#     box_link  ──(calib_box.py 量的,剛性不變)── body / camera_link
#
# 混在一起的話,盒子在車上挪 2 公分就要同時改四個數字,而且很容易只改一半。
#
# ── FAST-LIO 的 /tf 必須關掉 ────────────────────────────────────
# 它發 camera_init -> body。box_link 也要發 body,body 就有兩個父節點 ——
# tf2 **不會報錯**,它會交替採用兩者,整棵樹以 10 Hz 抖動,症狀看起來像
# 里程計在飄。startall.sh 用 -r /tf:=/tf_fastlio_unused 把它靜音。
#
# FAST-LIO 的位姿不會浪費,三種模式各有去處:
#   fused       odom_cov_relay.py 把 /Odometry 換算成 base_footprint 的位姿
#               餵給 EKF,跟輪速融合
#   chassis     slam_toolbox 拿 /scan 做掃描匹配,變成 map -> odom 的修正量
#   standalone  fastlio_base_tf.py 讀 /Odometry 換算成 lidar_odom -> box_link
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

MODE="${1:-fused}"
D=~/slam2d

# ============ base_link -> box_link ============
# 原點在盒子的**底面中心**(盒子鎖在支架上的那個安裝面)。
#
# ★ 2026-08-12 用捲尺直接量,取代先前的地面擬合值。
#
#   盒底離地              0.5300   ← 捲尺
#   base_link 離地        0.2089   ← 見下面「車型」
#                         ──────
#   BOX_Z                 0.3211
#
#   盒中心往前            0.1300   ← 捲尺,base_link 原點在底盤幾何中心
#                                    (base_link.STL 的 X 是 ±0.505,對稱)
#
# ── ★ 車型是 DD-M-HH(加高版),不是 DD-M ────────────────────────
# 兩份 xacro 只差在 wheel_z,但 base_link 的高度就差 5.7 mm:
#   DD-M      wheel_z  0           base_link 離地 0.2032
#   DD-M-HH   wheel_z -0.0056939   base_link 離地 0.2089
# 實機 tf2_echo base_footprint base_link 回 0.209 —— 是 DD-M-HH。
# 上蓋高度差更多:DD-M 的 STL 頂面 0.2490,DD-M-HH 是 0.3098。
# 拿錯車型算的話盒子會憑空浮 6 公分,2026-08-12 就這樣誤判過。
#
#   DD-M-HH 上蓋          0.3098   (5464 個頂點的大平面)
#   BOX_Z                 0.3211
#                         ──────
#   盒底離上蓋            0.0113   一片安裝板的厚度,基本上是貼著的。
#
# ── 和先前地面擬合值的差異 ──────────────────────────────────────
# 2026-08-11 calib_height.py 用 RANSAC 擬合地面得到 0.3705(內點
# 53258/135595,法線離垂直 0.49 度),比捲尺高 4.4 cm。擬合是把整條鏈的
# 誤差**全部歸給這一段**,所以那 4.4 cm 其實落在別處 —— 最可能是
# BODY_Z(光達在盒內的高度)或 BODY_PITCH 造成的地面傾斜殘差。
# 捲尺量的是單一段、沒有誤差轉嫁,優先採用。
#
# ★ 這個值錯的後果:start_scan.sh 的 /scan 高度帶會整段平移,
#   本來要切離地 0.10~1.50,錯 12 公分就變成 0.22~1.62 —— 矮障礙物看不到。
BOX_X=0.1300
BOX_Y=0.0000
BOX_Z=0.3211
BOX_YAW=0.0000

# ============ box_link -> body(Mid-360 的 IMU 座標系)============
# calib_box.py 2026-08-10 實測。Z 0.2005 就是「光達離盒底」的量測值,
# 不是靠盒高疊出來的 —— 比盒高 0.160 多出來的 4.05 cm 就是支架。
# pitch 0.5181 = 29.69 度,IMU 重力 / 地面法線 / 相機 pitch 三者交叉驗證。
#
# ★ body 這個名字改不掉 —— FAST-LIO 寫死在原始碼裡,參數列表裡沒有
#   任何 frame 相關設定。與其多接一段 lidar_link 的恆等變換讓人誤以為
#   那裡有實體意義,不如直接用 body 並在這裡註明它是什麼。
BODY_X=0.0250
BODY_Y=-0.0000
BODY_Z=0.2005
BODY_ROLL=0.0112
BODY_PITCH=0.5181
BODY_YAW=0.0000

# ============ box_link -> camera_link(D435F)============
# camera_link 這個名字也不能改 —— realsense2_camera 驅動自己會發
# camera_link -> camera_color_frame 那一串,名字對不上就接不起來。
CAM_X=0.0892
CAM_Y=-0.0454
CAM_Z=0.1190
CAM_ROLL=0.0092
CAM_PITCH=-0.0278
CAM_YAW=0.0154
# =====================================================================

pub() {   # pub parent child x y z roll pitch yaw
    setsid nohup ros2 run tf2_ros static_transform_publisher \
        --x "$3" --y "$4" --z "$5" --roll "$6" --pitch "$7" --yaw "$8" \
        --frame-id "$1" --child-frame-id "$2" \
        > "/tmp/tf_$2.log" 2>&1 < /dev/null &
    printf "  %-11s -> %-12s (%s, %s, %s)  rpy(%s, %s, %s)\n" \
        "$1" "$2" "$3" "$4" "$5" "$6" "$7" "$8"
}

echo "=== 模式:$MODE ==="

# 清掉所有我們可能發過的段(包含歷次改名留下的孤兒)。
# 樹莓派的發布者不在這台機器上,pkill 殺不到也不該殺。
for ch in box_link body camera_link wiring_box base_lidar camera_init base_link base_footprint; do
    pkill -f "static_transform_publisher.*--child-frame-id $ch" 2>/dev/null
done
pkill -f fastlio_base_tf.py 2>/dev/null
pkill -f tf_static_repeat.py 2>/dev/null
pkill -f tf_relay_wheels.py 2>/dev/null
pkill -f odom_cov_relay.py 2>/dev/null
pkill -f ekf_node 2>/dev/null
sleep 2

case "$MODE" in
fused)
    # EKF 融合,直接**取代**底盤原本發的 odom -> base_footprint。
    #
    # 前提:樹莓派的 chassis_bringup/config/vehicle_param_DD-M.yaml
    #       設 publish_tf: false。
    #       chassis_driver_node.py:180 的 `if self._publish_tf:` 只包住 TF,
    #       /odom topic 的發布在它外面 —— 資料照送,只是不發 TF。
    #       base_footprint -> base_link 和輪子由 bringup 的 robot_state_publisher
    #       發,不受影響。
    #
    # 為什麼要取代而不是並存:slam_toolbox 只從 TF 讀里程計
    # (odom_frame -> base_frame),沒有 odom topic 輸入。要讓它吃融合後的
    # 估計,那個估計就必須「是」base_footprint 的父節點。
    #
    # 之前為了不動樹莓派,改成發 multi_odom -> box_link,結果 TF 斷成兩棵。
    # 現在關掉底盤的 TF,那一段空出來由 EKF 接手,併回一棵。
    echo "--- 里程計:EKF 融合(輪速 + 光達)-> multi_odom -> base_footprint ---"
    if ! timeout 6 ros2 topic hz /odom 2>/dev/null | head -1 | grep -q rate; then
        echo "  ⚠ /odom 沒資料 —— 少了輪速那一路,EKF 會只靠光達"
    fi
    # ── 讓底盤停發 odom -> base_footprint ────────────────────────────
    # 兩邊都發的話 base_footprint 會有兩個父節點,tf2 不報錯,只會在兩個
    # 答案之間隨機翻轉 —— 症狀看起來像里程計在飄,很難查。
    #
    # ITRI 在 faa4bfd 把 publish_tf 開放成執行期可寫(broadcaster 無條件建構,
    # 另外掛 on_set_parameters callback),所以上位機可以直接遠端關掉,
    # 不用登入底盤。README 5.4 明講這是給買方上位機用的。
    #
    # ★ 每次都要重設:這個參數**不保存**,底盤一重開機就回到參數檔的 true。
    #   靠人記得遲早會漏,所以每次啟動都無條件設一次。
    echo "  遠端關閉底盤的 TF 廣播"
    if ! timeout 20 ros2 param set /chassis_driver publish_tf false 2>&1 | grep -q successful; then
        echo "  ⚠ 設不動 —— 底盤可能是舊版(faa4bfd 之前 publish_tf 是唯讀的)"
        echo "    舊版要在底盤上重啟:"
        echo "    ros2 launch chassis_bringup bringup.launch.py localization_mode:=external_takeover"
    fi
    sleep 3

    # 設完還是要驗。參數設成功不等於 TF 真的停了 —— 舊版就是「設得動但沒作用」。
    #
    # ★ 不能用 `ros2 topic echo /tf --once`:那只抓**一則**訊息,而 /tf 上同時
    #   有好幾個發布者。抓到的那則剛好不是底盤發的,grep 就落空、守門就放行。
    #   2026-08-11 就是這樣漏掉,實際跑出了雙父節點。改成連續取樣 4 秒。
    if timeout 10 python3 -c '
import sys, time, rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
rclpy.init(); n = Node("tf_guard"); hit = []
n.create_subscription(TFMessage, "/tf",
    lambda m: [hit.append(t.header.frame_id) for t in m.transforms
               if t.child_frame_id == "base_footprint"], 50)
t0 = time.time()
while time.time() - t0 < 4 and not hit:
    rclpy.spin_once(n, timeout_sec=0.1)
sys.exit(0 if hit else 1)
' 2>/dev/null; then
        echo "  ✗ 底盤仍在發 odom -> base_footprint —— 中止,不要製造雙父節點"
        exit 1
    fi
    echo "  已確認沒有人發 base_footprint 的 TF"
    cd "$D" && setsid nohup python3 odom_cov_relay.py \
        > /tmp/odom_relay.log 2>&1 < /dev/null &
    sleep 5
    pgrep -f odom_cov_relay.py > /dev/null \
        && echo "  odom_cov_relay OK" \
        || { echo "  odom_cov_relay DEAD"; tail -10 /tmp/odom_relay.log; exit 1; }

    setsid nohup ros2 run robot_localization ekf_node \
        --ros-args --params-file "$D/ekf_multi.yaml" \
        -r __node:=ekf_filter_node \
        > /tmp/ekf.log 2>&1 < /dev/null &
    sleep 8
    pgrep -f ekf_node > /dev/null \
        && echo "  ekf_node OK" \
        || { echo "  ekf_node DEAD"; tail -12 /tmp/ekf.log; exit 1; }
    ;;
chassis)
    echo "--- 里程計:底盤輪速(樹莓派發 odom -> base_footprint -> base_link)---"
    echo "    我們只在它下面接 base_link -> box_link"
    if timeout 6 ros2 topic hz /odom 2>/dev/null | head -1 | grep -q rate; then
        echo "  /odom 有資料"
    else
        echo "  ⚠ /odom 沒資料 —— 樹莓派沒開的話 base_link 不存在,"
        echo "    整棵樹會斷在這裡。要單機跑請用:robot_tf.sh standalone"
    fi
    ;;
standalone)
    echo "--- 里程計:FAST-LIO(自己補 lidar_odom -> box_link)---"
    cd "$D" && setsid nohup python3 fastlio_base_tf.py \
        > /tmp/fastlio_base_tf.log 2>&1 < /dev/null &
    sleep 5
    pgrep -f fastlio_base_tf.py > /dev/null \
        && echo "  fastlio_base_tf OK" \
        || { echo "  DEAD"; tail -10 /tmp/fastlio_base_tf.log; exit 1; }
    ;;
*)
    echo "  未知模式,用 chassis 或 standalone"
    exit 1
    ;;
esac

# box_link 一律掛在底盤的 base_link 底下 —— 感測器就是鎖在車上,這件事
# 跟用哪種里程計無關。變的是**再往上**那一段:
#   fused      multi_odom -> base_footprint   由 ekf_node 動態發
#   chassis    odom       -> base_footprint   由樹莓派動態發(publish_tf: true)
#   standalone lidar_odom -> box_link         由 fastlio_base_tf.py 動態發
#
# ★ standalone 是唯一還把 box_link 當根的模式(底盤整台沒開,base_link
#   根本不存在),所以它不發這一段靜態。
if [ "$MODE" != "standalone" ]; then
    echo "--- box_link 掛在底盤的 base_link 底下 ---"
    pub base_link box_link "$BOX_X" "$BOX_Y" "$BOX_Z" 0 0 "$BOX_YAW"
else
    echo "--- standalone:box_link 是樹根,父節點由 fastlio_base_tf.py 發 ---"
fi

echo "--- 感測器安裝(三種模式完全相同,已校準的剛性關係)---"
pub box_link  body        "$BODY_X" "$BODY_Y" "$BODY_Z" "$BODY_ROLL" "$BODY_PITCH" "$BODY_YAW"
pub box_link  camera_link "$CAM_X"  "$CAM_Y"  "$CAM_Z"  "$CAM_ROLL"  "$CAM_PITCH"  "$CAM_YAW"

# ★ 曾經在這裡發過 box_link -> wiring_box(盒子外殼的中心),後來拿掉了。
#   那是為了讓 RViz 畫得出盒子 —— RobotModel 是逐 link 查 TF 的,URDF 裡
#   有的 link 沒有對應 frame 就畫不出來。
#   但正確做法是在 URDF 的 <visual> 裡給 origin 偏移,不需要多一個 link:
#       <link name="box_link">
#         <visual><origin xyz="0 0 0.080"/><geometry><box .../></geometry></visual>
#       </link>
#   多開 link 的代價是多一段靜態 TF、多一個會查不到的 frame。
sleep 4

# ── 讓靜態變換定期重發 ────────────────────────────────────────────
# /tf_static 每個發布者只發一次。同機的 DDS 訂閱者靠 TRANSIENT_LOCAL 補得到,
# 但隔著 zenoh bridge 就不行 —— bridge 沒有保留歷史訊息的能力,只轉發
# 「它連上之後」才發的東西。實測 WSL 的 RViz 完全收不到任何靜態變換,
# 車體、光達、相機全部畫不出來,而 Jetson 上查得好好的。
#
# 這支每 2 秒把整組重發一次,之後不管誰什麼時候連上都補得到。
# 重發同樣的值不會製造雙父節點 —— tf2 的靜態緩衝以 child 為鍵,覆寫成一樣的。
# ── 轉發底盤的輪子 TF ─────────────────────────────────────────────
# zenoh bridge 按「探索到的節點」建路由,而它探索不到底盤的
# robot_state_publisher —— 所以輪子的 TF 過不了橋,WSL 的 RViz 少四個
# frame,車看起來散掉(RobotModel 是逐 link 查 TF 的)。
#
# ★ 2026-08-12 排除過的假設,都不是原因:
#     「bridge 啟動太早」 在確認 ros2 node list 看得到底盤之後才重啟,還是 0 個
#     「initialPeersList 蓋掉 multicast」 拿掉 fastdds_peers.xml,還是 0 個
#     「WiFi 丟包」 ping 0% 遺失、資料流 20 秒 0 次斷點
#   而 ros2 node list **看得到**那兩個節點,bridge 就是看不到。
#   根因沒釘死,但復現率 100%,所以繞過。
#
# 這支在 Jetson 本機把輪子的變換再發一次,bridge 對本機節點的探索是穩的。
echo "--- 轉發底盤的輪子 TF(bridge 探索不到底盤的 rsp)---"
cd "$D" && setsid nohup python3 tf_relay_wheels.py 10     > /tmp/tf_relay_wheels.log 2>&1 < /dev/null &
sleep 4
pgrep -f tf_relay_wheels.py > /dev/null     && echo "  tf_relay_wheels OK"     || { echo "  ⚠ tf_relay_wheels DEAD"; tail -6 /tmp/tf_relay_wheels.log; }
echo "--- /tf_static 定期重發(給 zenoh bridge 和晚到的訂閱者)---"
cd "$D" && setsid nohup python3 tf_static_repeat.py 2.0     > /tmp/tf_static_repeat.log 2>&1 < /dev/null &
sleep 4
pgrep -f tf_static_repeat.py > /dev/null     && echo "  tf_static_repeat OK"     || { echo "  ⚠ tf_static_repeat DEAD"; tail -6 /tmp/tf_static_repeat.log; }

echo
echo "=== 驗證 ==="
python3 - <<'PY'
import math, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

def q2R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

rclpy.init(); n = Node("tree_verify"); buf = Buffer(); TransformListener(buf, n)
child = {}; acc = []
def cb(m):
    for t in m.transforms:
        child.setdefault(t.child_frame_id, set()).add(t.header.frame_id)
q = QoSProfile(depth=200, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               history=HistoryPolicy.KEEP_LAST)
n.create_subscription(TFMessage, "/tf_static", cb, q)
n.create_subscription(TFMessage, "/tf", cb, 50)
n.create_subscription(Imu, "/livox/imu", lambda m: acc.append(
    [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]),
    qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.1)

dup = {c: sorted(p) for c, p in child.items() if len(p) > 1}
roots = sorted(set(p for ps in child.values() for p in ps) - set(child))
print("  雙父節點: %s" % (dup if dup else "沒有"))
print("  樹根: %s%s" % (roots, "   <-- 超過一個代表樹是斷的" if len(roots) > 1 else ""))
print("  樹:")
for c, ps in sorted(child.items()):
    print("    %-26s <- %s" % (c, ", ".join(sorted(ps))))
print()
for a, b in [("multi_odom", "base_footprint"), ("base_link", "box_link"),
             ("box_link", "body"), ("map", "base_footprint"),
             ("base_footprint", "body")]:
    try:
        t = buf.lookup_transform(a, b, rclpy.time.Time())
        v = t.transform.translation
        print("  %-11s -> %-11s (%+.4f, %+.4f, %+.4f)" % (a, b, v.x, v.y, v.z))
    except Exception as e:
        print("  %-11s -> %-11s FAIL %s" % (a, b, str(e)[:48]))

# 重力驗證:base_link 依定義是水平的(車在平地上),所以把現場重力
# 透過 base_link -> body 轉回去,應該落在垂直方向 1~2 度以內。
if len(acc) > 50:
    up = np.mean(acc, axis=0); up /= np.linalg.norm(up)
    try:
        t = buf.lookup_transform("box_link", "body", rclpy.time.Time())
        qq = t.transform.rotation
        u = q2R(qq.x, qq.y, qq.z, qq.w) @ up
        d = math.degrees(math.acos(min(1, max(-1, u[2]))))
        print("  重力在 box_link 裡離垂直 %.2f 度  %s"
              % (d, "OK" if d < 3 else "還是斜的 —— 檢查 BODY_PITCH"))
    except Exception as e:
        print("  重力檢查失敗:", str(e)[:60])
PY
