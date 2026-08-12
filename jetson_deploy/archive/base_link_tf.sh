#!/usr/bin/env bash
# 座標鏈的兩個靜態變換。原本這兩個都是 identity 佔位符,現在是量出來的值。
#
#   odom --(OD_*)-- camera_init --(FAST-LIO)-- body --(BL_*)-- base_link
#
# OD_*  把 FAST-LIO 的世界座標系扶正(它的原點是開機那一刻的光達姿態,
#       光達斜的話整個 odom 平面就斜,slam_toolbox 的二維投影會有尺度誤差)
# BL_*  把傾斜的光達座標系轉成水平的車體座標系,原點落在地面上
#
# 數值來源:python3 ~/calib_base_link.py(IMU 重力 + 地面平面擬合)
# 不要用 CAD 的標稱角度手填 —— 機構公差、輪胎、地面都會讓實際值不一樣。
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# ============ 改這裡 ============
# 2026-08-10 校準。來源不是 calib_base_link.py —— 車子停的地方太窄太雜,
# 光達看不到成片地板,地面擬合會抓到雜訊(實測吐過一個看似合理的假值)。
# 改成兩段各自用最可靠的來源:
#
#   BL_*  由 qbot_sensors.xacro 的 LIDAR_* 反推(base_link -> body 取逆)。
#         那組角度是導航箱單獨正放在開闊地板上量的,15596 個地面內點、
#         IMU 與地面法線互相驗證差 0.94 度。**車體固定,不受現場地面坡度影響**,
#         這正是要的 —— base_link 應該綁在車身上,不是綁在當下的重力方向。
#         驗證:把現在的重力用這組角度轉進 base_link,離垂直只有 1.48 度
#         (那 1.48 度是現場地板坡度 + 上蓋平行度,不是誤差)。
#
#   OD_*  現場用 IMU + camera_init->body 算的。這一段**本來就該對齊重力** ——
#         實測 camera_init 離重力 31.28 度,二維投影尺度誤差 14.5%,
#         也就是地上走 1 m 在地圖上只記 0.855 m。這是導航壞掉的主因。
#
# base_link 的位置:輪軸上,離地 0.2032(跟原廠 chassis_DD-M.xacro 一致)。
# 不是地面上 —— 舊版 calib_base_link.py 放地面,跟 URDF 差 0.2032,已棄用。
# 重算:python3 ~/calc_tf.sh
BL_X=0.2019
BL_Y=-0.0045
BL_Z=-0.4046
BL_ROLL=-0.0129
BL_PITCH=-0.5181
BL_YAW=0.0064

OD_X=0.0000
OD_Y=0.0000
OD_Z=0.6267
OD_ROLL=-0.0131
OD_PITCH=0.5458
OD_YAW=-0.0000
# ================================================================

# 舊的發布者一定要先殺掉。TF 的單一父節點規則:同一個 child 被兩個 publisher
# 用不同的值發,tf2 不會報錯,它會**交替採用兩者**,結果是整棵樹以 10Hz 抖動,
# 而且症狀看起來像是里程計在飄,很難查。
# ★ 用 child-frame-id 比對,不要用 "static_transform_publisher.*base_link" ——
#   相機那條的命令列是 "--frame-id base_link --child-frame-id camera_link",
#   也含有 base_link,會被一起殺掉(2026-08-10 踩到:跑完這支之後
#   odom -> camera_color_optical_frame 就斷了,但畫面上完全看不出來)。
#   TF 的單一父節點規則:同一個 child 被兩個 publisher 用不同的值發,
#   tf2 不會報錯,它會**交替採用兩者**,整棵樹以 10Hz 抖動。所以只殺同名 child。
pkill -f "child-frame-id camera_init" 2>/dev/null
pkill -f "child-frame-id base_link"   2>/dev/null
sleep 1

setsid nohup ros2 run tf2_ros static_transform_publisher \
  --x "$OD_X" --y "$OD_Y" --z "$OD_Z" \
  --roll "$OD_ROLL" --pitch "$OD_PITCH" --yaw "$OD_YAW" \
  --frame-id odom --child-frame-id camera_init \
  > /tmp/tf_odom.log 2>&1 < /dev/null &

setsid nohup ros2 run tf2_ros static_transform_publisher \
  --x "$BL_X" --y "$BL_Y" --z "$BL_Z" \
  --roll "$BL_ROLL" --pitch "$BL_PITCH" --yaw "$BL_YAW" \
  --frame-id body --child-frame-id base_link \
  > /tmp/tf_baselink.log 2>&1 < /dev/null &

sleep 3
echo "  odom -> camera_init : ($OD_X, $OD_Y, $OD_Z) rpy($OD_ROLL, $OD_PITCH, $OD_YAW)"
echo "  body -> base_link   : ($BL_X, $BL_Y, $BL_Z) rpy($BL_ROLL, $BL_PITCH, $BL_YAW)"

echo
echo "=== 驗證:base_link 應該是水平的、原點在地面 ==="
python3 - <<'PY'
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

def quat_to_R(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

rclpy.init()
n = Node("baselink_verify")
buf = Buffer(); TransformListener(buf, n)
acc = []
n.create_subscription(Imu, "/livox/imu",
                      lambda m: acc.append([m.linear_acceleration.x,
                                            m.linear_acceleration.y,
                                            m.linear_acceleration.z]),
                      qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.1)

if len(acc) < 50:
    print("  IMU 沒資料,跳過水平度檢查")
else:
    up_body = np.mean(acc, axis=0); up_body /= np.linalg.norm(up_body)
    try:
        tf = buf.lookup_transform("base_link", "body", rclpy.time.Time())
        q = tf.transform.rotation
        up_bl = quat_to_R(q.x, q.y, q.z, q.w) @ up_body
        d = math.degrees(math.acos(min(1.0, max(-1.0, up_bl[2]))))
        print("  重力在 base_link 裡 = (%+.4f, %+.4f, %+.4f)" % tuple(up_bl))
        # ★ base_link 是**綁在車體上**的,不是綁重力的(BL_* 來自 URDF 量測,
        #   不是現場 IMU)。所以車停在斜地上時,這裡本來就會顯示地面坡度 ——
        #   那是正確行為,不是誤差。真正必須接近 0 的是下面的 odom 那一項。
        #   門檻放寬到 3 度,只有大到像裝歪了才提醒。
        print("  離垂直 %.2f 度  %s" % (
            d, "OK(這是現場地面坡度,base_link 綁車體不綁重力)" if d < 3.0
            else "<-- 偏大,確認箱子有沒有鎖歪、或車停在斜坡上"))
    except Exception as e:
        print("  查不到 base_link -> body:", str(e)[:70])

for a, b in [("odom", "base_link"), ("map", "base_link")]:
    try:
        t = buf.lookup_transform(a, b, rclpy.time.Time())
        v = t.transform.translation
        ok = "OK" if abs(v.z - 0.2032) < 0.05 else "<-- 對不上"
        print("  %-5s -> %-10s (%.3f, %.3f, %.3f)   z 應該是 0.2032(輪軸)  %s"
              % (a, b, v.x, v.y, v.z, ok))
    except Exception:
        print("  %-5s -> %-10s 查不到" % (a, b))

# odom 這一段扶正了沒有 —— 這才是二維地圖尺度誤差的來源
# (up_body 只在上面 IMU 有資料的分支裡定義,所以這裡要再擋一次)
try:
    up_body
    tf = buf.lookup_transform("odom", "body", rclpy.time.Time())
    q = tf.transform.rotation
    up_od = quat_to_R(q.x, q.y, q.z, q.w) @ up_body
    d = math.degrees(math.acos(min(1.0, max(-1.0, up_od[2]))))
    err = (1 - math.cos(math.radians(d))) * 100
    print("  重力在 odom 裡 = (%+.4f, %+.4f, %+.4f)" % tuple(up_od))
    print("  odom 平面離水平 %.2f 度 -> 二維尺度誤差 %.2f%%  %s"
          % (d, err, "OK" if d < 2.0 else "<-- odom 還是斜的"))
except Exception as e:
    print("  查不到 odom -> body:", str(e)[:70])
PY
