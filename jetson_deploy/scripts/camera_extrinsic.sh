#!/usr/bin/env bash
# base_link -> camera_link 靜態外參。
#
# ★ 這是唯一需要實體量測的東西,而且 mesh 會不會重影完全取決於它。
#
# 注意量測基準:我們的 base_link **在光達上**,不是車體中心也不是地面
# (base_link 是 FAST-LIO `body` frame 的 static identity 子節點,
#  而 body 是 Mid-360 的 IMU 座標系)。
# 所以要量的是「從光達到相機」。
#
# 座標慣例(ROS 標準):
#   x 往前為正   y 往左為正   z 往上為正
#   相機若往下傾斜,pitch 給正值
#
# 誤差的症狀:
#   位置差 1~2 cm  -> 牆壁變厚、桌腳重影
#   角度差 1~2 度  -> 遠處物體明顯偏移,移動時場景像在漂
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# ============ 改這裡 ============
# 2026-08-07 ICP 校準結果(殘差 12.9cm -> 4.6cm)
#
# ★ pitch 的 -32° 不是誤差,是機構本身:**光達斜 30 度、相機平視**。
#   base_link 就在光達上(FAST-LIO 的 body = Mid-360 的 IMU 座標系),
#   所以在光達座標系裡看,平視的相機自然是傾斜的。
#   ICP 算出 32.33° vs 機構圖標示 30°,差 2.3° —— 機械規格獨立驗證了結果。
#
# Z 是負的也符合 CAD:D435 裝在光達下方。
# 兩次獨立校準的結果:pitch -32.33° / -31.26°,重複性 1.07°,
# 對得上機構圖的 30°。殘差停在 5cm 是 voxel_size 0.08 的取樣下限,
# 不是外參還不準。
X=0.0961
Y=-0.0458
Z=-0.0385
ROLL=-0.0129
PITCH=-0.5456
YAW=0.0248
# ================================

pkill -f "static_transform_publisher.*camera_link" 2>/dev/null
sleep 1

setsid nohup ros2 run tf2_ros static_transform_publisher \
  --x "$X" --y "$Y" --z "$Z" \
  --roll "$ROLL" --pitch "$PITCH" --yaw "$YAW" \
  --frame-id base_link --child-frame-id camera_link \
  > /tmp/cam_tf.log 2>&1 < /dev/null &
sleep 3

echo "已發布 base_link -> camera_link"
echo "  平移 ($X, $Y, $Z)  旋轉 (roll $ROLL, pitch $PITCH, yaw $YAW)"
echo
echo "=== 驗證完整鏈路 ==="
python3 - <<'PY'
import time, rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
rclpy.init()
n = Node("tf_verify"); buf = Buffer(); TransformListener(buf, n)
t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.2)
for a, b in [("odom", "camera_link"),
             ("odom", "camera_depth_optical_frame"),
             ("odom", "camera_color_optical_frame")]:
    try:
        t = buf.lookup_transform(a, b, rclpy.time.Time())
        v = t.transform.translation
        print("  [OK]   %-6s -> %-30s (%.3f, %.3f, %.3f)" % (a, b, v.x, v.y, v.z))
    except Exception as e:
        print("  [FAIL] %-6s -> %-30s %s" % (a, b, str(e)[:60]))
PY
