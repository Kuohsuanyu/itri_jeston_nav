#!/usr/bin/env bash
# Lidar-only TF chain, on frame names the chassis can never collide with.
# ASCII only.
#
# WHY THE RENAME
#   The Pi's bringup publishes  odom -> base_footprint -> base_link -> wheels.
#   We cannot modify the Pi. Any frame we also publish under those names ends up
#   with two parents, and tf2 does not error -- it just keeps whichever arrived
#   last, so lookups flip between two answers at random.
#
#   Solution: give the lidar side its own names. The two trees then never touch,
#   and it no longer matters what the chassis does or whether it is even on.
#
#     chassis (theirs, we never query):
#         odom -> base_footprint -> base_link -> wheels
#
#     lidar (ours):
#         map -> odom_lidar -> camera_init -> body -> base_lidar -> camera_link
#
#   base_lidar sits exactly where base_link would: level, at wheel-axle height.
#   Only the string changed.
#
# WHAT PUBLISHES WHAT
#   map -> odom_lidar        slam_toolbox
#   camera_init -> body      FAST-LIO   (its /tf must NOT be remapped away here)
#   odom_lidar -> camera_init   this script, rotation only, measured from gravity
#   body -> base_lidar          this script, the calibrated mount, inverted
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

# body -> base_lidar : inverse of the calibrated base->body mount.
# Derived from qbot_sensors.xacro (calib_box.py 2026-08-10), pitch 29.69 deg.
BL_X=0.2019
BL_Y=-0.0045
BL_Z=-0.4046
BL_ROLL=-0.0129
BL_PITCH=-0.5181
BL_YAW=0.0064

echo "=== measuring gravity to level odom_lidar ==="
# camera_init is FAST-LIO's world frame = the lidar pose at its startup, which
# is tilted by the mount. Publishing a level odom_lidar above it is what stops
# slam_toolbox projecting the trajectory onto a tilted plane -- that projection
# error is cos(29.7deg) = 13% short on every metre travelled.
LEVEL=$(python3 - <<'PY'
import math, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

acc = []
rclpy.init()
n = Node("grav")
n.create_subscription(Imu, "/livox/imu",
                      lambda m: acc.append([m.linear_acceleration.x,
                                            m.linear_acceleration.y,
                                            m.linear_acceleration.z]),
                      qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 6 and len(acc) < 800:
    rclpy.spin_once(n, timeout_sec=0.05)
if len(acc) < 100:
    print("FAIL no imu")
    raise SystemExit
A = np.asarray(acc, float)
mag = np.linalg.norm(A, axis=1)
jitter = float(mag.std() / max(mag.mean(), 1e-9))
up = A.mean(axis=0); up /= np.linalg.norm(up)

# Build the frame whose z is up, keeping the original x projected onto the
# horizontal plane so the heading does not spin.
z = up
x = np.array([1.0, 0.0, 0.0]); x -= np.dot(x, z) * z; x /= np.linalg.norm(x)
y = np.cross(z, x)
R = np.column_stack([x, y, z])          # v_camera_init = R @ v_level
Ri = R.T                                 # v_level = Ri @ v_camera_init
sy = math.hypot(Ri[0, 0], Ri[1, 0])
if sy > 1e-6:
    r, p, w = math.atan2(Ri[2,1], Ri[2,2]), math.atan2(-Ri[2,0], sy), math.atan2(Ri[1,0], Ri[0,0])
else:
    r, p, w = math.atan2(-Ri[1,2], Ri[1,1]), math.atan2(-Ri[2,0], sy), 0.0
tilt = math.degrees(math.acos(min(1.0, max(-1.0, float(up[2])))))
print("%.6f %.6f %.6f %.3f %.5f" % (r, p, w, tilt, jitter))
PY
)
set -- $LEVEL
if [ "$1" = "FAIL" ]; then
    echo "  no IMU data -- is the livox driver running?"
    exit 1
fi
LR=$1; LP=$2; LW=$3; TILT=$4; JIT=$5
echo "  lidar tilt $TILT deg,  sample jitter $JIT"
if [ "$(echo "$JIT > 0.02" | bc -l 2>/dev/null)" = "1" ]; then
    echo "  WARNING: jitter high -- the vehicle was moving. Re-run while stationary."
fi
echo "  odom_lidar -> camera_init  rpy($LR, $LP, $LW)"

echo
echo "=== publishing ==="
pkill -f "static_transform_publisher.*--child-frame-id camera_init" 2>/dev/null
pkill -f "static_transform_publisher.*--child-frame-id base_lidar"  2>/dev/null
pkill -f "static_transform_publisher.*--child-frame-id base_link"   2>/dev/null
pkill -f "static_transform_publisher.*--child-frame-id body"        2>/dev/null
sleep 2

setsid nohup ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll "$LR" --pitch "$LP" --yaw "$LW" \
  --frame-id odom_lidar --child-frame-id camera_init \
  > /tmp/tf_odom_lidar.log 2>&1 < /dev/null &

setsid nohup ros2 run tf2_ros static_transform_publisher \
  --x "$BL_X" --y "$BL_Y" --z "$BL_Z" \
  --roll "$BL_ROLL" --pitch "$BL_PITCH" --yaw "$BL_YAW" \
  --frame-id body --child-frame-id base_lidar \
  > /tmp/tf_base_lidar.log 2>&1 < /dev/null &

# camera_link moves under base_lidar too, so the overlay tool keeps working.
# NOTE: these are still the 2026-08-07 numbers, taken when the base frame was
# bolted to the tilted lidar. They are wrong against a level base and will make
# the overlay/colouring point at the ceiling. Navigation does not use them.
setsid nohup ros2 run tf2_ros static_transform_publisher \
  --x 0.0892 --y -0.0454 --z 0.3700 \
  --roll 0.0092 --pitch -0.0278 --yaw 0.0154 \
  --frame-id base_lidar --child-frame-id camera_link \
  > /tmp/tf_camera.log 2>&1 < /dev/null &

sleep 4

echo
echo "=== verify ==="
python3 - <<'PY'
import math, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

def q2R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

rclpy.init(); n=Node("v"); buf=Buffer(); TransformListener(buf,n)
child={}; acc=[]
def cb(m):
    for t in m.transforms: child.setdefault(t.child_frame_id,set()).add(t.header.frame_id)
q=QoSProfile(depth=200,durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST)
n.create_subscription(TFMessage,"/tf_static",cb,q)
n.create_subscription(TFMessage,"/tf",cb,50)
n.create_subscription(Imu,"/livox/imu",lambda m:acc.append(
    [m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]),
    qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<10: rclpy.spin_once(n,timeout_sec=0.1)

dup={c:sorted(p) for c,p in child.items() if len(p)>1}
print("  double-parent frames: %s" % (dup if dup else "NONE"))
print("  tree:")
for c,ps in sorted(child.items()):
    print("    %-26s <- %s" % (c, ", ".join(sorted(ps))))
print()
for a,b in [("odom_lidar","base_lidar"),("base_lidar","body")]:
    try:
        t=buf.lookup_transform(a,b,rclpy.time.Time()); v=t.transform.translation
        print("  %-12s -> %-12s (%+.4f, %+.4f, %+.4f)" % (a,b,v.x,v.y,v.z))
    except Exception as e:
        print("  %-12s -> %-12s FAIL %s" % (a,b,str(e)[:50]))
if len(acc)>50:
    up=np.mean(acc,axis=0); up/=np.linalg.norm(up)
    try:
        t=buf.lookup_transform("base_lidar","body",rclpy.time.Time()); qq=t.transform.rotation
        u=q2R(qq.x,qq.y,qq.z,qq.w)@up
        d=math.degrees(math.acos(min(1,max(-1,u[2]))))
        print("  gravity in base_lidar  %.2f deg from vertical  %s"
              % (d,"OK" if d<3 else "STILL TILTED"))
    except Exception as e:
        print("  gravity check failed:", str(e)[:60])
PY
