#!/usr/bin/env bash
# Remove the base_link double-parent conflict. ASCII only.
#
# BEFORE (broken -- two static publishers claim base_link as their child):
#     base_footprint -> base_link      Pi's robot_state_publisher (ITRI URDF)
#     body           -> base_link      our base_link_tf.sh
#   tf2 keeps only ONE static entry per child, so whichever publisher last
#   re-sent wins. Two lookups five minutes apart returned x=+0.831 and x=+0.202
#   for the same query.
#
# AFTER (REP-105 standard, one parent each):
#     map -> odom            slam_toolbox
#     odom -> base_footprint chassis wheel odometry
#     base_footprint -> base_link   Pi URDF   (untouched -- we cannot modify the Pi)
#     base_link -> body      our calibration, INVERTED direction
#     base_link -> camera_link  our calibration
#
# Two consequences that must be handled together, or the tree breaks again:
#   1. body would get a second parent from FAST-LIO's own camera_init -> body,
#      so FAST-LIO's /tf is remapped away.
#   2. odom -> camera_init becomes meaningless (camera_init is FAST-LIO's world,
#      odom is the chassis's world -- different origins), so it is removed.
#
# What this costs: FAST-LIO is no longer the odometry source. Wheel odometry
# fills odom -> base_footprint, and slam_toolbox corrects it by scan-matching
# LIDAR against the map. The lidar still drives the map and the global
# correction; the wheels only interpolate between keyframes.
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

echo "=== [1/6] invert base_link -> body from the current publisher args ==="
ARGS=$(pgrep -af "static_transform_publisher .*--child-frame-id base_link" | grep -v "ros2 run" | head -1)
if [ -z "$ARGS" ]; then
    echo "  no body->base_link publisher found; using the calibrated values from qbot_sensors.xacro"
    X=0.2019; Y=-0.0045; Z=-0.4046; R=-0.0129; P=-0.5181; W=0.0064
else
    X=$(echo "$ARGS" | sed -n 's/.*--x \([^ ]*\).*/\1/p')
    Y=$(echo "$ARGS" | sed -n 's/.*--y \([^ ]*\).*/\1/p')
    Z=$(echo "$ARGS" | sed -n 's/.*--z \([^ ]*\).*/\1/p')
    R=$(echo "$ARGS" | sed -n 's/.*--roll \([^ ]*\).*/\1/p')
    P=$(echo "$ARGS" | sed -n 's/.*--pitch \([^ ]*\).*/\1/p')
    W=$(echo "$ARGS" | sed -n 's/.*--yaw \([^ ]*\).*/\1/p')
fi
echo "  body -> base_link : xyz=($X, $Y, $Z) rpy=($R, $P, $W)"

INV=$(python3 - "$X" "$Y" "$Z" "$R" "$P" "$W" <<'PY'
import math, sys
import numpy as np
x,y,z,r,p,w = [float(v) for v in sys.argv[1:7]]
cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(w),math.sin(w)
R = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
              [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
              [-sp,   cp*sr,          cp*cr]])
Ri = R.T
ti = -Ri @ np.array([x,y,z])
sy2 = math.hypot(Ri[0,0], Ri[1,0])
if sy2 > 1e-6:
    rr, pp, ww = math.atan2(Ri[2,1],Ri[2,2]), math.atan2(-Ri[2,0],sy2), math.atan2(Ri[1,0],Ri[0,0])
else:
    rr, pp, ww = math.atan2(-Ri[1,2],Ri[1,1]), math.atan2(-Ri[2,0],sy2), 0.0
print("%.6f %.6f %.6f %.6f %.6f %.6f" % (ti[0],ti[1],ti[2],rr,pp,ww))
PY
)
read IX IY IZ IR IP IW <<< "$INV"
echo "  base_link -> body : xyz=($IX, $IY, $IZ) rpy=($IR, $IP, $IW)"
echo "  (pitch should be about +0.518 -- the lidar tilts DOWN when seen from a level base)"

echo
echo "=== [2/6] stop the conflicting publishers ==="
pkill -f "static_transform_publisher.*--child-frame-id base_link" 2>/dev/null && echo "  killed body->base_link"
pkill -f "static_transform_publisher.*--child-frame-id camera_init" 2>/dev/null && echo "  killed odom->camera_init"
sleep 2

echo
echo "=== [3/6] restart FAST-LIO with its /tf remapped away ==="
# FAST-LIO has no parameter to disable TF publishing. Remapping /tf to an unused
# name is the standard way to silence it without patching the source.
pkill -f fastlio_mapping 2>/dev/null; sleep 3
systemctl --user reset-failed fastlio-run.scope 2>/dev/null
systemd-run --user --scope -p MemoryMax=3G --unit=fastlio-run --quiet \
    ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false \
    --ros-args -r /tf:=/tf_fastlio_unused \
    > /tmp/fastlio.log 2>&1 < /dev/null &
sleep 18
pgrep -f fastlio_mapping > /dev/null && echo "  FAST-LIO up" || { echo "  FAST-LIO DEAD"; tail -15 /tmp/fastlio.log; exit 1; }

echo
echo "=== [4/6] publish the sensor mounts as children of base_link ==="
setsid nohup ros2 run tf2_ros static_transform_publisher \
    --x "$IX" --y "$IY" --z "$IZ" --roll "$IR" --pitch "$IP" --yaw "$IW" \
    --frame-id base_link --child-frame-id body \
    > /tmp/tf_body.log 2>&1 < /dev/null &
sleep 3

echo
echo "=== [5/6] restart the consumers ==="
# Both hold a tf2 buffer that still remembers the old tree, and their
# MessageFilters latch onto whatever /tf_static they saw at startup.
pkill -f pointcloud_to_laserscan_node 2>/dev/null; sleep 2
setsid nohup ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -r cloud_in:=/cloud_registered_body -r scan:=/scan \
    -p target_frame:=base_link -p transform_tolerance:=0.20 \
    -p min_height:=-0.1032 -p max_height:=1.2968 \
    -p angle_min:=-3.14159 -p angle_max:=3.14159 -p angle_increment:=0.0087 \
    -p scan_time:=0.1 -p range_min:=0.35 -p range_max:=40.0 -p use_inf:=true \
    -p queue_size:=30 \
    > /tmp/p2l.log 2>&1 < /dev/null &
sleep 5

pkill -f async_slam_toolbox_node 2>/dev/null; sleep 2
systemctl --user reset-failed slam2d-run.scope 2>/dev/null
systemd-run --user --scope -p MemoryMax=1500M --unit=slam2d-run --quiet \
    ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args --params-file ~/slam2d/slam_params.yaml \
    > /tmp/slam2d.log 2>&1 < /dev/null &
sleep 15

echo
echo "=== [6/6] verify ==="
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

rclpy.init(); n=Node("verify"); buf=Buffer(); TransformListener(buf,n)
child={}; acc=[]
def cb(m):
    for t in m.transforms:
        child.setdefault(t.child_frame_id,set()).add(t.header.frame_id)
q=QoSProfile(depth=200,durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST)
n.create_subscription(TFMessage,"/tf_static",cb,q)
n.create_subscription(TFMessage,"/tf",cb,50)
n.create_subscription(Imu,"/livox/imu",lambda m:acc.append(
    [m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]),
    qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<10: rclpy.spin_once(n,timeout_sec=0.1)

dup={c:p for c,p in child.items() if len(p)>1}
print("  double-parent frames: %s" % (dup if dup else "NONE  <-- fixed"))
for a,b in [("base_link","body"),("map","base_link"),("odom","base_footprint")]:
    try:
        t=buf.lookup_transform(a,b,rclpy.time.Time()); v=t.transform.translation
        print("  %-14s -> %-14s (%+.4f, %+.4f, %+.4f)" % (a,b,v.x,v.y,v.z))
    except Exception as e:
        print("  %-14s -> %-14s FAIL %s" % (a,b,str(e)[:45]))
if len(acc)>50:
    up=np.mean(acc,axis=0); up/=np.linalg.norm(up)
    t=buf.lookup_transform("base_link","body",rclpy.time.Time())
    qq=t.transform.rotation
    u=q2R(qq.x,qq.y,qq.z,qq.w)@up
    d=math.degrees(math.acos(min(1,max(-1,u[2]))))
    print("  gravity in base_link: (%+.4f, %+.4f, %+.4f)  %.2f deg from vertical  %s"
          % (u[0],u[1],u[2],d,"OK" if d<3 else "STILL TILTED"))
PY

echo
echo "=== /scan ==="
timeout 8 ros2 topic hz /scan 2>/dev/null | head -2 | sed 's/^/  /'
echo "  drops so far: $(grep -c dropping /tmp/p2l.log 2>/dev/null || echo 0)"
