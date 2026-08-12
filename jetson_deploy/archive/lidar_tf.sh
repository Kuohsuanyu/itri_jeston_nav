#!/usr/bin/env bash
# base_link -> body (Mid-360 IMU frame). ASCII only.
#
# Replaces the old base_link_tf.sh, which published the transform the WRONG WAY
# ROUND as  body -> base_link.
#
# WHY THE DIRECTION MATTERS
#   The Pi runs the ITRI bringup including robot_state_publisher, which
#   publishes  base_footprint -> base_link  from the chassis URDF. We cannot
#   modify the Pi, so base_link's parent slot is TAKEN.
#
#   Publishing  body -> base_link  claims base_link as a child a second time.
#   tf2 keeps only one static entry per child, so whichever publisher last
#   re-sent wins -- and which one that is is not deterministic. Two identical
#   lookups five minutes apart returned x=+0.831 and x=+0.202 for the same
#   query. Nothing errors; the numbers just quietly change.
#
#   Publishing  base_link -> body  instead gives every frame exactly one parent:
#
#     map -> odom -> base_footprint -> base_link -+-> body
#                                                 +-> camera_link
#                                                 +-> wheels (Pi URDF)
#
# THIS ONLY WORKS IF FAST-LIO's own /tf is silenced. Otherwise its
# camera_init -> body gives body a second parent and the same bug moves one
# frame down the tree. startall.sh launches FAST-LIO with -r /tf:=/tf_fastlio_unused
# for exactly this reason.
#
# VALUES: chassis_description/urdf/qbot_sensors.xacro (calib_box.py 2026-08-10).
# pitch +0.5181 = 29.69 deg nose-down = the mechanical mount angle.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

X=0.024996
Y=-0.000032
Z=0.451509
ROLL=0.011201
PITCH=0.518136
YAW=-0.000013

# Kill any publisher that claims base_link, body or camera_init as a child.
# The Pi's publisher is unreachable from here, which is fine -- it is the one
# we WANT to keep.
pkill -f "static_transform_publisher.*--child-frame-id base_link"   2>/dev/null
pkill -f "static_transform_publisher.*--child-frame-id camera_init" 2>/dev/null
pkill -f "static_transform_publisher.*--child-frame-id body"        2>/dev/null
sleep 2

setsid nohup ros2 run tf2_ros static_transform_publisher \
  --x "$X" --y "$Y" --z "$Z" --roll "$ROLL" --pitch "$PITCH" --yaw "$YAW" \
  --frame-id base_link --child-frame-id body \
  > /tmp/tf_body.log 2>&1 < /dev/null &
sleep 3
echo "  base_link -> body  ($X, $Y, $Z)  rpy($ROLL, $PITCH, $YAW)"

echo "  verify:"
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
rclpy.init(); n=Node("lidar_tf_verify"); buf=Buffer(); TransformListener(buf,n)
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
while time.time()-t0<9: rclpy.spin_once(n,timeout_sec=0.1)
dup={c:sorted(p) for c,p in child.items() if len(p)>1}
print("    double-parent frames: %s" % (dup if dup else "NONE"))
if len(acc)>50:
    up=np.mean(acc,axis=0); up/=np.linalg.norm(up)
    try:
        t=buf.lookup_transform("base_link","body",rclpy.time.Time()); qq=t.transform.rotation
        u=q2R(qq.x,qq.y,qq.z,qq.w)@up
        d=math.degrees(math.acos(min(1,max(-1,u[2]))))
        print("    gravity in base_link  %.2f deg from vertical  %s"
              % (d,"OK" if d<3 else "STILL TILTED"))
    except Exception as e:
        print("    lookup failed:", str(e)[:60])
PY
