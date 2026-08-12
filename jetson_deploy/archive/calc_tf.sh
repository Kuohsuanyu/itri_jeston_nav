#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0
echo "######## start_slam2d.sh 的高度帶 ########"
grep -nE "min_height|max_height|BL_PITCH|range_max|range_min|angle_|target_frame|transform_tol" ~/slam2d/start_slam2d.sh
echo
echo "######## 算 BL_* / OD_* ########"
python3 - <<'PY'
import math, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

# ── URDF 量到的值(車體固定,不依賴現場地面平不平)──
LIDAR_X, LIDAR_Y, LIDAR_Z = 0.0250, 0.0000, 0.4515
LIDAR_ROLL, LIDAR_PITCH, LIDAR_YAW = 0.0112, 0.5181, 0.0000
AXLE_Z = 0.2032          # base_link 離地(原廠 URDF:輪半徑)

def Rx(a): return np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
def Ry(a): return np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])
def Rz(a): return np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
def quat_to_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def R_to_rpy(R):
    sy=math.sqrt(R[0,0]**2+R[1,0]**2)
    if sy>1e-6: return (math.atan2(R[2,1],R[2,2]),math.atan2(-R[2,0],sy),math.atan2(R[1,0],R[0,0]))
    return (math.atan2(-R[1,2],R[1,1]),math.atan2(-R[2,0],sy),0.0)
def level_basis(up, hint=np.array([1.0,0,0])):
    z=up/np.linalg.norm(up)
    x=hint-np.dot(hint,z)*z
    if np.linalg.norm(x)<1e-6:
        x=np.array([0.0,1,0]); x=x-np.dot(x,z)*z
    x/=np.linalg.norm(x)
    return np.column_stack([x,np.cross(z,x),z])       # v_原 = R @ v_水平

rclpy.init(); n=Node("calc"); buf=Buffer(); TransformListener(buf,n)
acc=[]
n.create_subscription(Imu,"/livox/imu",lambda m:acc.append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]),qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<10 and len(acc)<1200: rclpy.spin_once(n,timeout_sec=0.05)
up_b=np.mean(acc,0); up_b/=np.linalg.norm(up_b)

# ── BL_*:body -> base_link,純粹由 URDF 反推 ──
R_bl_b = Rz(LIDAR_YAW)@Ry(LIDAR_PITCH)@Rx(LIDAR_ROLL)   # v_bl = R @ v_body
t_bl_b = np.array([LIDAR_X,LIDAR_Y,LIDAR_Z])            # body 原點在 base_link 裡
R_b_bl = R_bl_b.T
t_b_bl = -R_b_bl@t_bl_b                                 # base_link 原點在 body 裡
blr,blp,bly = R_to_rpy(R_b_bl)

print("IMU 樣本 %d,body 裡的重力上 = (%+.4f,%+.4f,%+.4f)"%(len(acc),*up_b))
chk = R_bl_b@up_b
print("用 URDF 的角度把重力轉進 base_link = (%+.4f,%+.4f,%+.4f)  離垂直 %.2f 度"
      %(*chk, math.degrees(math.acos(min(1,abs(chk[2]))))))
print("  (這是現場地面 vs 車體上蓋的夾角,不是誤差 —— 地不平就會有)")

# ── OD_*:odom -> camera_init ──
tf=None; t0=time.time()
while time.time()-t0<8:
    rclpy.spin_once(n,timeout_sec=0.1)
    try:
        tf=buf.lookup_transform("camera_init","body",rclpy.time.Time()); break
    except Exception: pass
if tf is None:
    print("查不到 camera_init -> body,OD_* 算不了"); raise SystemExit
q=tf.transform.rotation; v=tf.transform.translation
R_ci_b=quat_to_R(q.x,q.y,q.z,q.w); t_ci_b=np.array([v.x,v.y,v.z])
up_ci=R_ci_b@up_b
tilt=math.degrees(math.acos(min(1,abs(up_ci[2]))))
print()
print("重力在 camera_init 裡 = (%+.4f,%+.4f,%+.4f)   離 camera_init 的 z 軸 %.2f 度"%(*up_ci,tilt))
if tilt<3:
    print("  ==> FAST-LIO 的世界座標系**已經自己對齊重力**,OD_* 的旋轉接近 0")
else:
    print("  ==> camera_init 是斜的,二維投影尺度誤差 %.1f%%,OD_* 必須補這個旋轉"
          %((1-math.cos(math.radians(tilt)))*100))
R_od_ci = level_basis(up_ci).T          # v_odom = R_od_ci @ v_camera_init
odr,odp,ody = R_to_rpy(R_od_ci)

# OD_Z 選成:讓現在的 odom -> base_link 的 z 等於 AXLE_Z
p = R_od_ci@(t_ci_b + R_ci_b@t_b_bl)
OD_Z = AXLE_Z - p[2]
print()
print("="*64)
print("貼進 ~/slam2d/base_link_tf.sh 的參數區")
print("="*64)
print("BL_X=%.4f"%t_b_bl[0]); print("BL_Y=%.4f"%t_b_bl[1]); print("BL_Z=%.4f"%t_b_bl[2])
print("BL_ROLL=%.4f"%blr); print("BL_PITCH=%.4f"%blp); print("BL_YAW=%.4f"%bly)
print()
print("OD_X=0.0000"); print("OD_Y=0.0000"); print("OD_Z=%.4f"%OD_Z)
print("OD_ROLL=%.4f"%odr); print("OD_PITCH=%.4f"%odp); print("OD_YAW=%.4f"%ody)
print("="*64)
print()
print("套用後預期:odom -> base_link 的 z = %.4f(base_link 在輪軸上,離地 %.4f)"%(AXLE_Z,AXLE_Z))
print("/scan 高度帶要改成相對 base_link:min %.4f  max %.4f  (= 離地 0.10 ~ 1.50)"
      %(0.10-AXLE_Z, 1.50-AXLE_Z))
PY
