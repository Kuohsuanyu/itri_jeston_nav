#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

echo "########## 1. TOPIC RATES ##########"
for t in /livox/imu /livox/lidar /cloud_registered /cloud_registered_body /Odometry /scan \
         /camera/camera/color/image_raw /camera/camera/depth/image_rect_raw \
         /camera/camera/aligned_depth_to_color/image_raw ; do
  printf "%-52s " "$t"
  r=$(timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate")
  if [ -z "$r" ]; then echo "NO DATA"; else echo "$r"; fi
done

echo
echo "########## 2. TF FRAMES ##########"
timeout 8 ros2 run tf2_tools view_frames -o /tmp/tfcheck >/dev/null 2>&1
echo "--- frame list ---"
python3 - <<'PY'
import subprocess,re
out = subprocess.run(["cat","/tmp/tfcheck.gv"],capture_output=True,text=True).stdout
print(out[:3000] if out else "(no gv)")
PY

echo
echo "########## 3. TF LOOKUPS ##########"
python3 - <<'PY'
import time, math, rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
rclpy.init()
n = Node("tfchk"); buf = Buffer(); TransformListener(buf, n)
t0=time.time()
while time.time()-t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.2)

def q2e(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    r=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    sp=2*(w*y-z*x); sp=max(-1,min(1,sp)); p=math.asin(sp)
    yw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return [math.degrees(v) for v in (r,p,yw)]

pairs=[("map","odom"),("odom","camera_init"),("camera_init","body"),
       ("body","base_link"),("odom","base_link"),("map","base_link"),
       ("base_link","camera_link"),
       ("base_link","camera_color_optical_frame"),
       ("base_link","camera_depth_optical_frame"),
       ("odom","camera_color_optical_frame")]
for a,b in pairs:
    try:
        t=buf.lookup_transform(a,b,rclpy.time.Time())
        v=t.transform.translation; e=q2e(t.transform.rotation)
        print("  [OK]   %-12s -> %-32s xyz(%7.3f,%7.3f,%7.3f)  rpy(%7.2f,%7.2f,%7.2f) deg"%(a,b,v.x,v.y,v.z,*e))
    except Exception as ex:
        print("  [FAIL] %-12s -> %-32s %s"%(a,b,str(ex)[:70]))
PY

echo
echo "########## 4. GRAVITY IN base_link  (base_link 到底水不水平) ##########"
python3 - <<'PY'
import math, rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
import tf2_ros, time

rclpy.init()
n=Node("gchk"); buf=Buffer(); TransformListener(buf,n)
acc=[]
def cb(m): acc.append((m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z))
n.create_subscription(Imu,"/livox/imu",cb,qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<6 and len(acc)<600:
    rclpy.spin_once(n,timeout_sec=0.05)
if not acc:
    print("  IMU 沒資料"); raise SystemExit
a=np.array(acc); m=a.mean(0); s=a.std(0)
g=m/np.linalg.norm(m)
print("  IMU 樣本 %d 筆  mean=(%.3f,%.3f,%.3f)  std=(%.3f,%.3f,%.3f)"%(len(acc),*m,*s))
print("  body frame 裡的重力單位向量: (%.4f, %.4f, %.4f)"%tuple(g))
print("  body 的 z 軸離垂直: %.2f 度"%math.degrees(math.acos(min(1,abs(g[2])))))
try:
    tr=buf.lookup_transform("base_link","body",rclpy.time.Time())
    q=tr.transform.rotation
    x,y,z,w=q.x,q.y,q.z,q.w
    R=np.array([
      [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
      [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
      [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
    gb=R@g
    print("  base_link 裡的重力單位向量: (%.4f, %.4f, %.4f)"%tuple(gb))
    ang=math.degrees(math.acos(min(1,abs(gb[2]))))
    print("  base_link 的 z 軸離垂直: %.2f 度   %s"%(ang,"OK" if ang<1.5 else "*** 不水平 ***"))
except Exception as e:
    print("  查不到 base_link->body:",str(e)[:80])
PY

echo
echo "########## 5. GROUND PLANE FROM LIDAR (地面在 base_link 的哪裡) ##########"
python3 - <<'PY'
import time, math, rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

rclpy.init()
n=Node("gp"); buf=Buffer(); TransformListener(buf,n)
frames=[]
def cb(m): frames.append(m)
n.create_subscription(PointCloud2,"/cloud_registered_body",cb,qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<8 and len(frames)<12:
    rclpy.spin_once(n,timeout_sec=0.05)
if not frames:
    print("  /cloud_registered_body 沒資料"); raise SystemExit
pts=[]
for m in frames[-8:]:
    for p in pc2.read_points(m, field_names=("x","y","z"), skip_nans=True):
        pts.append((p[0],p[1],p[2]))
P=np.array(pts,dtype=float)
print("  點數 %d  frame_id=%s"%(len(P),frames[-1].header.frame_id))

tr=buf.lookup_transform("base_link",frames[-1].header.frame_id,rclpy.time.Time())
q=tr.transform.rotation; t=tr.transform.translation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
Q=(R@P.T).T + np.array([t.x,t.y,t.z])

d=np.linalg.norm(Q[:,:2],axis=1)
sel=Q[(d>0.8)&(d<6.0)]
print("  水平距離 0.8~6 m 的點: %d"%len(sel))
lo=np.percentile(sel[:,2],2)
cand=sel[sel[:,2]<lo+0.35]
best=None
rng=np.random.default_rng(0)
for _ in range(400):
    i=rng.choice(len(cand),3,replace=False)
    a,b,c=cand[i]
    nv=np.cross(b-a,c-a); nn=np.linalg.norm(nv)
    if nn<1e-6: continue
    nv=nv/nn
    if nv[2]<0: nv=-nv
    dist=np.abs((cand-a)@nv)
    cnt=(dist<0.03).sum()
    if best is None or cnt>best[0]: best=(cnt,nv,a)
cnt,nv,a=best
inl=cand[np.abs((cand-a)@nv)<0.03]
c0=inl.mean(0)
u,s,vt=np.linalg.svd(inl-c0)
nv=vt[2];  nv = -nv if nv[2]<0 else nv
h=-(c0@nv)
tilt=math.degrees(math.acos(min(1,nv[2])))
print("  地面內點 %d  法向量 (%.4f,%.4f,%.4f)"%(len(inl),*nv))
print("  地面法向量離 base_link 的 +Z: %.2f 度  %s"%(tilt,"OK" if tilt<2 else "*** base_link 沒擺平 ***"))
print("  base_link 原點離地: %.4f m  %s"%(h,"(URDF 預期 0.2032)"))
PY

echo
echo "########## 6. LIDAR vs DEPTH CAMERA 一致性 ##########"
python3 - <<'PY'
import time, math, rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

rclpy.init()
n=Node("cmp"); buf=Buffer(); TransformListener(buf,n)
store={}
def mk(k):
    def cb(m): store[k]=m
    return cb
n.create_subscription(PointCloud2,"/cloud_registered_body",mk("cloud"),qos_profile_sensor_data)
n.create_subscription(Image,"/camera/camera/aligned_depth_to_color/image_raw",mk("depth"),qos_profile_sensor_data)
n.create_subscription(CameraInfo,"/camera/camera/color/camera_info",mk("info"),qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<10 and len(store)<3:
    rclpy.spin_once(n,timeout_sec=0.05)
missing=[k for k in ("cloud","depth","info") if k not in store]
if missing:
    print("  缺少:",missing); raise SystemExit

info=store["info"]; K=info.k
fx,fy,cx,cy=K[0],K[4],K[2],K[5]
dm=store["depth"]
import struct
h,w=dm.height,dm.width
enc=dm.encoding
print("  depth encoding=%s %dx%d  color K: fx=%.1f fy=%.1f cx=%.1f cy=%.1f"%(enc,w,h,fx,fy,cx,cy))
D=np.frombuffer(dm.data,dtype=np.uint16).reshape(h,dm.step//2)[:,:w].astype(float)/1000.0

# lidar points -> camera_color_optical_frame
cl=store["cloud"]
tr=buf.lookup_transform("camera_color_optical_frame",cl.header.frame_id,rclpy.time.Time())
q=tr.transform.rotation; t=tr.transform.translation
x,y,z,ww=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*ww),2*(x*z+y*ww)],
            [2*(x*y+z*ww),1-2*(x*x+z*z),2*(y*z-x*ww)],
            [2*(x*z-y*ww),2*(y*z+x*ww),1-2*(x*x+y*y)]])
P=np.array([(p[0],p[1],p[2]) for p in pc2.read_points(cl,field_names=("x","y","z"),skip_nans=True)])
Q=(R@P.T).T+np.array([t.x,t.y,t.z])
Q=Q[Q[:,2]>0.3]
u=(Q[:,0]*fx/Q[:,2]+cx); v=(Q[:,1]*fy/Q[:,2]+cy)
m=(u>=0)&(u<w)&(v>=0)&(v<h)
Q=Q[m]; u=u[m].astype(int); v=v[m].astype(int)
dz=D[v,u]
ok=(dz>0.3)&(dz<6.0)&(Q[:,2]<6.0)
print("  投影進畫面的光達點: %d   其中深度圖也有值的: %d"%(len(Q),ok.sum()))
if ok.sum()<50:
    print("  *** 有效配對太少,無法判斷 ***"); raise SystemExit
err=Q[ok,2]-dz[ok]
print("  距離差 (光達 - 深度相機):")
print("    中位數 %+.4f m   平均 %+.4f m   標準差 %.4f m"%(np.median(err),err.mean(),err.std()))
print("    |err|<5cm 的比例 %.1f%%   |err|<10cm 的比例 %.1f%%"%(
      100*(np.abs(err)<0.05).mean(),100*(np.abs(err)<0.10).mean()))
med=abs(np.median(err))
print("    判定: %s"%("對得上" if med<0.05 and (np.abs(err)<0.10).mean()>0.6 else "*** 外參有偏差 ***"))
PY
echo
echo "########## DONE ##########"
