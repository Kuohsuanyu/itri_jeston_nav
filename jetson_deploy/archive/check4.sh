#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0
echo "########## 濾掉地面/掠射點後再比 ##########"
python3 - <<'PY'
import time, math, rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo, Imu
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

rclpy.init(); n=Node("cmp4"); buf=Buffer(); TransformListener(buf,n)
S={}; acc=[]
def mk(k):
    def cb(m): S[k]=m
    return cb
n.create_subscription(PointCloud2,"/cloud_registered_body",mk("cloud"),qos_profile_sensor_data)
n.create_subscription(Image,"/camera/camera/aligned_depth_to_color/image_raw",mk("depth"),qos_profile_sensor_data)
n.create_subscription(CameraInfo,"/camera/camera/color/camera_info",mk("info"),qos_profile_sensor_data)
n.create_subscription(Imu,"/livox/imu",lambda m: acc.append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]),qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<10 and (len(S)<3 or len(acc)<300): rclpy.spin_once(n,timeout_sec=0.05)
if len(S)<3: print("  缺 topic"); raise SystemExit

up=np.mean(acc,0); up/=np.linalg.norm(up)          # body 座標系的「上」
info=S["info"]; K=info.k; fx,fy,cx,cy=K[0],K[4],K[2],K[5]
dm=S["depth"]; h,w=dm.height,dm.width
D=np.frombuffer(dm.data,dtype=np.uint16).reshape(h,dm.step//2)[:,:w].astype(float)/1000.0

cl=S["cloud"]
P=np.array([(p[0],p[1],p[2]) for p in pc2.read_points(cl,field_names=("x","y","z"),skip_nans=True)])
hgt=P@up                                            # 相對光達的高度(沿重力)
# 地面在光達下方約 0.2 m(calib_box 量到 0.2005)
GROUND=-0.2005
near_ground=np.abs(hgt-GROUND)<0.08

tr=buf.lookup_transform("camera_color_optical_frame",cl.header.frame_id,rclpy.time.Time())
q=tr.transform.rotation; t=tr.transform.translation
x,y,z,ww=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*ww),2*(x*z+y*ww)],
            [2*(x*y+z*ww),1-2*(x*x+z*z),2*(y*z-x*ww)],
            [2*(x*z-y*ww),2*(y*z+x*ww),1-2*(x*x+y*y)]])
Q=(R@P.T).T+np.array([t.x,t.y,t.z])
keep=Q[:,2]>0.3
Q=Q[keep]; ng=near_ground[keep]; hh=hgt[keep]
u=Q[:,0]*fx/Q[:,2]+cx; v=Q[:,1]*fy/Q[:,2]+cy
m=(u>=0)&(u<w)&(v>=0)&(v<h)
Q=Q[m]; ng=ng[m]; hh=hh[m]; ui=u[m].astype(int); vi=v[m].astype(int)
dz=D[vi,ui]
valid=(dz>0.3)&(dz<6.0)&(Q[:,2]<6.0)

def report(tag,sel):
    if sel.sum()<40: print("  %-22s 點太少 (%d)"%(tag,sel.sum())); return
    e=Q[sel,2]-dz[sel]
    print("  %-22s n=%5d  中位 %+.4f  |e|<5cm %5.1f%%  |e|<10cm %5.1f%%"
          %(tag,sel.sum(),np.median(e),100*(np.abs(e)<0.05).mean(),100*(np.abs(e)<0.10).mean()))

print("  光達點總數 %d,投影進畫面且深度有效 %d"%(len(P),valid.sum()))
report("全部",valid)
report("扣掉地面點",valid&~ng)
report("只有地面點",valid&ng)
print()
print("  扣掉地面後,分距離看(角度誤差會隨距離單調變化,平移誤差不會):")
s=valid&~ng
for lo,hi in [(0.5,1.5),(1.5,2.5),(2.5,4.0),(4.0,6.0)]:
    ss=s&(Q[:,2]>=lo)&(Q[:,2]<hi)
    if ss.sum()>30:
        e=Q[ss,2]-dz[ss]
        print("    %.1f~%.1f m  n=%5d  中位 %+.4f m"%(lo,hi,ss.sum(),np.median(e)))
print()
print("  扣掉地面後,分畫面象限:")
for name,mask in [("左半",ui<w/2),("右半",ui>=w/2),("上半",vi<h/2),("下半",vi>=h/2)]:
    report("  "+name,s&mask)
PY
