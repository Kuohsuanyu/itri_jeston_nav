#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0
echo "########## LIDAR vs DEPTH CAMERA ##########"
python3 - <<'PY'
import time, math, rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

rclpy.init(); n=Node("cmp2"); buf=Buffer(); TransformListener(buf,n)
S={}
def mk(k):
    def cb(m): S[k]=m
    return cb
n.create_subscription(PointCloud2,"/cloud_registered_body",mk("cloud"),qos_profile_sensor_data)
n.create_subscription(Image,"/camera/camera/aligned_depth_to_color/image_raw",mk("depth"),qos_profile_sensor_data)
n.create_subscription(CameraInfo,"/camera/camera/color/camera_info",mk("info"),qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<12 and len(S)<3: rclpy.spin_once(n,timeout_sec=0.05)
miss=[k for k in("cloud","depth","info") if k not in S]
if miss: print("  缺:",miss); raise SystemExit

info=S["info"]; K=info.k; fx,fy,cx,cy=K[0],K[4],K[2],K[5]
dm=S["depth"]; h,w=dm.height,dm.width
D=np.frombuffer(dm.data,dtype=np.uint16).reshape(h,dm.step//2)[:,:w].astype(float)/1000.0
print("  depth %dx%d %s   K fx=%.1f fy=%.1f cx=%.1f cy=%.1f"%(w,h,dm.encoding,fx,fy,cx,cy))
print("  深度有效像素比例 %.1f%%"%(100*(D>0.2).mean()))

cl=S["cloud"]
tr=buf.lookup_transform("camera_color_optical_frame",cl.header.frame_id,rclpy.time.Time())
q=tr.transform.rotation; t=tr.transform.translation
x,y,z,ww=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*ww),2*(x*z+y*ww)],
            [2*(x*y+z*ww),1-2*(x*x+z*z),2*(y*z-x*ww)],
            [2*(x*z-y*ww),2*(y*z+x*ww),1-2*(x*x+y*y)]])
P=np.array([(p[0],p[1],p[2]) for p in pc2.read_points(cl,field_names=("x","y","z"),skip_nans=True)])
Q=(R@P.T).T+np.array([t.x,t.y,t.z])
Q=Q[Q[:,2]>0.3]
u=Q[:,0]*fx/Q[:,2]+cx; v=Q[:,1]*fy/Q[:,2]+cy
m=(u>=0)&(u<w)&(v>=0)&(v<h)
Q=Q[m]; ui=u[m].astype(int); vi=v[m].astype(int)
dz=D[vi,ui]
ok=(dz>0.3)&(dz<6.0)&(Q[:,2]<6.0)
print("  投影進畫面的光達點 %d   深度圖同時有值 %d"%(len(Q),ok.sum()))
if ok.sum()<50: print("  有效配對太少"); raise SystemExit
err=Q[ok,2]-dz[ok]
print("  距離差 (光達 - 深度):  中位數 %+.4f m  平均 %+.4f m  std %.4f m"%(np.median(err),err.mean(),err.std()))
print("    |err|<0.05 m: %.1f%%    |err|<0.10 m: %.1f%%"%(100*(np.abs(err)<0.05).mean(),100*(np.abs(err)<0.10).mean()))
# 分區看,判斷是平移誤差還是角度誤差
for lo,hi in [(0.5,1.5),(1.5,2.5),(2.5,4.0),(4.0,6.0)]:
    s=ok&(Q[:,2]>=lo)&(Q[:,2]<hi)
    if s.sum()>30:
        e=Q[s,2]-dz[s]
        print("    %.1f~%.1f m  n=%5d  中位數 %+.4f m"%(lo,hi,s.sum(),np.median(e)))
# 上下左右分區,看有沒有系統性偏斜
for name,mask in [("畫面左半",ui<w/2),("畫面右半",ui>=w/2),("畫面上半",vi<h/2),("畫面下半",vi>=h/2)]:
    s=ok&mask
    if s.sum()>30: print("    %s n=%5d 中位數 %+.4f m"%(name,s.sum(),np.median(Q[s,2]-dz[s])))
med=abs(np.median(err))
print("  判定: %s"%("對得上" if med<0.05 and (np.abs(err)<0.10).mean()>0.6 else "*** 有偏差 ***"))
PY
