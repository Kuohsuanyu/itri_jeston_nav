#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0
python3 - <<'PY'
import time, math, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n=Node("dp"); C=[]; A=[]
n.create_subscription(PointCloud2,"/cloud_registered_body",lambda m:C.append(m),qos_profile_sensor_data)
n.create_subscription(Imu,"/livox/imu",lambda m:A.append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]),qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<12 and (len(C)<25 or len(A)<800): rclpy.spin_once(n,timeout_sec=0.05)
def xyz(m):
    k=m.width*m.height
    raw=np.frombuffer(m.data,dtype=np.uint8)[:k*m.point_step].reshape(k,m.point_step)
    p=raw[:,0:12].copy().view(np.float32).reshape(k,3).astype(float)
    return p[np.isfinite(p).all(1)]
P=np.vstack([xyz(m) for m in C[-25:]])
up=np.array(A).mean(0); up/=np.linalg.norm(up)
d=P@up
hor=np.linalg.norm(P-np.outer(d,up),axis=1)
print("合併 %d 幀 %d 點   重力上 = (%+.4f,%+.4f,%+.4f)"%(len(C[-25:]),len(P),*up))
print()
print("沿重力軸高度分布(相對光達,2 cm 一格,只印佔比 >0.3%% 的格)")
print("  高度區間          點數   水平距離 中位/最大    佔比")
lo,hi=-1.2,0.6
nb=int((hi-lo)/0.02)
h,e=np.histogram(d,bins=nb,range=(lo,hi))
for i,c in enumerate(h):
    if c > len(P)*0.003:
        m=(d>=e[i])&(d<e[i+1])
        print("  %+6.2f ~ %+6.2f  %7d   %5.2f / %5.2f m   %5.2f%%  %s"%(
            e[i],e[i+1],c,np.median(hor[m]),hor[m].max(),100*c/len(P),"#"*min(40,int(300.0*c/len(P)))))
print()
print("水平面偵測:每個高度帶做平面擬合,看它是不是真的水平且夠大")
print("  高度      內點   法線離重力  水平範圍")
for center in np.arange(-1.10, 0.05, 0.02):
    sel=(np.abs(d-center)<0.03)
    if sel.sum()<400: continue
    Q=P[sel]
    c0=Q.mean(0); _,_,vt=np.linalg.svd(Q-c0); nv=vt[2]
    if nv@up<0: nv=-nv
    ang=math.degrees(math.acos(min(1,abs(nv@up))))
    if ang<5.0 and sel.sum()>800:
        print("  %+6.3f  %7d   %5.2f 度    %.2f ~ %.2f m   高度(擬合)= %+.4f"%(
            center,sel.sum(),ang,hor[sel].min(),hor[sel].max(),-(c0@nv)))
PY
