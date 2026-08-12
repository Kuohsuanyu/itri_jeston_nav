#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0
python3 - <<'PY'
import time, math, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n=Node("dr"); C=[]; A=[]
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
d=P@up; r=np.linalg.norm(P,axis=1)
print("%d 點   重力上 = (%+.4f,%+.4f,%+.4f)"%(len(P),*up))
print("測距 r: min %.2f  中位 %.2f  max %.2f m"%(r.min(),np.median(r),r.max()))
print("高度 d: %.2f%% 在光達下方,%.2f%% 在上方"%(100*(d<0).mean(),100*(d>=0).mean()))
print()
print("RANSAC 找最大的幾個平面(不切片,對整團點雲做):")
print("  #  內點    法線離重力   到 body 原點的距離   類型")
rest=P.copy(); rng=np.random.default_rng(0)
for k in range(8):
    if len(rest)<800: break
    best=None
    for _ in range(1500):
        i=rng.choice(len(rest),3,replace=False)
        a,b,c=rest[i]
        nv=np.cross(b-a,c-a); nn=np.linalg.norm(nv)
        if nn<1e-6: continue
        nv/=nn
        cnt=int((np.abs((rest-a)@nv)<0.03).sum())
        if best is None or cnt>best[0]: best=(cnt,nv,a)
    if best is None or best[0]<600: break
    cnt,nv,a=best
    inl=np.abs((rest-a)@nv)<0.03
    Q=rest[inl]; c0=Q.mean(0)
    _,_,vt=np.linalg.svd(Q-c0, full_matrices=False); nv=vt[2]
    if nv@up<0: nv=-nv
    ang=math.degrees(math.acos(min(1,abs(float(nv@up)))))
    off=float(-(c0@nv))
    if ang<10: kind="水平面  高度 %+.4f m"%off
    elif ang>80: kind="垂直面(牆)"
    else: kind="斜面"
    print("  %d %6d    %6.2f 度      %+.4f m           %s"%(k+1,inl.sum(),ang,off,kind))
    rest=rest[~inl]
print()
print("光達下方 (d<-0.10) 的點,依水平距離分段的高度中位數 —— 地板的話應該是常數:")
hor=np.linalg.norm(P-np.outer(d,up),axis=1)
low=d<-0.10
for lo,hi in [(0.5,1.0),(1.0,1.5),(1.5,2.0),(2.0,3.0),(3.0,4.5)]:
    s=low&(hor>=lo)&(hor<hi)
    if s.sum()>50:
        print("   水平 %.1f~%.1f m  n=%6d  高度 中位 %+.3f  最低 %+.3f"%(lo,hi,s.sum(),np.median(d[s]),d[s].min()))
PY
