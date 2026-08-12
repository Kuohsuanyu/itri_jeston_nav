#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

echo "########## LOGS ##########"
ls -la /tmp/*.log 2>/dev/null
for f in /tmp/cam_extrinsic.log /tmp/cam_tf.log; do
  echo "--- $f ---"; tail -15 "$f" 2>&1
done

echo
echo "########## GROUND PLANE (以重力定義上方,修正版) ##########"
python3 - <<'PY'
import time, math, rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
n=Node("gp2")
cl=[]; im=[]
n.create_subscription(PointCloud2,"/cloud_registered_body",lambda m: cl.append(m),qos_profile_sensor_data)
n.create_subscription(Imu,"/livox/imu",lambda m: im.append((m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z)),qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<8 and (len(cl)<8 or len(im)<400):
    rclpy.spin_once(n,timeout_sec=0.05)
if not cl or not im:
    print("  沒資料"); raise SystemExit
g=np.array(im).mean(0); up=g/np.linalg.norm(g)   # Livox IMU: 靜止時加速度指向「上」
print("  重力軸(body 座標,指向上方): (%.4f,%.4f,%.4f)"%tuple(up))

P=np.array([(p[0],p[1],p[2]) for m in cl[-6:] for p in pc2.read_points(m,field_names=("x","y","z"),skip_nans=True)])
hgt=P@up                       # 沿重力軸的高度(相對 body 原點)
hor=np.linalg.norm(P-np.outer(hgt,up),axis=1)
sel=(hor>0.8)&(hor<6.0)
P2=P[sel]; h2=hgt[sel]
print("  點數 %d  取水平 0.8~6 m 共 %d"%(len(P),len(P2)))
import numpy as _n
hist,edges=_n.histogram(h2,bins=80,range=(-3,3))
print("  沿重力軸的高度分布(相對光達):")
for i,c in enumerate(hist):
    if c> len(h2)*0.01:
        print("    %+.2f ~ %+.2f m : %6d %s"%(edges[i],edges[i+1],c,"#"*int(40*c/hist.max())))
lo=np.percentile(h2,1)
cand=P2[(h2>lo-0.1)&(h2<lo+0.4)]
best=None; rng=np.random.default_rng(0)
for _ in range(600):
    i=rng.choice(len(cand),3,replace=False); a,b,c=cand[i]
    nv=np.cross(b-a,c-a); nn=np.linalg.norm(nv)
    if nn<1e-6: continue
    nv/=nn
    if nv@up<0: nv=-nv
    cnt=(np.abs((cand-a)@nv)<0.03).sum()
    if best is None or cnt>best[0]: best=(cnt,nv,a)
cnt,nv,a=best
inl=cand[np.abs((cand-a)@nv)<0.03]
c0=inl.mean(0); _,_,vt=np.linalg.svd(inl-c0); nv=vt[2]
if nv@up<0: nv=-nv
print("  地面內點 %d  法向量 (%.4f,%.4f,%.4f)"%(len(inl),*nv))
print("  地面法向量 vs 重力軸夾角: %.2f 度  (應該 <2 度)"%math.degrees(math.acos(min(1,nv@up))))
print("  光達(body 原點)離地高度: %.4f m"%(-(c0@nv)))
print("  → URDF 目前寫 LIDAR_Z=0.4646,加上 base_link 離地 0.2032 = %.4f m"%(0.4646+0.2032))
PY
