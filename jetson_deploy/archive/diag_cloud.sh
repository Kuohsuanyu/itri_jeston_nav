#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0
echo "=== 頻率 ==="
for t in /livox/lidar /livox/imu /cloud_registered_body /Odometry; do
  printf "%-26s " "$t"; r=$(timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate"); echo "${r:-NO DATA}"
done
echo
echo "=== FAST-LIO log 尾巴 ==="
tail -12 /tmp/fastlio.log
echo
echo "=== 點雲實際內容 ==="
python3 - <<'PY'
import time, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n=Node("dc"); C=[]; A=[]
n.create_subscription(PointCloud2,"/cloud_registered_body",lambda m:C.append(m),qos_profile_sensor_data)
n.create_subscription(Imu,"/livox/imu",lambda m:A.append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]),qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<8 and len(C)<15: rclpy.spin_once(n,timeout_sec=0.05)
if not C: print("  沒有點雲"); raise SystemExit
print("  收到 %d 幀,最後一幀 %d 點,frame_id=%s"%(len(C),C[-1].width*C[-1].height,C[-1].header.frame_id))
def xyz(m):
    k=m.width*m.height
    raw=np.frombuffer(m.data,dtype=np.uint8)[:k*m.point_step].reshape(k,m.point_step)
    p=raw[:,0:12].copy().view(np.float32).reshape(k,3).astype(float)
    return p[np.isfinite(p).all(1)]
P=np.vstack([xyz(m) for m in C[-15:]])
print("  合併 %d 點"%len(P))
up=np.array(A).mean(0); up/=np.linalg.norm(up)
d=P@up; hor=np.linalg.norm(P-np.outer(d,up),axis=1)
print("  沿重力軸高度 d: min %.2f  1%% %.2f  中位 %.2f  99%% %.2f  max %.2f"%(
      d.min(),np.percentile(d,1),np.median(d),np.percentile(d,99),d.max()))
print("  水平距離 hor:   min %.2f  中位 %.2f  99%% %.2f  max %.2f"%(hor.min(),np.median(hor),np.percentile(hor,99),hor.max()))
print("  d<-0.15 的點: %d    再加 hor<4.0: %d    hor<8.0: %d"%(
      (d<-0.15).sum(), ((d<-0.15)&(hor<4)).sum(), ((d<-0.15)&(hor<8)).sum()))
print()
print("  低處高度分布(d 從 -1.5 到 0):")
h,e=np.histogram(d[(d>-1.5)&(d<0)],bins=30)
for i,c in enumerate(h):
    if c>0: print("    %+.2f ~ %+.2f : %6d %s"%(e[i],e[i+1],c,"#"*min(50,int(50*c/max(h.max(),1)))))
PY
