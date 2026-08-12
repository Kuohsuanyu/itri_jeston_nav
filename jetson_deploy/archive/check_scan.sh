#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0
echo "######## /scan 品質 ########"
python3 - <<'PY'
import time, math, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n=Node("sc"); S=[]
n.create_subscription(LaserScan,"/scan",lambda m:S.append(m),qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<8 and len(S)<20: rclpy.spin_once(n,timeout_sec=0.05)
if not S: print("  沒有 /scan"); raise SystemExit
m=S[-1]
r=np.array(m.ranges)
fin=np.isfinite(r)
print("  frame_id=%s  射線 %d  range_min=%.2f range_max=%.1f"%(m.header.frame_id,len(r),m.range_min,m.range_max))
print("  有回波 %d 條 (%.1f%%),inf %d 條"%(fin.sum(),100*fin.mean(),(~fin).sum()))
if fin.sum()==0: raise SystemExit
v=r[fin]
print("  距離 min %.3f  1%% %.3f  中位 %.3f  99%% %.3f  max %.3f m"%(
      v.min(),np.percentile(v,1),np.median(v),np.percentile(v,99),v.max()))
print()
print("  近距離分布(車體半對角線 0.65 m,range_min 0.70 —— 0.70~0.90 若有一大圈就是掃到自己):")
h,e=np.histogram(v[v<2.0],bins=13,range=(0.6,1.9))
for i,c in enumerate(h):
    print("    %.2f ~ %.2f m : %4d %s"%(e[i],e[i+1],c,"#"*min(40,c//2)))
print()
print("  多幀穩定度(靜止時同一角度的距離該幾乎不變):")
if len(S)>=10:
    A=np.array([np.array(s.ranges) for s in S[-10:]])
    ok=np.isfinite(A).all(0)
    if ok.sum()>50:
        sd=A[:,ok].std(0)
        print("    %d 條穩定射線,標準差 中位 %.4f m  90%% %.4f m"%(ok.sum(),np.median(sd),np.percentile(sd,90)))
PY
