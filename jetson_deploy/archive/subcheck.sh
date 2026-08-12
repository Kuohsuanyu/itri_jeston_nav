#!/usr/bin/env bash
bash ~/enter_isaac.sh '
echo "=== nvblox 目前的訂閱 ==="
ros2 node info /nvblox_node 2>/dev/null | sed -n "/Subscribers/,/Publishers/p"
echo
echo "=== 相機 topic 的訂閱者數(>=1 才代表 nvblox 接上了) ==="
for t in /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info; do
  echo "--- $t ---"
  ros2 topic info "$t" 2>/dev/null
done
'
