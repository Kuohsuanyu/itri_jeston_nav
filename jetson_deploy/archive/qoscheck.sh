#!/usr/bin/env bash
bash ~/enter_isaac.sh '
echo "=== 深度影像 topic 的兩端 QoS ==="
ros2 topic info -v /camera/camera/depth/image_rect_raw 2>/dev/null \
  | grep -E "Node name|Endpoint type|Reliability|Durability|History|QoS" | sed "s/^/  /"

echo
echo "=== nvblox 的 QoS 參數 ==="
ros2 param dump /nvblox_node 2>/dev/null | grep -iE "qos" | sed "s/^/  /"
'
