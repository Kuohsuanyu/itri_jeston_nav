#!/usr/bin/env bash
echo "=== 容器內的 topic 與相機發布 ==="
docker exec nvblox bash -c '
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
timeout 20 ros2 topic list 2>/dev/null | grep -E "camera0|nvblox" | head -20 | sed "s/^/  /"
' 2>/dev/null

echo
echo "=== nvblox 的訂閱(確認接上容器內的相機) ==="
docker exec nvblox bash -c '
source /opt/ros/humble/setup.bash
timeout 20 ros2 node info /nvblox_node 2>/dev/null | sed -n "/Subscribers/,/Publishers/p" | sed "s/^/  /"
' 2>/dev/null

echo
echo "=== 深度整合的時間統計(有 depth/integrate 才代表真的在融合) ==="
docker logs nvblox 2>&1 | grep -iE "depth/integrate|color/integrate|mesh/update|Rates" | tail -12 | sed 's/^/  /'

echo
echo "=== 地圖內容 ==="
docker exec nvblox bash -c '
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
python3 /workspaces/isaac_ros-dev/content.py
' 2>/dev/null

echo
echo "=== 資源 ==="
docker stats nvblox --no-stream --format "  CPU {{.CPUPerc}}  MEM {{.MemUsage}}" 2>/dev/null
cat /proc/loadavg | awk '{print "  loadavg "$1}'
