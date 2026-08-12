#!/usr/bin/env bash
# mesh 檢視器必須跑在容器裡(nvblox_msgs/Mesh 是自訂型別,host 沒有定義)。
# three.js 從 host 的 lidar_web 掛進去 —— 容器內沒有外網,不能用 CDN。
IMG=isaac_ros_dev-aarch64:nvblox-rs

docker rm -f nvblox_mesh_web 2>/dev/null || true

docker run -d --name nvblox_mesh_web \
    --network host --ipc=host --runtime nvidia \
    -e ROS_DOMAIN_ID=0 \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/fastdds_udp_only.xml \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v /mnt/ssd/ws:/workspaces/isaac_ros-dev \
    -v /home/andykuo/lidar_web:/host_lidar_web:ro \
    --workdir /workspaces/isaac_ros-dev \
    --entrypoint /bin/bash \
    "$IMG" -c "
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 /workspaces/isaac_ros-dev/mesh_server.py
" > /dev/null

sleep 15
echo "  容器狀態: $(docker inspect -f '{{.State.Status}}' nvblox_mesh_web 2>/dev/null)"
docker logs nvblox_mesh_web 2>&1 | tail -5 | sed 's/^/  /'
echo "  端點測試:"
curl -sS -o /dev/null -w "    /  HTTP %{http_code}\n" http://127.0.0.1:8093/ 2>/dev/null
curl -sS -o /dev/null -w "    /mesh.bin  HTTP %{http_code}, %{size_download} bytes\n" \
    http://127.0.0.1:8093/mesh.bin 2>/dev/null
