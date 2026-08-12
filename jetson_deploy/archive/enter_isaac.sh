#!/usr/bin/env bash
# 進入 Isaac ROS 容器。
#
# 跟官方 run_dev.sh 的差別只有一項:掛入並指定 fastdds_udp_only.xml。
# 沒有它的話,容器裡 `ros2 topic list` 看得到所有 topic,但一則資料都收不到
# —— Fast DDS 的 SHM 傳輸跨不過容器邊界。詳見那個 xml 的註解。
#
# 用法:
#   ./enter_isaac.sh              進入互動 shell
#   ./enter_isaac.sh "指令"       執行單一指令後離開(可背景跑)
# :nvblox 這個 tag 是在 :latest 之上裝了 NITROS/GXF 之後 commit 出來的。
# nvblox_ros 硬性 find_package(isaac_ros_managed_nitros),用 :latest 編不過。
IMG=isaac_ros_dev-aarch64:nvblox
WS=/mnt/ssd/ws
PROFILE=$WS/fastdds_udp_only.xml

if [ ! -f "$PROFILE" ]; then
    echo "找不到 $PROFILE"; exit 1
fi

DOCKER_ARGS=(
    --network host
    --ipc=host
    --privileged
    --runtime nvidia
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=all
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/fastdds_udp_only.xml
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    -v "$WS":/workspaces/isaac_ros-dev
    -v /dev:/dev
    -v /tmp/.X11-unix:/tmp/.X11-unix
    -e DISPLAY
    --workdir /workspaces/isaac_ros-dev
)

if [ $# -eq 0 ]; then
    exec docker run -it --rm "${DOCKER_ARGS[@]}" \
        --name isaac_ros_dev --entrypoint /bin/bash "$IMG"
else
    exec docker run --rm "${DOCKER_ARGS[@]}" \
        --entrypoint /bin/bash "$IMG" -c "source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
$*"
fi
