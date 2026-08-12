#!/usr/bin/env bash
# nvblox_ros hard-requires isaac_ros_managed_nitros. Prefer the prebuilt apt
# package inside the container over building NITROS from source -- NITROS
# pulls in GXF and is a big build.
docker run --rm \
    --network host --ipc=host --privileged --runtime nvidia \
    -v /mnt/ssd/ws:/workspaces/isaac_ros-dev -v /dev:/dev \
    --entrypoint /bin/bash isaac_ros_dev-aarch64:latest -c '
echo "=== 容器裡已安裝的 isaac_ros 套件 ==="
ls /opt/ros/humble/share 2>/dev/null | grep -i isaac | sed "s/^/  /" || echo "  無"

echo
echo "=== apt 有哪些 nitros / gxf 套件 ==="
apt-get update -qq 2>/dev/null
apt-cache search "isaac-ros" 2>/dev/null | grep -iE "nitros|gxf" | head -20 | sed "s/^/  /" || echo "  搜尋不到"

echo
echo "=== 關鍵套件的候選版本 ==="
for p in ros-humble-isaac-ros-managed-nitros ros-humble-isaac-ros-nitros ros-humble-isaac-ros-gxf; do
    V=$(apt-cache policy $p 2>/dev/null | sed -n "s/.*Candidate: //p" | head -1)
    echo "  $p -> ${V:-無}"
done

echo
echo "=== nvblox_ros 到底要哪些 nitros 套件 ==="
grep -nE "find_package|nitros" /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/CMakeLists.txt \
  | head -20 | sed "s/^/  /"

echo
echo "=== package.xml 的相依 ==="
grep -E "depend" /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/package.xml \
  | sed "s/^/  /"
'
