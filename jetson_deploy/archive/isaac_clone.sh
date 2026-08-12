#!/usr/bin/env bash
# Clone Isaac ROS release-3.2 onto the NVMe.
#
# ★ The branch matters. The default branch now targets Jetson Thor /
#   JetPack 7 / ROS Jazzy. Orin + JetPack 6.x + Humble needs release-3.2.
#   Getting this wrong only shows up hours into the build.
set -e

WS=/mnt/ssd/ws
BRANCH=release-3.2

echo "=== target ==="
df -h /mnt/ssd | tail -1 | sed 's/^/  /'
mkdir -p "$WS/src"
cd "$WS/src"

clone() {
    name=$1; shift
    if [ -d "$name" ]; then
        echo "  $name 已存在,檢查分支"
        git -C "$name" rev-parse --abbrev-ref HEAD | sed 's/^/    /'
        return
    fi
    echo "  clone $name ($BRANCH)"
    timeout 900 git clone "$@" -b "$BRANCH" \
        "https://github.com/NVIDIA-ISAAC-ROS/$name.git" 2>&1 | tail -3 | sed 's/^/    /'
}

echo
echo "=== clone ==="
clone isaac_ros_common
clone isaac_ros_nvblox --recursive

echo
echo "=== 結果 ==="
for d in isaac_ros_common isaac_ros_nvblox; do
    if [ -d "$d" ]; then
        printf "  %-20s branch=%s  size=%s\n" "$d" \
            "$(git -C $d rev-parse --abbrev-ref HEAD)" \
            "$(du -sh $d | cut -f1)"
    else
        echo "  $d  MISSING"
    fi
done

echo
echo "=== isaac_ros_common 的 docker 腳本 ==="
ls -1 "$WS/src/isaac_ros_common/scripts/" 2>/dev/null | head -10 | sed 's/^/  /'

echo
echo "=== 磁碟 ==="
df -h /mnt/ssd | tail -1 | sed 's/^/  /'
