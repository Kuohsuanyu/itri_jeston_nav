#!/usr/bin/env bash
# 在 WSL 內佈署檢視環境 —— ROS 2 裝好之後跑這支
set -e

SRC=/mnt/c/Users/ag133/Desktop/工作資料/程式/wheeled-robot-lidar-nav/wsl
DST="$HOME/lidar_view"

echo "### 1/4 複製設定檔到 $DST"
mkdir -p "$DST"
cp "$SRC"/*.xml "$SRC"/*.rviz "$SRC"/*.sh "$DST"/
# 從 Windows 檔案系統複製過來會帶 CRLF,bash 會因為 \r 而報錯
sed -i 's/\r$//' "$DST"/*.sh
chmod +x "$DST"/*.sh
ls -1 "$DST"

echo "### 2/4 調整核心緩衝區(大型 PointCloud2 走 UDP 分片必需)"
sudo tee /etc/sysctl.d/60-ros2-lidar.conf > /dev/null <<'EOF'
# /cloud_registered 每則約 0.36 MB,遠大於單一 UDP datagram 上限,
# 必須分片傳送。預設的 208KB 接收緩衝會讓分片還沒收齊就被丟掉,
# 症狀是 RViz 收到的點雲一閃一閃或整片消失。
net.core.rmem_max = 8388608
net.core.rmem_default = 8388608
net.core.wmem_max = 8388608
net.core.wmem_default = 1048576
# 分片重組的暫存與逾時
net.ipv4.ipfrag_high_thresh = 134217728
net.ipv4.ipfrag_time = 3
EOF
sudo sysctl -q -p /etc/sysctl.d/60-ros2-lidar.conf
echo "  rmem_max = $(cat /proc/sys/net/core/rmem_max)"

echo "### 3/4 寫入 .bashrc"
if ! grep -q "lidar_view/ros_env.sh" "$HOME/.bashrc"; then
    echo '[ -f "$HOME/lidar_view/ros_env.sh" ] && source "$HOME/lidar_view/ros_env.sh"' >> "$HOME/.bashrc"
    echo "  已加入"
else
    echo "  已存在,略過"
fi

echo "### 4/4 驗證"
source "$DST/ros_env.sh"
echo "  ROS_DISTRO=$ROS_DISTRO"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  RMW=$RMW_IMPLEMENTATION"
echo "  profiles=$FASTRTPS_DEFAULT_PROFILES_FILE"
command -v rviz2 > /dev/null && echo "  rviz2 OK" || echo "  ✗ 找不到 rviz2"
echo "### 完成 ###"
