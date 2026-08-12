#!/usr/bin/env bash
# 共用的 ROS 2 環境 —— 其他腳本 source 這一支就好
#
# 2026-08-04 起改走 zenoh bridge,不再讓 WSL 的 DDS 直接跟 Jetson 對話。
#
# 為什麼要換:實測跨機 DDS 會「先通後斷」—— 剛重啟 Jetson 時 /cloud_registered
# 收得到 10Hz,幾分鐘後衰減到 0。用 tcpdump 對照確認封包其實**有**進到 WSL
# (15 秒 6648 個),但 Fast DDS 就是不把資料交付給訂閱者。
# 已排除的原因:防火牆(封包有到)、/dev/shm 耗盡(只用 2%)。
#
# 現在的架構:
#   Jetson  (ROS_DOMAIN_ID=0)  zenoh-bridge-ros2dds -l tcp/0.0.0.0:7447
#              |
#              |  單一條 TCP 連線,WSL 主動撥出 -> 不需要任何入站防火牆規則
#              v
#   WSL     (ROS_DOMAIN_ID=1)  zenoh-bridge-ros2dds -e tcp/192.168.40.98:7447
#              |
#              +-- RViz2 也在 domain 1,只跟本機的 bridge 說話

source /opt/ros/humble/setup.bash

# 刻意跟 Jetson(domain 0)分開。
# 這樣 WSL 的 DDS 完全不會去探索 Jetson,那條會衰減的 UDP 路徑就徹底消失。
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 只放大 socket 緩衝,不再寫 initialPeersList(見 fastdds_local.xml 的說明)
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/lidar_view/fastdds_local.xml"

# Jetson 上 zenoh bridge 的位址
export ZENOH_JETSON="tcp/192.168.40.98:7447"

# ★ 2026-08-11 打開。WSLg 的硬體 GL 直通(Mesa 的 d3d12 驅動)會**間歇性
#   整個失效** —— 同樣的指令有時給 GL 4.1,有時連 1.5 都拿不到:
#
#     RenderingAPIException: OpenGL 1.5 is not supported
#         in GLRenderSystem::initialiseContext
#
#   接著 RViz 用同一個視窗名重試 100 次,log 就被
#   "Window with name 'OgreWindow(0)' already exists" 洗版,
#   把真正的第一則錯誤蓋掉 —— 花了很久才看到它。
#
#   實測對照:
#       硬體  OpenGl version: 4.1 (GLSL 4.1)   不穩,有時完全拿不到
#       軟體  OpenGl version: 4.5 (GLSL 4.5)   永遠可用,版本還更高
#
#   llvmpipe 對這個用途綽綽有餘 —— 二維地圖加幾千個雷射點,GPU 本來就閒著。
#   點雲開到幾十萬點才會感覺到差別,而那個我們在 bridge 端限流到 1 Hz 了。
#
#   哪天 WSLg 修好想換回硬體,把下面這行註解掉即可。
export LIBGL_ALWAYS_SOFTWARE=1

export QT_QPA_PLATFORM=xcb

# WSL 建出來的 /run/user/$UID 是 0755,Qt 會抱怨權限太鬆。
# 純屬雜訊但每次都印,直接修掉。
[ -d "/run/user/$(id -u)" ] && chmod 700 "/run/user/$(id -u)" 2>/dev/null || true

# ── package:// 的解析路徑 ────────────────────────────────────────────
# RViz 的 RobotModel 要把 <mesh filename="package://chassis_description/...">
# 解成實際檔案路徑,靠的是 ament index。少了這行,URDF 收到了也畫不出車 ——
# 而且 RViz 只在 display 的 Status 裡小聲抱怨,主視窗就只是空的。
#
# ~/qbot_ws/install/chassis_description 是 setup_model.sh 建的「假 install」:
# 純資料包不需要 colcon,複製檔案 + 在 ament index 註冊一筆就夠了。
if [ -d "$HOME/qbot_ws/install/chassis_description" ]; then
    export AMENT_PREFIX_PATH="$HOME/qbot_ws/install/chassis_description:$AMENT_PREFIX_PATH"
fi
