#!/usr/bin/env bash
# 把 chassis_description 搬進 WSL,並展開成 RViz 讀得懂的 URDF。
#
# 為什麼不用 colcon build:
# chassis_description 是純資料包(只有 urdf / meshes,沒有程式碼),
# colcon 對它做的唯一有意義的事就是「把檔案複製到 install/ 並在 ament index
# 註冊一筆」。那兩件事 20 行 shell 就做完了,省掉在 WSL 裝 colcon 和整套
# ament_cmake 的麻煩。
#
# 註冊到 ament index 是必要的,兩個地方會用到:
#   xacro   展開 $(find chassis_description)/urdf/chassis_DD-M.xacro
#   RViz    解析 <mesh filename="package://chassis_description/meshes/...">
# 少了它,xacro 展不開,或是展得開但 RViz 只畫得出感測器、底盤是空的。
set -e

SRC=/mnt/c/Users/ag133/Desktop/工作資料/程式/wheeled-robot-lidar-nav
PKG=chassis_description
WS="$HOME/qbot_ws"
SHARE="$WS/install/$PKG/share/$PKG"

source /opt/ros/humble/setup.bash

echo "### 1/3 複製 $PKG 到 $SHARE"
rm -rf "$WS/install/$PKG"
mkdir -p "$SHARE"
cp -r "$SRC/chassis/chassis-ros2-driver/$PKG"/{urdf,meshes,config,launch,package.xml} "$SHARE"/
# 我們自己那一份(從 robot_tf.sh 產生的)也放進去,include 才找得到
cp "$SRC/wsl/qbot_view.xacro" "$SHARE/urdf/"
find "$SHARE" -name '*.xacro' -o -name '*.urdf' | xargs -r sed -i 's/\r$//'

# ament index:一個空檔案,檔名就是套件名。ros2 / RViz 靠掃這個目錄找套件。
mkdir -p "$WS/install/$PKG/share/ament_index/resource_index/packages"
touch    "$WS/install/$PKG/share/ament_index/resource_index/packages/$PKG"
echo "  meshes: $(ls "$SHARE/meshes/DD-M" | wc -l) 個 STL"

echo "### 2/3 展開 xacro -> URDF"
export AMENT_PREFIX_PATH="$WS/install/$PKG:$AMENT_PREFIX_PATH"
mkdir -p "$HOME/lidar_view"
xacro "$SHARE/urdf/qbot_view.xacro" -o "$HOME/lidar_view/qbot_view.urdf"
echo "  $(grep -c '<link ' "$HOME/lidar_view/qbot_view.urdf") 個 link,"\
"$(grep -c '<joint ' "$HOME/lidar_view/qbot_view.urdf") 個 joint"

echo "### 3/3 檢查 URDF 合法性"
check_urdf "$HOME/lidar_view/qbot_view.urdf" 2>/dev/null \
    || python3 -c "
import sys
try:
    from urdf_parser_py.urdf import URDF
    URDF.from_xml_file('$HOME/lidar_view/qbot_view.urdf')
    print('  URDF 解析 OK')
except ImportError:
    import xml.etree.ElementTree as ET
    ET.parse('$HOME/lidar_view/qbot_view.urdf')
    print('  XML 合法(沒裝 urdf_parser_py,只做到這一層)')
"
echo
echo "### 完成 ###"
echo "URDF:  $HOME/lidar_view/qbot_view.urdf"
echo "套件:  $WS/install/$PKG"
echo
echo "之後每次開新 shell 要讓 RViz 找得到 meshes,需要這一行"
echo "(view_model.sh / view_live.sh 已經內建,手動跑 rviz2 才需要):"
echo "  export AMENT_PREFIX_PATH=$WS/install/$PKG:\$AMENT_PREFIX_PATH"
