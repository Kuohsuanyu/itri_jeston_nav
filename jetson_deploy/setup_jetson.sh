#!/usr/bin/env bash
# 從零把 Jetson 建起來 —— 換一台機器、或系統重灌之後跑這支。
#
# ── 為什麼不把上游原始碼整包放進 repo ──────────────────────────────
# FAST_LIO 光是 clone 下來就 357 MB(128 MB 是它的 doc/,其餘大多是 .git),
# 整包 vendor 進版控會讓每次 clone 都很痛,而且上游更新時很難合併。
#
# 我們實際改動的只有**兩個設定檔**:
#     FAST_LIO/config/mid360.yaml
#     livox_ros_driver2/config/MID360_config.json
# 那兩個放在 overlays/,加上這裡記錄的 commit SHA,就足以完整重建。
#
# ★ 釘 commit 不是龜毛。FAST-LIO 的 ROS2 分支會直接改行為,不釘的話
#   下次重建可能拿到不同版本,而症狀會是「參數一樣但跑起來不一樣」。
set -e

# ── 上游版本(2026-08-12 實機在跑的)──────────────────────────────
FASTLIO_URL="https://github.com/hku-mars/FAST_LIO.git"
FASTLIO_BRANCH="ROS2"
FASTLIO_SHA="a4743b095409588842a5b30ddfa27e29d2f99164"      # 2025-01-15

LIVOX_URL="https://github.com/Livox-SDK/livox_ros_driver2.git"
LIVOX_BRANCH="master"
LIVOX_SHA="4a1def929e5b59c7a8122d19fce6efba581ce9f7"        # 2026-07-31

CHASSIS_URL="https://github.com/ITRI-Mechatronics-Intelligent-Decision/chassis-ros2-driver.git"
CHASSIS_BRANCH="main"
CHASSIS_SHA="317b66c38ee79d5be154b451344c954050d213f9"      # 2026-08-11
# ─────────────────────────────────────────────────────────────────

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OVR="$REPO/jetson_deploy/overlays"

say() { echo; echo "=== $* ==="; }

clone_at() {   # clone_at <url> <branch> <sha> <目的地>
    local url=$1 br=$2 sha=$3 dst=$4
    if [ -d "$dst/.git" ]; then
        echo "  已存在:$dst"
        local cur; cur=$(cd "$dst" && git rev-parse HEAD)
        if [ "$cur" != "$sha" ]; then
            echo "  ⚠ commit 不符"
            echo "     現在  $cur"
            echo "     預期  $sha"
            echo "     要對齊:cd $dst && git fetch && git checkout $sha"
        fi
        return 0
    fi
    echo "  clone $url ($br)"
    git clone -q -b "$br" "$url" "$dst"
    (cd "$dst" && git checkout -q "$sha")
    echo "  釘在 $sha"
}

say "1/6 系統套件"
sudo apt-get update -qq
sudo apt-get install -y -qq git git-lfs colcon-common-extensions \
    ros-humble-xacro ros-humble-robot-localization ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan ros-humble-twist-mux \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-rviz2 ros-humble-tf2-tools ros-humble-rqt-tf-tree
git lfs install

say "2/6 Livox SDK2(livox_ros_driver2 的相依)"
if [ ! -d ~/Livox-SDK2 ]; then
    git clone -q https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2
    mkdir -p ~/Livox-SDK2/build && cd ~/Livox-SDK2/build
    cmake .. > /dev/null && make -j"$(nproc)" > /dev/null && sudo make install > /dev/null
    # ★ 一定要 ldconfig。裝完不更新快取的話,livox_ros_driver2 會 build 過
    #   但執行時找不到 liblivox_lidar_sdk_shared.so,錯誤訊息完全不提示原因。
    sudo ldconfig
    echo "  裝好了"
else
    echo "  已存在"
fi

say "3/6 ws_livox"
mkdir -p ~/ws_livox/src
clone_at "$FASTLIO_URL" "$FASTLIO_BRANCH" "$FASTLIO_SHA" ~/ws_livox/src/FAST_LIO
clone_at "$LIVOX_URL"   "$LIVOX_BRANCH"   "$LIVOX_SHA"   ~/ws_livox/src/livox_ros_driver2

say "4/6 套用 overlays(我們改過的設定)"
cp -v "$OVR/FAST_LIO/config/mid360.yaml" ~/ws_livox/src/FAST_LIO/config/
cp -v "$OVR/livox_ros_driver2/config/MID360_config.json" ~/ws_livox/src/livox_ros_driver2/config/
echo "  mid360.yaml   對 Orin Nano 調過:point_filter_num 4 / filter_size 0.5"
echo "                / cube_side 300 / det_range 40。上游預設太重,"
echo "                FAST-LIO 會吃滿一顆核心還跟不上,IMU 佇列積壓到"
echo "                失去同步 -> No Effective Points -> 狀態發散(實測跑到 3000 km 外)"
echo "  MID360_config 光達 192.168.0.50 / 主機 192.168.0.100"

say "5/6 底盤介面"
clone_at "$CHASSIS_URL" "$CHASSIS_BRANCH" "$CHASSIS_SHA" ~/chassis-ros2-driver
mkdir -p ~/chassis_ws/src
[ -e ~/chassis_ws/src/chassis_msgs ] || ln -s ~/chassis-ros2-driver/chassis_msgs ~/chassis_ws/src/chassis_msgs
echo "  用符號連結,不是複製 —— 複製的話上游 git pull 之後 colcon 看不到,"
echo "  2026-08-11 就這樣白 build 過一次(1.3 秒結束,因為什麼都沒變)"

say "6/6 編譯"
source /opt/ros/humble/setup.bash
cd ~/ws_livox && colcon build --symlink-install 2>&1 | tail -4
cd ~/chassis_ws && colcon build --packages-select chassis_msgs 2>&1 | tail -3

say "7/6 執行路徑的符號連結"
for pair in "slam2d:jetson_deploy/scripts" "nav2:jetson_deploy/nav2" \
            "chassis_test:jetson_deploy/chassis_test"; do
    name=${pair%%:*}; sub=${pair##*:}
    if [ -L ~/"$name" ]; then
        echo "  ~/$name 已是連結"
    elif [ -d ~/"$name" ]; then
        mv ~/"$name" ~/"$name.old.$(date +%Y%m%d-%H%M%S)"
        ln -s "$REPO/$sub" ~/"$name"
        echo "  ~/$name 舊的改名保留,已建連結"
    else
        ln -s "$REPO/$sub" ~/"$name"
        echo "  ~/$name 已建連結"
    fi
done

say "完成"
cat <<'MSG'
接下來:
    bash ~/slam2d/startall.sh             光達 + FAST-LIO + 相機
    bash ~/slam2d/start_slam2d.sh         TF + EKF + /scan + slam_toolbox
    bash ~/slam2d/start_zenoh_jetson.sh   給筆電的 RViz 用

底盤(樹莓派)要自己先起 bringup。
MSG
