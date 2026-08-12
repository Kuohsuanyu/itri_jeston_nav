#!/usr/bin/env bash
# 深度串流啟動失敗:是硬體/供電,還是軟體狀態?
# 繞過 ROS 和容器,直接用 librealsense 開串流。
PW=2919

echo "=== 確保沒有其他人佔用相機 ==="
docker rm -f nvblox 2>/dev/null || true
pkill -f realsense2_camera_node 2>/dev/null || true
pkill -9 -f "python3 cam_server.py" 2>/dev/null || true
sleep 5

echo
echo "=== 核心層的 USB 訊息(最近 25 條) ==="
echo "$PW" | sudo -S -p "" dmesg 2>/dev/null | grep -iE "usb|xhci|uvc" | tail -25 | sed 's/^/  /'

echo
echo "=== Jetson 電源模式(供電預算) ==="
if command -v nvpmodel > /dev/null 2>&1; then
    echo "$PW" | sudo -S -p "" nvpmodel -q 2>/dev/null | sed 's/^/  /'
fi
command -v jetson_clocks > /dev/null 2>&1 && echo "  jetson_clocks 可用"

echo
echo "=== 直接用 librealsense 測深度(不經過 ROS/容器) ==="
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH

python3 - <<'PY'
import sys
try:
    import pyrealsense2 as rs
except Exception as e:
    print("  pyrealsense2 不可用:", e)
    sys.exit(0)

for w, h, fps, label in [(424, 240, 6, "最低負載"),
                         (640, 480, 15, "我們用的設定"),
                         (848, 480, 30, "高負載")]:
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    try:
        pipe.start(cfg)
        ok = 0
        for _ in range(10):
            try:
                f = pipe.wait_for_frames(3000)
                if f.get_depth_frame():
                    ok += 1
            except Exception:
                break
        pipe.stop()
        print("  [%s] %dx%d@%d -> 取得 %d/10 幀" % (label, w, h, fps, ok))
    except Exception as e:
        print("  [%s] %dx%d@%d -> 失敗: %s" % (label, w, h, fps, str(e)[:70]))
PY
