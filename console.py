#!/usr/bin/env python3
"""
Jetson 光達展示控制台 — 在 Windows 上跑,透過 SSH 控制 Orin Nano。

  一鍵啟動   光達驅動 -> FAST-LIO -> 網頁檢視器,依序帶起來
  即時監看   服務狀態、乙太網路、光達連線、資料流量、CPU/溫度
  一鍵關機   讓 Jetson 正常關機(不是拔電,microSD 才不會壞)

用法:雙擊「啟動控制台.bat」,或 python console.py
"""

import base64
import json
import socket
import subprocess
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# ---------------------------------------------------------------- 設定
HOST = "192.168.40.98"
USER = "andykuo"
PW = "2919"
HOSTKEY = "SHA256:ph8AvnetrS39dH8fnsuW8FPp9tsIg3wGecSXe/egfI4"
PLINK = r"C:\Program Files\PuTTY\plink.exe"

ETH = "enP8p1s0"          # 接光達的網路介面
LIDAR_IP = "192.168.0.50"
LIDAR_HOST_IP = "192.168.0.100"   # Jetson 在光達網段的位址,驅動會 bind 到這個
PANEL_PORT = 7788

WSL_DISTRO = "Ubuntu-22.04"
ROS_POLL_SEC = 8          # ros2 topic list 要幾秒,不能跟主輪詢同頻率

HERE = Path(__file__).resolve().parent

# ---------------------------------------------------------------- SSH


def reachable(timeout=1.5):
    """先做便宜的 TCP 探測,避免 Jetson 關機時每次都卡 SSH 逾時。"""
    try:
        with socket.create_connection((HOST, 22), timeout=timeout):
            return True
    except OSError:
        return False


def ssh(script, timeout=60):
    """在 Jetson 上執行 bash 腳本,回傳 (成功, 輸出)。"""
    b64 = base64.b64encode(script.encode("utf-8")).decode("ascii")
    cmd = [PLINK, "-ssh", "-batch", "-hostkey", HOSTKEY, "-pw", PW,
           f"{USER}@{HOST}", f"echo {b64} | base64 -d | bash"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True,
                           timeout=timeout, encoding="utf-8", errors="replace",
                           creationflags=subprocess.CREATE_NO_WINDOW)
        return True, (r.stdout or "") + (r.stderr or "")
    except subprocess.TimeoutExpired:
        return False, "SSH 逾時"
    except Exception as e:
        return False, f"SSH 失敗:{e}"


# ---------------------------------------------------------------- 遠端腳本

STATUS_SH = f"""
pgrep -f livox_ros_driver2_node >/dev/null && echo livox=1 || echo livox=0
pgrep -f fastlio_mapping        >/dev/null && echo fastlio=1 || echo fastlio=0
pgrep -f "python3 server.py"    >/dev/null && echo viewer=1 || echo viewer=0
pgrep -f zenoh-bridge-ros2dds   >/dev/null && echo zenoh=1 || echo zenoh=0
pgrep -f async_slam_toolbox_node >/dev/null && echo slam=1 || echo slam=0
pgrep -f pointcloud_to_laserscan >/dev/null && echo scan=1 || echo scan=0
pgrep -f "python3 map_server.py" >/dev/null && echo mapweb=1 || echo mapweb=0
pgrep -f realsense2_camera_node  >/dev/null && echo cam=1 || echo cam=0
pgrep -f "python3 cam_server.py" >/dev/null && echo camweb=1 || echo camweb=0

# 注意:STATUS_SH 是 f-string,這裡**不能出現大括號**,否則會被當成插值。
# 所以用 sed 取值而不是 awk。
echo camhz=$(curl -sS --max-time 2 http://127.0.0.1:8092/stats.json 2>/dev/null \
  | sed -n 's/.*"color": *\\([0-9.]*\\).*/\\1/p')
# 上色率是相機那條線唯一有意義的健康指標。點數在跑但這個卡在 0,
# 就是 base_link -> camera_color_optical_frame 的 TF 斷了 ——
# 光看 process 在不在完全看不出來。
echo colorpct=$(curl -sS --max-time 2 http://127.0.0.1:8080/stats.json 2>/dev/null \
  | sed -n 's/.*"colored_pct": *\\([0-9.]*\\).*/\\1/p')
echo ssd=$(df -m /mnt/ssd 2>/dev/null | sed -n '2s/  */ /gp' | cut -d' ' -f4)
echo lio_mb=$(( $(ps -o rss= -p $(pgrep -f fastlio_mapping | head -1) 2>/dev/null || echo 0) / 1024 ))
echo slam_mb=$(( $(ps -o rss= -p $(pgrep -f async_slam_toolbox_node | head -1) 2>/dev/null || echo 0) / 1024 ))
echo eth=$(cat /sys/class/net/{ETH}/carrier 2>/dev/null || echo 0)
echo rx=$(cat /sys/class/net/{ETH}/statistics/rx_bytes 2>/dev/null || echo 0)
ping -c1 -W1 {LIDAR_IP} >/dev/null 2>&1 && echo lidar=1 || echo lidar=0
awk '{{print "load="$1}}' /proc/loadavg
free -m | awk '/Mem:/{{print "mem_used="$3; print "mem_total="$2}}'
echo temp=$(awk '{{printf "%.1f", $1/1000}}' /sys/devices/virtual/thermal/thermal_zone0/temp 2>/dev/null || echo 0)
awk '{{print "uptime="int($1)}}' /proc/uptime
echo pts=$(ls -l ~/ws_livox/src/FAST_LIO/PCD/*.pcd 2>/dev/null | wc -l)
"""

START_SH = """
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

# FAST-LIO 和 slam_toolbox 都要包在 cgroup 上限裡。
# 不包的話,任何一邊記憶體失控都會觸發**全域** OOM killer,
# 實測連 containerd 這種不相干的行程都會被掃到,等於整塊板子陪葬。
start_capped() {
    unit="$1"; cap="$2"; log="$3"; shift 3
    systemctl --user reset-failed "$unit.scope" 2>/dev/null
    if command -v systemd-run > /dev/null 2>&1; then
        systemd-run --user --scope -p MemoryMax="$cap" --unit="$unit" --quiet \
          "$@" > "$log" 2>&1 < /dev/null &
    else
        setsid nohup "$@" > "$log" 2>&1 < /dev/null &
    fi
}

pkill -9 -f "python3 server.py"     2>/dev/null
pkill -9 -f "python3 map_server.py" 2>/dev/null
pkill -f fastlio_mapping            2>/dev/null
pkill -f livox_ros_driver2          2>/dev/null
pkill -f zenoh-bridge-ros2dds       2>/dev/null
pkill -f async_slam_toolbox_node    2>/dev/null
pkill -f pointcloud_to_laserscan    2>/dev/null
pkill -f static_transform_publisher 2>/dev/null
pkill -f realsense2_camera_node     2>/dev/null
pkill -9 -f "python3 cam_server.py" 2>/dev/null
sleep 3

# 先等網卡就緒再啟動驅動。
#
# Livox 的驅動會 bind 到設定檔裡的 host IP(192.168.0.100),而且
# **bind 失敗就永久放棄、不會重試**。開機或剛插上網路線時,啟動腳本常常
# 比 NetworkManager 快一步,結果驅動一啟動就 `bind failed` -> `Init lds lidar fail!`,
# 之後行程還活著、pgrep 看起來是 [OK],但 /livox/lidar 從來沒發布過。
# 這種「服務都在跑但完全沒資料」最難查,所以寧可在這裡等。
echo "[1/6] 等待光達網路就緒"
for i in $(seq 1 30); do
    CARRIER=$(cat /sys/class/net/ETH_IF/carrier 2>/dev/null || echo 0)
    HAVE_IP=$(ip -4 -o addr show ETH_IF 2>/dev/null | grep -c "LIDAR_HOST_IP")
    if [ "$CARRIER" = "1" ] && [ "$HAVE_IP" -ge 1 ]; then
        echo "      網卡就緒(carrier=1, IP=LIDAR_HOST_IP),等了 ${i} 秒"
        break
    fi
    sleep 1
done
if [ "$(cat /sys/class/net/ETH_IF/carrier 2>/dev/null || echo 0)" != "1" ]; then
    echo "      ✗ 30 秒內網卡仍無 carrier —— 網路線沒插好,或光達沒供電"
    echo "        後面的服務會全部空轉,先處理實體連線再重試"
fi
ping -c1 -W2 LIDAR_ADDR >/dev/null 2>&1 \
  && echo "      光達 LIDAR_ADDR 有回應" \
  || echo "      ⚠ ping 不到 LIDAR_ADDR"

echo "[1b/6] 光達驅動"
setsid nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  > /tmp/livox.log 2>&1 < /dev/null &
sleep 10
# bind 失敗是致命的,直接把它挑出來講清楚,不要讓它默默空轉
if grep -q "Init lds lidar fail" /tmp/livox.log 2>/dev/null; then
    echo "      ✗ 驅動 bind 失敗 —— 通常是網卡還沒拿到 IP,或有舊行程殘留"
    echo "        補救:pkill -9 -f livox_ros_driver2 之後重新啟動"
fi

echo "[2/6] FAST-LIO(3GB 上限)"
start_capped fastlio-console 3G /tmp/fastlio.log \
  ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
sleep 12

echo "[3/6] 3D 點雲檢視器 :8080"
cd ~/lidar_web
setsid nohup python3 server.py > /tmp/webviewer.log 2>&1 < /dev/null &
sleep 4

# 2D 建圖:static TF 把 FAST-LIO 的 camera_init/body 接成 odom/base_link,
# 再把 body-frame 點雲轉成 /scan 餵給 slam_toolbox。細節見 ~/slam2d/start_slam2d.sh。
echo "[4/6] 2D 建圖(TF + /scan + slam_toolbox)"
bash ~/slam2d/start_slam2d.sh 2>&1 | sed 's/^/      /'

echo "[5/6] 2D 地圖檢視器 :8090"
cd ~/slam2d
setsid nohup python3 map_server.py > /tmp/mapweb.log 2>&1 < /dev/null &
sleep 4

# zenoh bridge —— WSL 端的 RViz 靠這條 TCP 通道拿資料。
# 必須排在最後,bridge 才探索得到所有 publisher。
echo "[6/8] zenoh bridge"
cd ~
RUST_LOG=info setsid nohup zenoh-bridge-ros2dds \
  -l tcp/0.0.0.0:7447 --no-multicast-scouting \
  --pub-max-frequency "/cloud_registered=3.0" \
  > /tmp/zenoh.log 2>&1 < /dev/null &
sleep 8

# RealSense:只開 depth + color,不開 pointcloud。
# nvblox 直接吃深度影像,先在 CPU 上組點雲是白做工。
echo "[7/8] RealSense D435"
pkill -f realsense2_camera_node 2>/dev/null
sleep 2
setsid nohup ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true enable_color:=true \
  enable_infra1:=false enable_infra2:=false \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15 \
  pointcloud.enable:=false align_depth.enable:=true \
  > /tmp/realsense.log 2>&1 < /dev/null &
sleep 22
pgrep -f realsense2_camera_node >/dev/null && echo "      相機 OK" || echo "      ✗ 相機沒起來"

# base_link -> camera_link 外參(數值暫定,相機位置調整後要重量)
bash ~/slam2d/camera_extrinsic.sh > /tmp/cam_extrinsic.log 2>&1

echo "[8/8] 影像檢視器 :8092"
pkill -9 -f "python3 cam_server.py" 2>/dev/null
sleep 1
cd ~/cam_web
setsid nohup python3 cam_server.py > /tmp/cam_web.log 2>&1 < /dev/null &
sleep 5

echo READY
"""

# START_SH 裡有 ${i} 這類 shell 語法,做成 f-string 會被大括號吃掉,
# 所以改用明確的字串代換。
START_SH = (START_SH
            .replace("ETH_IF", ETH)
            .replace("LIDAR_HOST_IP", LIDAR_HOST_IP)
            .replace("LIDAR_ADDR", LIDAR_IP))

STOP_SH = """
pkill -INT -f fastlio_mapping 2>/dev/null
sleep 4
pkill -9 -f "python3 server.py"     2>/dev/null
pkill -9 -f "python3 map_server.py" 2>/dev/null
pkill -9 -f fastlio_mapping         2>/dev/null
pkill -9 -f livox_ros_driver2       2>/dev/null
pkill -f zenoh-bridge-ros2dds       2>/dev/null
pkill -f async_slam_toolbox_node    2>/dev/null
pkill -f pointcloud_to_laserscan    2>/dev/null
pkill -f static_transform_publisher 2>/dev/null
pkill -f realsense2_camera_node     2>/dev/null
pkill -9 -f "python3 cam_server.py" 2>/dev/null
sync
echo STOPPED
"""

SHUTDOWN_SH = f"""
pkill -INT -f fastlio_mapping 2>/dev/null
sleep 3
pkill -9 -f "python3 server.py"     2>/dev/null
pkill -9 -f "python3 map_server.py" 2>/dev/null
pkill -9 -f livox_ros_driver2       2>/dev/null
pkill -f zenoh-bridge-ros2dds       2>/dev/null
pkill -f async_slam_toolbox_node    2>/dev/null
pkill -f pointcloud_to_laserscan    2>/dev/null
pkill -f static_transform_publisher 2>/dev/null
pkill -f realsense2_camera_node     2>/dev/null
pkill -9 -f "python3 cam_server.py" 2>/dev/null
sync
echo {PW} | sudo -S -p '' systemctl poweroff
"""

# ---------------------------------------------------------------- 狀態

state = {
    "busy": None,        # None / "starting" / "stopping" / "shutdown"
    "log": "",
    "prev_rx": None,     # (時間, rx_bytes) 用來算流量
    "rate": 0.0,
    "ros_topics": [],    # WSL 端看得到的 topic(背景執行緒快取)
    "ros_seen": 0.0,     # 最後一次成功看到 topic 的時間
    "wsl_up": False,
}
lock = threading.Lock()


# ---------------------------------------------------------------- WSL


def wsl(args, timeout=30, ros=True):
    """在 WSL 裡執行指令。

    注意不能用 `bash -lc` 加上寫在 .bashrc 的環境 —— Ubuntu 的 .bashrc
    開頭對非互動 shell 會直接 return,那行永遠不會執行(症狀:ros2: command not found)。
    這裡一律顯式 source ros_env.sh。
    """
    body = args[0] if len(args) == 1 else " ".join(args)
    if ros:
        body = 'source "$HOME/lidar_view/ros_env.sh" >/dev/null 2>&1; ' + body
    cmd = ["wsl", "-d", WSL_DISTRO, "--", "bash", "-c", body]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                           encoding="utf-8", errors="replace",
                           creationflags=subprocess.CREATE_NO_WINDOW)
        return r.returncode == 0, (r.stdout or "") + (r.stderr or "")
    except Exception as e:
        return False, str(e)


def launch_rviz():
    """開一個 cmd 視窗跑 RViz,這樣啟動時的檢查訊息看得到。"""
    subprocess.Popen(
        ["cmd", "/c", "start", "RViz2 - FAST-LIO",
         "wsl", "-d", WSL_DISTRO, "--", "bash", "-c",
         "$HOME/lidar_view/start_rviz.sh"],
        creationflags=subprocess.CREATE_NEW_CONSOLE,
    )


def ros_watcher():
    """背景輪詢 WSL 端看得到哪些 topic —— ros2 topic list 太慢,不能塞進主輪詢。

    WSL 現在跑在 ROS_DOMAIN_ID=1,跟 Jetson(domain 0)刻意分開,
    中間靠 zenoh bridge 橋接。所以輪詢前得先確定 WSL 端的 bridge 有起來,
    否則 topic list 永遠是空的。start_zenoh.sh 本身是 idempotent,重複呼叫無害。
    """
    wsl(["$HOME/lidar_view/start_zenoh.sh"], timeout=40, ros=False)
    while True:
        ok, out = wsl(["ros2 topic list 2>/dev/null"], timeout=25)
        topics = [t.strip() for t in out.splitlines()
                  if t.strip().startswith("/")] if ok else []
        with lock:
            state["wsl_up"] = ok
            state["ros_topics"] = topics
            if topics:
                state["ros_seen"] = time.time()
        time.sleep(ROS_POLL_SEC)


def collect():
    if state["busy"] == "shutdown":
        online = reachable(1.0)
        if not online:
            with lock:
                state["busy"] = None
    if not reachable():
        with lock:
            state["prev_rx"] = None
            state["rate"] = 0.0
        with lock:
            topics, wsl_up = list(state["ros_topics"]), state["wsl_up"]
        return {"online": False, "busy": state["busy"], "log": state["log"],
                "wsl_up": wsl_up, "ros_topics": topics,
                "ros_cloud": "/cloud_registered" in topics}

    ok, out = ssh(STATUS_SH, timeout=20)
    if not ok:
        with lock:
            topics, wsl_up = list(state["ros_topics"]), state["wsl_up"]
        return {"online": False, "busy": state["busy"], "log": state["log"],
                "wsl_up": wsl_up, "ros_topics": topics,
                "ros_cloud": "/cloud_registered" in topics}

    d = {}
    for line in out.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            d[k.strip()] = v.strip()

    now = time.time()
    rx = int(d.get("rx", 0) or 0)
    with lock:
        prev = state["prev_rx"]
        if prev and now > prev[0] and rx >= prev[1]:
            state["rate"] = (rx - prev[1]) / (now - prev[0]) / 1e6   # MB/s
        state["prev_rx"] = (now, rx)
        rate = state["rate"]

    def flag(k):
        return d.get(k) == "1"

    with lock:
        topics = list(state["ros_topics"])
        wsl_up = state["wsl_up"]

    return {
        "online": True,
        "busy": state["busy"],
        "log": state["log"],
        "wsl_up": wsl_up,
        "ros_topics": topics,
        "ros_cloud": "/cloud_registered" in topics,
        "livox": flag("livox"),
        "fastlio": flag("fastlio"),
        "viewer": flag("viewer"),
        "zenoh": flag("zenoh"),
        "slam": flag("slam"),
        "scan": flag("scan"),
        "mapweb": flag("mapweb"),
        "cam": flag("cam"),
        "camweb": flag("camweb"),
        "camhz": float(d.get("camhz", 0) or 0),
        "colorpct": float(d.get("colorpct", 0) or 0),
        "ssd_mb": int(d.get("ssd", 0) or 0),
        # 這兩個是健康指標:FAST-LIO 逼近 3GB、slam_toolbox 逼近 1.5GB
        # 就代表快要被 cgroup 砍掉,畫面會先開始飄再整個停掉。
        "lio_mb": int(d.get("lio_mb", 0) or 0),
        "slam_mb": int(d.get("slam_mb", 0) or 0),
        "eth": flag("eth"),
        "lidar": flag("lidar"),
        "rate": round(rate, 2),
        "load": float(d.get("load", 0) or 0),
        "mem_used": int(d.get("mem_used", 0) or 0),
        "mem_total": int(d.get("mem_total", 1) or 1),
        "temp": float(d.get("temp", 0) or 0),
        "uptime": int(d.get("uptime", 0) or 0),
        "viewer_url": f"http://{HOST}:8080/index.html",
        "map_url": f"http://{HOST}:8090/",
        "cam_url": f"http://{HOST}:8092/",
    }


def run_async(kind, script, timeout):
    def work():
        with lock:
            state["busy"] = kind
            state["log"] = ""
        ok, out = ssh(script, timeout=timeout)
        with lock:
            # 啟動流程從 4 步變成 7 步,還多了網卡等待與 bind 失敗的提示,
            # 1500 字元會把開頭切掉,看不到是哪一步出問題。
            state["log"] = out.strip()[-3000:]
            if kind != "shutdown":
                state["busy"] = None
    threading.Thread(target=work, daemon=True).start()


# ---------------------------------------------------------------- HTTP


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.startswith("/api/status"):
            self._send(200, json.dumps(collect()))
        elif self.path in ("/", "/index.html"):
            self._send(200, (HERE / "ui.html").read_bytes(), "text/html; charset=utf-8")
        else:
            self._send(404, "not found", "text/plain; charset=utf-8")

    def do_POST(self):
        if self.path == "/api/start":
            run_async("starting", START_SH, 90)
        elif self.path == "/api/stop":
            run_async("stopping", STOP_SH, 60)
        elif self.path == "/api/shutdown":
            run_async("shutdown", SHUTDOWN_SH, 60)
        elif self.path == "/api/rviz":
            launch_rviz()
        else:
            return self._send(404, '{"ok":false}')
        self._send(200, '{"ok":true}')


def main():
    threading.Thread(target=ros_watcher, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", PANEL_PORT), Handler)
    url = f"http://127.0.0.1:{PANEL_PORT}/"
    print("=" * 52)
    print("  Jetson 光達展示控制台")
    print(f"  {url}")
    print(f"  目標:{USER}@{HOST}")
    print("  關閉這個視窗即結束控制台(不影響 Jetson)")
    print("=" * 52)
    threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
