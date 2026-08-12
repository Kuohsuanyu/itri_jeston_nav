#!/usr/bin/env python3
"""底盤檢查 —— 查 chassis_driver 在 ROS domain 0 的狀態。

  Windows ──plink SSH──> Jetson 192.168.40.98 ──ROS domain 0──> Pi 192.168.40.160
                                                                  chassis_driver
                                                                       │ RS485
                                                                       ▼ VCU

用法:雙擊「底盤檢查.bat」,或 python chassis_check.py

為什麼不寫成 .bat:CMD 的解析器會在 chcp 65001 生效前就把整行讀進去,
UTF-8 的中文會被拆成亂碼,後半段字串被當成指令執行。中文放 Python 才穩。
"""

import base64
import subprocess
import sys

HOST = "192.168.40.98"
USER = "andykuo"
PW = "2919"
HOSTKEY = "SHA256:ph8AvnetrS39dH8fnsuW8FPp9tsIg3wGecSXe/egfI4"
PLINK = r"C:\Program Files\PuTTY\plink.exe"

CHASSIS_PI = "192.168.40.160"

# 一律顯式 source,不靠 .bashrc
ROS = ("source /opt/ros/humble/setup.bash; "
       "source ~/ws_livox/install/setup.bash 2>/dev/null; "
       "export ROS_DOMAIN_ID=0; ")

# ros2 daemon 會快取節點清單。快取過期時「看不到遠端節點」——
# 症狀是 topic list 只剩本機的東西,很容易誤判成底盤根本沒開機。
# 任何要列舉節點/topic 的查詢,前面都得先把 daemon 停掉。
FRESH = ROS + "ros2 daemon stop >/dev/null 2>&1; sleep 2; "


def ssh(script, timeout=120):
    """在 Jetson 上執行 bash 腳本,即時把輸出印出來。"""
    b64 = base64.b64encode(script.encode("utf-8")).decode("ascii")
    cmd = [PLINK, "-ssh", "-batch", "-hostkey", HOSTKEY, "-pw", PW,
           f"{USER}@{HOST}", f"echo {b64} | base64 -d | bash"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                           encoding="utf-8", errors="replace")
        out = (r.stdout or "") + (r.stderr or "")
        print(out.rstrip() or "  (無輸出)")
    except subprocess.TimeoutExpired:
        print(f"  ✗ 逾時({timeout}s)")
    except FileNotFoundError:
        print(f"  ✗ 找不到 plink:{PLINK}")


def stream(script):
    """長時間執行的指令(topic echo),直接串到終端機讓 Ctrl+C 生效。"""
    b64 = base64.b64encode(script.encode("utf-8")).decode("ascii")
    cmd = [PLINK, "-ssh", "-batch", "-hostkey", HOSTKEY, "-pw", PW,
           f"{USER}@{HOST}", f"echo {b64} | base64 -d | bash"]
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass


# ---------------------------------------------------------------- 各項檢查

def c_nodes():
    print("=== 節點清單(fresh discovery,約 20 秒)===")
    print("看得到 /chassis_driver 就代表底盤驅動有在跑。\n")
    ssh(FRESH + "timeout 40 ros2 node list | sed 's/^/  /'", timeout=90)


def c_topics():
    print("=== 底盤 topic ===\n")
    ssh(FRESH + "timeout 40 ros2 topic list -t 2>/dev/null | "
        "grep -iE 'cmd_vel|/odom|battery|joint_state|diagnostic|chassis' | sed 's/^/  /'",
        timeout=90)
    print("\n  註:/chassis/motor_state 與 /chassis/status 是自訂型別。")
    print("     Jetson 上沒裝 chassis_msgs,所以 echo/hz 這兩個會失敗 —— ")
    print("     那是反序列化不了,不是沒發布。")


def c_rates():
    print("=== 發布頻率(每個量 10 秒,預期 10 Hz)===\n")
    ssh(ROS + """
for t in /odom /joint_states /battery_state /diagnostics; do
  R=$(timeout 11 ros2 topic hz $t 2>/dev/null | grep -m1 'average rate' | awk '{print $3}')
  echo "  $t : ${R:-NO DATA}"
done
""", timeout=90)


def c_health():
    print("=== 健康檢查 ===\n")
    print("--- 診斷 ---")
    ssh(ROS + "timeout 12 ros2 topic echo /diagnostics --once 2>/dev/null | "
        "grep -E 'level|message|key|value' | sed 's/^/  /'", timeout=40)
    print("\n--- 電池 ---")
    ssh(ROS + "timeout 12 ros2 topic echo /battery_state --once 2>/dev/null | "
        "grep -E 'voltage|current|percentage' | sed 's/^/  /'", timeout=40)
    print("  (電源監測模組的 CANbus 還沒接,全 0 是正常的)")

    print("\n--- odom 是否凍結(間隔 10 秒兩次取樣)---")
    ssh(ROS + """
timeout 10 ros2 topic echo /odom --once 2>/dev/null | grep -A2 'position:' | head -3 | sed 's/^/  t0  /'
sleep 10
timeout 10 ros2 topic echo /odom --once 2>/dev/null | grep -A2 'position:' | head -3 | sed 's/^/  t1  /'
""", timeout=90)
    print("\n  車靜止時兩次相同是正常的。")
    print("  要判斷「RS485 斷了但節點還在重播舊資料」,得在車行進中比對 ——")
    print("  行進中 odom 卻完全不動,或停車後 odom 仍持續前進,就是那個問題。")


def c_params():
    print("=== 目前生效的參數 ===\n")
    ssh(ROS + """
for p in port baudrate serial_timeout send_interval gear_ratio wheel_radius \
         wheel_separation cmd_left_direction cmd_right_direction \
         fb_left_direction fb_right_direction publish_tf cmd_vel_timeout; do
  echo "  $p = $(timeout 8 ros2 param get /chassis_driver $p 2>/dev/null | tail -1)"
done
""", timeout=150)
    print("\n  對照 vehicle_param_DD-M.yaml:gear_ratio 應為 50.0、fb_*_direction 應為 1。")
    print("  若跑出 30.0 / -1,代表 yaml 沒載到,節點用了程式內建預設值(不會報錯)。")


def c_lost():
    print("=== 封包遺漏計數 ===\n")
    ssh(ROS + "timeout 12 ros2 topic echo /diagnostics --once 2>/dev/null | "
        "grep -A1 total_lost_packets | sed 's/^/  /'", timeout=40)
    print("\n  這個數字目前不可信。seq 比對寫在 ROS timer(10Hz)裡,")
    print("  但封包是序列執行緒(也 10Hz)收的,兩個 loop 必然漂移。")
    print("  同一筆 state 被讀兩次時會算成「掉了 255 個」,所以會一直暴衝。")


def c_echo_odom():
    print("=== /odom 即時(Ctrl+C 離開)===\n")
    stream(ROS + "ros2 topic echo /odom")


def c_echo_diag():
    print("=== /diagnostics 即時(Ctrl+C 離開)===\n")
    stream(ROS + "ros2 topic echo /diagnostics")


def c_clear_alarm():
    print("=== 清除馬達 Alarm ===\n")
    ssh(ROS + "timeout 20 ros2 service call /clear_alarm std_srvs/srv/Trigger", timeout=40)
    print("\n  注意:這個服務有已知的 race —— 若清除請求送出後、序列執行緒")
    print("  還沒把它寫進封包前來了一筆 /cmd_vel,flag 會被洗掉、指令靜默遺失。")
    print("  沒反應的話多按幾次,或先確認沒有東西在發 /cmd_vel。")


def c_spdp():
    print("=== 網路層探測(不依賴 ROS discovery)===\n")
    print("直接聽 DDS 的 SPDP 廣播。topic list 空的時候用這個判斷對方到底有沒有開機 ——")
    print("能收到廣播就代表機器活著、ROS 有跑,只是 discovery 沒走完。\n")
    ssh(SPDP_SCRIPT, timeout=90)


def c_reach():
    print("=== 連線探測 ===\n")
    ssh(f"""
for ip in {HOST} {CHASSIS_PI}; do
  P=$(ping -c1 -W1 $ip >/dev/null 2>&1 && echo alive || echo "NO PING")
  S=$(timeout 2 bash -c "echo > /dev/tcp/$ip/22" 2>/dev/null && echo ssh-open || echo ssh-closed)
  echo "  $ip  $P  $S"
done
echo
echo "  ARP:"
ip neigh 2>/dev/null | grep -E "REACHABLE|STALE" | grep -v ":" | sed 's/^/    /'
""", timeout=60)


SPDP_SCRIPT = r'''
cat > /tmp/spdp_chk.py <<'PYEOF'
import socket, struct, time, collections, select
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("", 7400))                       # domain 0 = 7400 + 250*0
s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
             struct.pack("4sl", socket.inet_aton("239.255.0.1"), socket.INADDR_ANY))
s.setblocking(False)
LOCAL = ("192.168.40.98", "192.168.0.100", "127.0.0.1", "172.17.0.1")
seen = collections.defaultdict(set)
end = time.time() + 20
while time.time() < end:
    r, _, _ = select.select([s], [], [], 1.0)
    for sk in r:
        d, a = sk.recvfrom(65535)
        if len(d) >= 20 and d[:4] == b"RTPS":
            seen[a[0]].add(d[8:20].hex())
print()
for ip in sorted(seen):
    tag = "(Jetson 自己)" if ip in LOCAL else "<<< 遠端機器"
    print("  %-16s participants=%-3d %s" % (ip, len(seen[ip]), tag))
if not any(ip not in LOCAL for ip in seen):
    print("  沒有聽到任何遠端機器 —— 對方沒開機,或不在同一個網段")
PYEOF
python3 /tmp/spdp_chk.py
'''


MENU = [
    ("節點清單", "有沒有 /chassis_driver", c_nodes),
    ("底盤 topic 清單", "七個 topic 在不在", c_topics),
    ("發布頻率", "應該都是 10 Hz", c_rates),
    ("健康檢查", "診斷 / 電池 / odom 是否凍結", c_health),
    ("目前參數", "gear_ratio 等有沒有正確載入", c_params),
    ("封包遺漏計數", "total_lost_packets", c_lost),
    ("即時看 odom", "Ctrl+C 離開", c_echo_odom),
    ("即時看 診斷", "Ctrl+C 離開", c_echo_diag),
    ("清除馬達 Alarm", "", c_clear_alarm),
    ("網路層探測", "誰在 domain 0 廣播", c_spdp),
    ("連線探測", "ping / ssh / ARP", c_reach),
]


def main():
    while True:
        print()
        print("=" * 62)
        print("  底盤檢查   chassis_driver @ Raspberry Pi " + CHASSIS_PI)
        print(f"  (透過 Jetson {HOST} 的 ROS domain 0)")
        print("=" * 62)
        for i, (name, hint, _) in enumerate(MENU, 1):
            print(f"  {i:>2}  {name:<16}{hint}")
        print("   0  離開")
        try:
            c = input("\n選擇:").strip()
        except (EOFError, KeyboardInterrupt):
            return
        if c == "0":
            return
        if not c.isdigit() or not (1 <= int(c) <= len(MENU)):
            continue
        print()
        try:
            MENU[int(c) - 1][2]()
        except KeyboardInterrupt:
            print("\n  (中斷)")
        input("\n按 Enter 回到選單…")


if __name__ == "__main__":
    if sys.platform == "win32":
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except Exception:
            pass
    main()
