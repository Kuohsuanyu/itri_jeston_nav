#!/usr/bin/env python3
"""底盤測試介面 —— 看所有 topic 的即時值,並手動送 /cmd_vel。

  瀏覽器 ──HTTP :8091──> 這支(跑在 Jetson) ──/cmd_vel──> 樹莓派 chassis_driver ──RS485──> VCU

安全設計(這台輪距 62cm、16 吋輪,不是小車):
  1. 預設「未啟用」,要明確按下啟用才會送出任何非零 /cmd_vel
  2. 伺服器端硬上限 MAX_VX / MAX_WZ,網頁傳什麼都會被夾住
  3. dead-man:瀏覽器每 200ms 要送一次心跳,超過 DEADMAN_SEC 沒收到就歸零並解除啟用
     —— 分頁關掉、網路斷掉、電腦當掉,車都會停
  4. 底盤驅動自己還有 0.5 秒逾時保護,這是第二道防線

注意 /cmd_vel 必須**持續**發布,只發一次車子會抽動一下就停。

環境變數:
  CHASSIS_CMD_TOPIC   要發布的 topic,預設 /cmd_vel。
                      之後接 Nav2 時改成 /cmd_vel_teleop,由 twist_mux 仲裁。
  CHASSIS_TOKEN       設了就必須在網址帶 ?t=<token> 才能操作。
  CHASSIS_PORT        HTTP 埠,預設 8091。
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# 關機服務是選配 —— chassis_msgs 沒 build 在這台機器上就只是少一顆按鈕,
# 不該讓整個介面起不來。
try:
    from chassis_msgs.srv import Shutdown
    HAS_SHUTDOWN = True
except ImportError:
    Shutdown = None
    HAS_SHUTDOWN = False

HTTP_PORT = int(os.environ.get("CHASSIS_PORT", "8091"))
CMD_TOPIC = os.environ.get("CHASSIS_CMD_TOPIC", "/cmd_vel")
TOKEN = os.environ.get("CHASSIS_TOKEN", "")

CMD_HZ = 10.0          # /cmd_vel 發布頻率
DEADMAN_SEC = 0.8      # 多久沒收到網頁心跳就歸零
MAX_VX = 0.50          # 硬上限,伺服器端夾住 (m/s)
MAX_WZ = 0.80          # 硬上限 (rad/s)

# 關機確認碼。要跟底盤 system_param.yaml 的 confirm_code 一致,
# 不一致的話服務會直接拒絕(這是它的設計,不是 bug)。
SHUTDOWN_CONFIRM = os.environ.get("CHASSIS_SHUTDOWN_CONFIRM", "SHUTDOWN")
SHUTDOWN_DELAY = float(os.environ.get("CHASSIS_SHUTDOWN_DELAY", "10"))

# 停用之後仍繼續送零速多久。送完就完全停止發布 —— 否則會一直用 0 蓋掉
# Nav2 的指令(兩個 publisher 同時存在時,底盤收到的是最後到的那筆)。
IDLE_ZERO_SEC = 1.5

# 這些型別內容太大,不訂閱,只列出名稱。
# 原版只擋了 PointCloud2/Image/CompressedImage,漏掉 livox 的 CustomMsg ——
# 那個 topic 10Hz、每則上萬個點,照樣訂閱會白白吃掉 Jetson 的 CPU。
SKIP_TYPES = (
    "sensor_msgs/msg/PointCloud2",
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "livox_ros_driver2/msg/CustomMsg",
    "nvblox_msgs/msg/Mesh",
    "nvblox_msgs/msg/VoxelBlockLayer",
    "nvblox_msgs/msg/DistanceMapSlice",
    "visualization_msgs/msg/MarkerArray",
)

# 這幾個 topic 在最上面的底盤面板單獨解析
CHASSIS_TOPICS = ("/odom", "/battery_state", "/diagnostics",
                  "/chassis/status", "/chassis/motor_state", "/joint_states")

state = {
    "enabled": False,
    "vx": 0.0,
    "wz": 0.0,
    "last_cmd": 0.0,       # 最後一次收到網頁心跳的時間
    "last_active": 0.0,    # 最後一次「啟用且有心跳」的時間
    "sent": 0,             # 已送出的 /cmd_vel 筆數
    "lock": threading.Lock(),
}

topics = {}      # name -> {"type":.., "count":.., "last":.., "hz":.., "text":..}
chassis = {}     # 解析後的底盤數值
tlock = threading.Lock()

# 關機。HTTP 執行緒只負責「排隊」,真正呼叫 service 交給 ROS 執行緒的計時器 ——
# rclpy 的 client 不保證跨執行緒安全,而且 call_async 需要 executor 在轉才有回應。
power = {
    "want": None,        # None / "shutdown" / "cancel"
    "pending": False,    # 來自 /system/shutdown_pending(latched)
    "msg": "",           # 服務回的訊息
    "at": 0.0,           # 最後一次有動作的時間
    "available": False,  # service 在不在
    "lock": threading.Lock(),
}


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def summarize(msg, depth=0, budget=None):
    """把訊息壓成一行摘要,只取純量欄位,陣列只印長度。

    原版是 `str(msg)[:300]` —— 對大訊息會先把**整包**序列化成字串再切掉,
    等於每秒做十次超大字串建構。這裡改成走欄位、遇到序列就只印長度,
    成本跟訊息大小無關。
    """
    if budget is None:
        budget = [220]
    try:
        fields = msg.get_fields_and_field_types()
    except AttributeError:
        return str(msg)[:60]

    parts = []
    for name, ftype in fields.items():
        if budget[0] <= 0:
            parts.append("…")
            break
        try:
            v = getattr(msg, name)
        except AttributeError:
            continue

        if "sequence" in ftype or ftype.endswith("]"):
            n = len(v) if hasattr(v, "__len__") else "?"
            piece = f"{name}[{n}]"
        elif hasattr(v, "get_fields_and_field_types"):
            if depth >= 2:
                continue
            piece = f"{name}{{{summarize(v, depth + 1, budget)}}}"
        elif isinstance(v, float):
            piece = f"{name}={v:.3f}"
        elif isinstance(v, (bytes, bytearray)):
            piece = f"{name}={v.hex()[:8]}"
        else:
            piece = f"{name}={v}"

        budget[0] -= len(piece)
        parts.append(piece)
    return " ".join(parts)


class Console(Node):
    def __init__(self):
        # jetson_ 前綴不是裝飾:這支跑在 Jetson 上,是「給人操作的遙控介面」,
        # 跟樹莓派上真正的 /chassis_driver 完全不同。
        # 舊名字叫 chassis_console,診斷時 grep chassis 會把它撈出來,
        # 讓人誤以為底盤已經上線 —— 實際上底盤一個節點都沒起來。
        super().__init__("jetson_chassis_console")
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_timer(1.0 / CMD_HZ, self.on_cmd_tick)
        self.create_timer(3.0, self.on_scan_topics)
        self.subs = {}

        # BEST_EFFORT 的訂閱者可以跟 RELIABLE 的發布者配對,反過來不行。
        # 這裡要看的是雜七雜八的 topic,用最寬鬆的 QoS 才不會有些 topic
        # 明明在發、卻因為 QoS 不相容而永遠收不到。
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- 關機服務 ----
        self.cli_off = self.cli_cancel = None
        if HAS_SHUTDOWN:
            self.cli_off = self.create_client(Shutdown, "/system/shutdown")
            self.cli_cancel = self.create_client(Trigger, "/system/shutdown_cancel")
            # latched:訂閱時就會收到目前狀態,不用等下一次變化
            self.create_subscription(
                Bool, "/system/shutdown_pending", self.on_pending,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST))
            self.create_timer(0.5, self.on_power_tick)
        else:
            self.get_logger().warn(
                "沒有 chassis_msgs,關機按鈕停用。"
                "解法:source ~/chassis_ws/install/setup.bash 再啟動這支")

        self.get_logger().info(f"jetson_chassis_console — HTTP :{HTTP_PORT}, 發布到 {CMD_TOPIC}")
        if not TOKEN:
            self.get_logger().warn(
                "沒有設 CHASSIS_TOKEN —— 同網段任何人都能開網頁操控這台車")

    # ---------------- 關機 ----------------

    def on_pending(self, msg):
        with power["lock"]:
            power["pending"] = bool(msg.data)

    def on_power_tick(self):
        """在 ROS 執行緒裡處理網頁排進來的關機/取消請求。"""
        with power["lock"]:
            power["available"] = bool(
                self.cli_off and self.cli_off.service_is_ready())
            want = power["want"]
            power["want"] = None
        if not want:
            return

        if want == "shutdown":
            if not (self.cli_off and self.cli_off.service_is_ready()):
                return self._power_msg("找不到 /system/shutdown —— 底盤沒開,"
                                       "或沒起 system_service 節點")
            req = Shutdown.Request()
            req.confirm = SHUTDOWN_CONFIRM
            req.delay_sec = float(SHUTDOWN_DELAY)
            req.reason = "chassis test UI"
            req.force = False       # 刻意不開:讓底盤自己做靜止檢查
            self.get_logger().warn("網頁請求關閉底盤")
            self.cli_off.call_async(req).add_done_callback(
                lambda f: self._on_off_done(f))
        elif want == "cancel":
            if not (self.cli_cancel and self.cli_cancel.service_is_ready()):
                return self._power_msg("找不到 /system/shutdown_cancel")
            self.cli_cancel.call_async(Trigger.Request()).add_done_callback(
                lambda f: self._power_msg(self._res_text(f)))

    def _on_off_done(self, fut):
        self._power_msg(self._res_text(fut))

    @staticmethod
    def _res_text(fut):
        try:
            r = fut.result()
            return ("✓ " if r.success else "✗ ") + (r.message or "")
        except Exception as e:
            return "✗ 呼叫失敗:%s" % e

    @staticmethod
    def _power_msg(text):
        with power["lock"]:
            power["msg"] = text
            power["at"] = time.monotonic()

    # ---------------- /cmd_vel ----------------

    def on_cmd_tick(self):
        now = time.monotonic()
        with state["lock"]:
            enabled = state["enabled"]
            stale = (now - state["last_cmd"]) > DEADMAN_SEC
            live = enabled and not stale
            vx = state["vx"] if live else 0.0
            wz = state["wz"] if live else 0.0
            if stale and enabled:
                # dead-man 觸發:自動解除啟用,避免恢復連線後車子突然又動
                state["enabled"] = False
                state["vx"] = state["wz"] = 0.0
            if live:
                state["last_active"] = now
            # 停用後只再送 IDLE_ZERO_SEC 的零速確保停住,然後完全閉嘴,
            # 不要一直跟 Nav2 搶 /cmd_vel。
            quiet = (not live) and (now - state["last_active"]) > IDLE_ZERO_SEC
            if not quiet:
                state["sent"] += 1
        if quiet:
            return

        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.pub.publish(t)

    # ---------------- topic 監看 ----------------

    def on_scan_topics(self):
        for name, types in self.get_topic_names_and_types():
            if not types:
                continue
            ttype = types[0]
            with tlock:
                if name not in topics:
                    topics[name] = {"type": ttype, "count": 0, "last": 0.0,
                                    "hz": 0.0, "text": "", "t0": time.monotonic(),
                                    "skipped": ttype in SKIP_TYPES}
            if name in self.subs or ttype in SKIP_TYPES:
                continue
            try:
                cls = get_message(ttype)
            except Exception as e:
                # 自訂型別在這台機器上沒安裝就會走到這裡(例如 chassis_msgs)。
                # 原版是靜默跳過,查不出原因 —— 這裡把它顯示在表格上。
                with tlock:
                    topics[name]["text"] = f"⚠ 本機沒有這個訊息型別({type(e).__name__})"
                    topics[name]["skipped"] = True
                continue
            self.subs[name] = self.create_subscription(
                cls, name, lambda m, n=name: self.on_any(n, m), self.qos)

    def on_any(self, name, msg):
        now = time.monotonic()
        with tlock:
            e = topics.get(name)
            if e is None:
                return
            e["count"] += 1
            e["last"] = now
            e["text"] = summarize(msg)
            dt = now - e["t0"]
            if dt >= 1.0:
                e["hz"] = e["count"] / dt
                e["count"] = 0
                e["t0"] = now
            if name in CHASSIS_TOPICS:
                self.parse_chassis(name, msg, now)

    def parse_chassis(self, name, msg, now):
        """把底盤的關鍵數值挑出來,給上方面板用。"""
        try:
            if name == "/odom":
                chassis["odom"] = {
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "vx": msg.twist.twist.linear.x,
                    "wz": msg.twist.twist.angular.z,
                    "t": now,
                }
            elif name == "/battery_state":
                chassis["batt"] = {"v": msg.voltage, "a": msg.current,
                                   "soc": msg.percentage, "t": now}
            elif name == "/chassis/status":
                chassis["status"] = {
                    "estop": bool(msg.emergency_stop),
                    "handle": bool(msg.handle_offline),
                    "drv1": bool(msg.driver1_offline),
                    "drv2": bool(msg.driver2_offline),
                    "t": now,
                }
            elif name == "/chassis/motor_state":
                chassis["motor"] = {
                    "lrpm": msg.left_rpm, "rrpm": msg.right_rpm,
                    "lalarm": bool(msg.left_alarm), "ralarm": bool(msg.right_alarm),
                    "lhall": msg.left_hall, "rhall": msg.right_hall,
                    "t": now,
                }
            elif name == "/diagnostics":
                for st in msg.status:
                    if "chassis" not in st.name.lower():
                        continue
                    lost = ""
                    for kv in st.values:
                        if kv.key == "total_lost_packets":
                            lost = kv.value
                    chassis["diag"] = {"level": int.from_bytes(st.level, "big")
                                       if isinstance(st.level, bytes) else int(st.level),
                                       "msg": st.message, "lost": lost, "t": now}
        except Exception:
            pass


PAGE = r"""<!doctype html><meta charset="utf-8">
<title>底盤測試介面</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
  :root{--bg:#0a0f16;--fg:#e6edf5;--dim:#7d8da3;--accent:#4ea1ff;--ok:#39d98a;
        --bad:#ff4d5e;--warn:#ffb648;--line:rgba(120,150,190,.20)}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);padding:18px;
       font:13px/1.6 ui-sans-serif,system-ui,"Noto Sans TC",sans-serif}
  h1{margin:0 0 4px;font-size:15px;letter-spacing:.06em}
  .sub{color:var(--dim);font-size:11.5px;margin-bottom:14px}
  .wrap{display:grid;grid-template-columns:340px 1fr;gap:16px;align-items:start}
  @media(max-width:900px){.wrap{grid-template-columns:1fr}}
  .card{background:rgba(16,22,30,.7);border:1px solid var(--line);
        border-radius:12px;padding:14px 16px;margin-bottom:14px}
  h2{margin:0 0 12px;font-size:11px;letter-spacing:.12em;text-transform:uppercase;
     color:var(--accent);font-weight:600}
  .row{display:flex;justify-content:space-between;padding:3px 0;gap:10px}
  .row span:first-child{color:var(--dim)}
  .row b{font-weight:500;font-variant-numeric:tabular-nums}
  button{background:rgba(78,161,255,.10);color:var(--fg);border:1px solid var(--line);
         border-radius:8px;padding:10px;cursor:pointer;font:inherit;font-size:13px;
         transition:.12s;user-select:none;-webkit-tap-highlight-color:transparent}
  button:hover{background:rgba(78,161,255,.24);border-color:var(--accent)}
  button:active{background:var(--accent);color:#04101f}
  #pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:10px}
  #pad button{height:52px;font-size:15px}
  #b-stop{grid-column:2;background:rgba(255,77,94,.16);border-color:rgba(255,77,94,.5)}
  #b-stop:hover{background:rgba(255,77,94,.35);border-color:var(--bad)}
  #b-enable{width:100%;margin-top:4px;padding:12px;font-weight:600}
  #b-enable.on{background:var(--ok);color:#04101f;border-color:var(--ok)}
  #b-off{width:100%;padding:11px;font-weight:600;
         background:rgba(255,77,94,.10);border-color:rgba(255,77,94,.35);
         color:#ffb0b8}
  #b-off:hover{background:rgba(255,77,94,.22);border-color:var(--bad)}
  #b-off.arm{background:var(--bad);color:#fff;border-color:var(--bad)}
  #b-cancel{width:100%;padding:11px;margin-top:8px;font-weight:600;
            background:rgba(255,182,72,.14);border-color:rgba(255,182,72,.45)}
  #pw-msg{color:var(--dim);font-size:11px;margin-top:9px;line-height:1.6;
          font-family:ui-monospace,monospace}
  label{display:flex;align-items:center;gap:10px;color:var(--dim);margin-top:12px}
  input[type=range]{flex:1;accent-color:var(--accent)}
  table{width:100%;border-collapse:collapse;font-size:11.5px}
  th{text-align:left;color:var(--dim);font-weight:500;padding:5px 8px 5px 0;
     border-bottom:1px solid var(--line);position:sticky;top:0;background:#0d131b}
  td{padding:4px 8px 4px 0;border-bottom:1px solid rgba(120,150,190,.08);
     vertical-align:top}
  td.v{color:var(--dim);font-family:ui-monospace,monospace;font-size:10.5px;
       max-width:520px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
  tr.hl td:first-child{color:var(--ok)}
  tr.skip td{opacity:.45}
  .dot{display:inline-block;width:7px;height:7px;border-radius:50%;margin-right:6px}
  .ok{background:var(--ok)} .bad{background:var(--bad)} .warn{background:var(--warn)}
  .off{background:#3a4557}
  #scroll{max-height:72vh;overflow:auto}
  .hint{color:var(--dim);font-size:11px;margin-top:10px;line-height:1.7}
  .big{font-size:17px;font-variant-numeric:tabular-nums}
</style>

<h1>底盤測試介面</h1>
<div class="sub">Jetson(上位機) → ROS 2 domain 0 → 樹莓派 chassis_driver → RS485 → VCU
  &nbsp;·&nbsp; 發布到 <b id="ctopic">—</b></div>

<div class="wrap">
  <div>
    <div class="card">
      <h2>底盤狀態</h2>
      <div class="row"><span><i class="dot off" id="d-link"></i>驅動節點</span><b id="v-link">—</b></div>
      <div class="row"><span><i class="dot off" id="d-diag"></i>診斷</span><b id="v-diag">—</b></div>
      <div class="row"><span>封包遺漏計數</span><b id="v-lost">—</b></div>
      <div class="row"><span>電池</span><b id="v-batt">—</b></div>
      <div class="row"><span>馬達 RPM(左/右)</span><b id="v-rpm">—</b></div>
      <div class="row"><span><i class="dot off" id="d-estop"></i>緊急開關</span><b id="v-estop">—</b></div>
      <div class="row"><span><i class="dot off" id="d-drv"></i>驅動器</span><b id="v-drv">—</b></div>
    </div>

    <div class="card">
      <h2>里程計</h2>
      <div class="row"><span>實測速度</span><b class="big" id="v-vx">—</b></div>
      <div class="row"><span>實測角速度</span><b id="v-wz">—</b></div>
      <div class="row"><span>位置 x / y</span><b id="v-xy">—</b></div>
    </div>

    <div class="card">
      <h2>手動控制</h2>
      <button id="b-enable">啟用控制(目前停用)</button>
      <!-- 預設停在滑桿最低檔。建圖時要繞著環境慢慢走，太快會讓 scan match 跟不上。 -->
      <label>速度上限 <input type="range" id="lim" min="0.05" max="0.5" step="0.05" value="0.05">
        <b id="v-lim" style="color:var(--fg);min-width:52px">0.05</b></label>
      <label>轉速上限 <input type="range" id="wlim" min="0.1" max="0.8" step="0.1" value="0.1">
        <b id="v-wlim" style="color:var(--fg);min-width:52px">0.10</b></label>
      <div id="pad">
        <div></div><button data-vx="1"  data-wz="0">▲ 前進</button><div></div>
        <button data-vx="0" data-wz="1">◀ 左轉</button>
        <button id="b-stop">■ 停</button>
        <button data-vx="0" data-wz="-1">右轉 ▶</button>
        <div></div><button data-vx="-1" data-wz="0">▼ 後退</button><div></div>
      </div>
      <div class="row" style="margin-top:10px"><span>送出中</span>
        <b id="v-cmd">vx 0.00 / wz 0.00</b></div>
      <div class="row"><span>已送出</span><b id="v-sent">0</b></div>
      <div class="hint">
        按住按鈕才會動,放開即停。也可用鍵盤 W / A / S / D。<br>
        分頁失焦、網路中斷或超過 0.8 秒沒有心跳,伺服器會自動歸零並解除啟用;
        底盤端另有 0.5 秒逾時保護。
      </div>
    </div>

    <div class="card">
      <h2>電源</h2>
      <button id="b-off">關閉底盤</button>
      <button id="b-cancel" style="display:none">取消關機</button>
      <div id="pw-msg">—</div>
      <div class="hint">
        關的是**底盤的樹莓派**,不是 Jetson。關掉之後要走過去按實體電源才能開回來。<br>
        要按兩次才會送出。底盤自己會檢查是否靜止,沒停穩會拒絕。
      </div>
    </div>
  </div>

  <div class="card">
    <h2>所有 Topic</h2>
    <div id="scroll">
      <table>
        <thead><tr><th style="width:215px">Topic</th><th style="width:185px">型別</th>
          <th style="width:58px">Hz</th><th>最新值</th></tr></thead>
        <tbody id="tb"></tbody>
      </table>
    </div>
    <div class="hint">點雲、影像、livox CustomMsg 等大型訊息不訂閱(只列名稱)。
      灰掉的列代表沒訂閱或本機缺少該訊息型別。</div>
  </div>
</div>

<script>
const $ = id => document.getElementById(id);
const TOKEN = new URLSearchParams(location.search).get('t') || '';
let enabled = false, vx = 0, wz = 0;

function lim()  { return parseFloat($('lim').value); }
function wlim() { return parseFloat($('wlim').value); }
$('lim').oninput  = () => $('v-lim').textContent  = lim().toFixed(2);
$('wlim').oninput = () => $('v-wlim').textContent = wlim().toFixed(2);

function setEnabled(on) {
  enabled = on;
  $('b-enable').classList.toggle('on', on);
  $('b-enable').textContent = on ? '控制已啟用(點此停用)' : '啟用控制(目前停用)';
  if (!on) { vx = wz = 0; }
}
$('b-enable').onclick = () => setEnabled(!enabled);

// 按住才動、放開就停 —— 不做「按一下持續前進」,對這種尺寸的車太危險
document.querySelectorAll('#pad button[data-vx]').forEach(b => {
  const set = () => { vx = parseFloat(b.dataset.vx) * lim();
                      wz = parseFloat(b.dataset.wz) * wlim(); };
  const clr = () => { vx = wz = 0; };
  b.addEventListener('mousedown', set);  b.addEventListener('mouseup', clr);
  b.addEventListener('mouseleave', clr);
  b.addEventListener('touchstart', e => { e.preventDefault(); set(); });
  b.addEventListener('touchend', clr);
});
$('b-stop').onclick = () => { vx = wz = 0; };

const KEYS = {w:[1,0], s:[-1,0], a:[0,1], d:[0,-1]};
addEventListener('keydown', e => {
  const k = KEYS[e.key.toLowerCase()];
  if (k && !e.repeat) { vx = k[0]*lim(); wz = k[1]*wlim(); }
  if (e.key === ' ') { vx = wz = 0; setEnabled(false); }
});
addEventListener('keyup', e => { if (KEYS[e.key.toLowerCase()]) { vx = wz = 0; } });

// 分頁被切走就停,避免看不到畫面時車還在跑
document.addEventListener('visibilitychange', () => { if (document.hidden) vx = wz = 0; });
addEventListener('blur', () => { vx = wz = 0; });

// 心跳 5Hz。伺服器 0.8 秒沒收到就歸零並解除啟用。
setInterval(async () => {
  try {
    const r = await fetch('/drive?t=' + encodeURIComponent(TOKEN), {method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({enabled, vx, wz})});
    if (r.status === 403) { $('v-cmd').textContent = '未授權(缺 token)'; return; }
    const j = await r.json();
    $('v-cmd').textContent = `vx ${j.vx.toFixed(2)} / wz ${j.wz.toFixed(2)}`;
    $('v-sent').textContent = j.sent.toLocaleString();
    if (!j.enabled && enabled) setEnabled(false);   // 伺服器端 dead-man 解除了
  } catch(e) {}
}, 200);

function dot(el, cls) { el.className = 'dot ' + cls; }

setInterval(async () => {
  try {
    const d = await (await fetch('/topics.json')).json();
    $('ctopic').textContent = d.cmd_topic;

    const tb = $('tb');
    tb.innerHTML = d.topics.map(t =>
      `<tr class="${t.hl?'hl':''} ${t.skipped?'skip':''}"><td>${t.name}</td>`
      + `<td style="color:#7d8da3">${t.type.replace(/^.*\//,'')}</td>`
      + `<td>${t.hz > 0.05 ? t.hz.toFixed(1) : '—'}</td>`
      + `<td class="v">${(t.text||'').replace(/</g,'&lt;')}</td></tr>`).join('');

    const c = d.chassis || {};
    dot($('d-link'), d.chassis_alive ? 'ok' : 'bad');
    $('v-link').textContent = d.chassis_alive ? '已連線' : '找不到';

    if (c.diag) {
      const L = c.diag.level;
      dot($('d-diag'), L===0?'ok':L===1?'warn':'bad');
      $('v-diag').textContent = (L===0?'OK':L===1?'WARN':'ERROR') + ' · ' + c.diag.msg;
      $('v-lost').textContent = c.diag.lost || '—';
    }
    if (c.batt) $('v-batt').textContent =
      `${c.batt.v.toFixed(1)} V / ${c.batt.a.toFixed(1)} A / ${c.batt.soc.toFixed(0)}%`;
    if (c.motor) {
      $('v-rpm').textContent = `${c.motor.lrpm} / ${c.motor.rrpm}`
        + ((c.motor.lalarm||c.motor.ralarm) ? '  ⚠ ALARM' : '');
    }
    if (c.status) {
      dot($('d-estop'), c.status.estop ? 'bad' : 'ok');
      $('v-estop').textContent = c.status.estop ? '已按下' : '正常';
      const bad = c.status.drv1 || c.status.drv2;
      dot($('d-drv'), bad ? 'bad' : 'ok');
      $('v-drv').textContent = bad
        ? `離線 ${c.status.drv1?'1 ':''}${c.status.drv2?'2':''}` : '兩顆都在線';
    }
    if (c.odom) {
      $('v-vx').textContent = c.odom.vx.toFixed(3) + ' m/s';
      $('v-wz').textContent = c.odom.wz.toFixed(3) + ' rad/s';
      $('v-xy').textContent = c.odom.x.toFixed(2) + ' / ' + c.odom.y.toFixed(2) + ' m';
    }
  } catch(e) {}
}, 500);
</script>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _authed(self):
        if not TOKEN:
            return True
        q = parse_qs(urlparse(self.path).query)
        return q.get("t", [""])[0] == TOKEN

    def do_POST(self):
        path = urlparse(self.path).path
        if path not in ("/drive", "/power"):
            return self._send(404, b"", "text/plain")
        if not self._authed():
            return self._send(403, b'{"error":"bad token"}', "application/json")
        try:
            n = int(self.headers.get("Content-Length", 0))
            req = json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            req = {}

        if path == "/power":
            act = req.get("action")
            if act not in ("shutdown", "cancel"):
                return self._send(400, b'{"error":"bad action"}', "application/json")
            # 只排隊,實際呼叫在 ROS 執行緒(Console.on_power_tick)
            with power["lock"]:
                power["want"] = act
                power["msg"] = "送出中…"
                power["at"] = time.monotonic()
            return self._send(200, json.dumps({"queued": act}), "application/json")
        with state["lock"]:
            state["enabled"] = bool(req.get("enabled"))
            # 伺服器端硬夾 —— 不信任網頁傳來的數值
            state["vx"] = clamp(float(req.get("vx", 0.0) or 0.0), -MAX_VX, MAX_VX)
            state["wz"] = clamp(float(req.get("wz", 0.0) or 0.0), -MAX_WZ, MAX_WZ)
            state["last_cmd"] = time.monotonic()
            out = {"enabled": state["enabled"], "vx": state["vx"],
                   "wz": state["wz"], "sent": state["sent"]}
        self._send(200, json.dumps(out), "application/json")

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/topics.json":
            now = time.monotonic()
            with tlock:
                rows = []
                for name, e in sorted(topics.items(),
                                      key=lambda kv: (kv[0] not in CHASSIS_TOPICS, kv[0])):
                    fresh = (now - e["last"]) < 3.0 if e["last"] else False
                    rows.append({"name": name, "type": e["type"],
                                 "hz": e["hz"] if fresh else 0.0,
                                 "text": e["text"] if (fresh or e.get("skipped")) else "",
                                 "hl": name in CHASSIS_TOPICS,
                                 "skipped": bool(e.get("skipped"))})
                od = topics.get("/odom", {})
                alive = bool(od.get("last") and (now - od["last"]) < 3.0)
                snap = {k: dict(v) for k, v in chassis.items()
                        if now - v.get("t", 0) < 5.0}
            with power["lock"]:
                pw = {"pending": power["pending"], "available": power["available"],
                      "msg": power["msg"] if now - power["at"] < 20 else "",
                      "supported": HAS_SHUTDOWN, "delay": SHUTDOWN_DELAY}
            self._send(200, json.dumps({"topics": rows, "chassis_alive": alive,
                                        "chassis": snap, "cmd_topic": CMD_TOPIC,
                                        "power": pw}),
                       "application/json")
            return
        self._send(200, PAGE, "text/html; charset=utf-8")


def main():
    rclpy.init()
    node = Console()
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"底盤測試介面  http://0.0.0.0:{HTTP_PORT}/   發布到 {CMD_TOPIC}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 收工前明確送幾筆零速,不要只靠底盤的逾時
        t = Twist()
        for _ in range(5):
            node.pub.publish(t)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
