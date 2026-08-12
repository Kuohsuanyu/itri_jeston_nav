#!/usr/bin/env python3
"""2D 佔據網格網頁檢視器 —— 跟點雲檢視器(:8080)分開跑,各開一個瀏覽器分頁。

  slam_toolbox ── /map (OccupancyGrid) ──> 這支 ──HTTP :8090──> 瀏覽器 canvas

設計上刻意用「瀏覽器輪詢」而不是 WebSocket:
佔據網格更新頻率低(map_update_interval 1 秒),整張圖也才幾十 KB,
輪詢比維護一條 WS 連線單純得多,斷線也會自己好。
"""

import json
import math
import os
import struct
import subprocess
import threading
import time

import numpy as np
import rclpy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

HTTP_PORT = 8090
MAP_DIR = os.path.expanduser("~/maps")

REPLAN_SEC = 1.5      # 有目標時多久重算一次路徑
ARRIVE_M = 0.35       # 離目標多近算抵達,抵達就自動清掉目標

state = {
    "grid": None,        # dict: w, h, res, ox, oy, data(bytes)
    "pose": (0.0, 0.0, 0.0),
    "paused": False,
    "node": None,
    "path": [],          # 規劃出來的路徑 [(x, y), ...]
    "goal": None,        # 目前目標 (x, y)
    "nav_msg": "",       # 重新規劃的最新狀態,給網頁顯示
    "navigating": False, # NavigateToPose 進行中
    "nav_gh": None,      # goal handle,取消時要用
    "cmd": (0.0, 0.0),   # 控制器下的 (線速度, 角速度)
    "cmd_age": 0.0,      # 上次收到指令距今幾秒
    "pose_ok": False,    # TF 查不查得到,查不到網頁要說清楚原因
    "lock": threading.Lock(),
}

# 控制器輸出的 topic。--dry 模式下 Nav2 發到 _dry 那個,twist_mux 不讀它,
# 所以指令到不了輪子;正式跑的時候改成 /cmd_vel_nav。
# 兩個都訂閱,不用因為切換模式而改程式。
CMD_TOPICS = ("/cmd_vel_nav_dry", "/cmd_vel_nav")


def call_service(client, req, timeout=8.0):
    """從 HTTP 執行緒呼叫 ROS 服務。

    node 在主執行緒 spin,所以這裡只能 call_async 之後輪詢 future,
    交給主執行緒的 executor 去完成。直接用 spin_until_future_complete
    會跟主 spin 打架而卡死。
    """
    if not client.wait_for_service(timeout_sec=2.0):
        return None
    fut = client.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.05)
    return fut.result() if fut.done() else None


def _wait(fut, timeout):
    """輪詢 future —— 同樣不能用 spin_until_future_complete,會跟主 spin 打架。"""
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.05)
    return fut.result() if fut.done() else None


def plan_to(node, gx, gy):
    """向 Nav2 要一條到 (gx, gy) 的路徑。回傳 (點列表, 訊息)。"""
    if not node.ac_plan.wait_for_server(timeout_sec=3.0):
        return [], "Nav2 planner 沒有回應 —— start_nav2.sh 跑了嗎?"

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = gx
    goal.pose.position.y = gy
    goal.pose.orientation.w = 1.0

    g = ComputePathToPose.Goal()
    g.goal = goal
    g.use_start = False        # 起點用 TF 的當前位置

    gh = _wait(node.ac_plan.send_goal_async(g), 10.0)
    if gh is None or not gh.accepted:
        return [], "規劃請求沒被接受"

    res = _wait(gh.get_result_async(), 20.0)
    if res is None:
        return [], "規劃逾時"

    poses = res.result.path.poses
    if not poses:
        # tolerance 0.5 已經會找附近可行點,還是空的就是真的到不了
        return [], "找不到路徑 —— 目標在未知區、障礙裡,或與現在位置不連通"

    pts = [(p.pose.position.x, p.pose.position.y) for p in poses]
    length = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    return pts, f"路徑 {len(pts)} 點,長度 {length:.2f} m"


def start_navigation(node, gx, gy):
    """送 NavigateToPose。bt_navigator 會接手規劃 + 控制。

    在 --dry 模式下,控制器的輸出被導到 /cmd_vel_nav_dry,twist_mux 不讀,
    所以整條行為樹會完整跑一遍,但車一步都不會動。
    """
    if not node.ac_nav.wait_for_server(timeout_sec=3.0):
        return False, "bt_navigator 沒有回應 —— start_nav2_control.sh 跑了嗎?"

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = gx
    goal.pose.position.y = gy
    goal.pose.orientation.w = 1.0

    g = NavigateToPose.Goal()
    g.pose = goal
    gh = _wait(node.ac_nav.send_goal_async(g), 10.0)
    if gh is None or not gh.accepted:
        return False, "導航目標沒被接受"

    with state["lock"]:
        state["nav_gh"] = gh
        state["navigating"] = True
    return True, "導航已啟動(dry:速度只會顯示,不會送到輪子)"


def cancel_navigation():
    with state["lock"]:
        gh = state["nav_gh"]
        state["navigating"] = False
        state["nav_gh"] = None
    if gh is None:
        return False, "目前沒有進行中的導航"
    try:
        gh.cancel_goal_async()
    except Exception as e:
        return False, f"取消失敗:{e}"
    return True, "已取消導航"


def replan_loop():
    """有目標時持續重算路徑,讓路線跟著目前位置更新。

    **不能做成 ROS timer** —— plan_to 是輪詢 future(會 sleep),放在 timer
    裡會卡住主 spin 執行緒,連 TF 監聽和 /map 訂閱都會一起停擺。
    所以開獨立執行緒,future 仍由主執行緒的 executor 完成。
    """
    while True:
        time.sleep(REPLAN_SEC)
        node = state["node"]
        with state["lock"]:
            goal = state["goal"]
            rx, ry, _ = state["pose"]
        if node is None or goal is None:
            continue

        # 抵達就收工,免得在目標點上原地反覆規劃
        if math.dist((rx, ry), goal) <= ARRIVE_M:
            with state["lock"]:
                state["goal"] = None
                state["path"] = []
                state["nav_msg"] = "已抵達目標"
            continue

        pts, msg = plan_to(node, goal[0], goal[1])
        with state["lock"]:
            # 期間若被清除或改了目標就不要蓋掉
            if state["goal"] != goal:
                continue
            state["path"] = pts
            state["nav_msg"] = msg if pts else f"重新規劃失敗:{msg}"


class MapBridge(Node):
    def __init__(self):
        super().__init__("map_web_viewer")

        # slam_toolbox 用 TRANSIENT_LOCAL 發佈 /map(latched),
        # 訂閱端 QoS 不相符的話會一則都收不到 —— 這是最常見的坑。
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)

        self.tf_buf = Buffer()
        self.tf_lis = TransformListener(self.tf_buf, self)
        self.create_timer(0.2, self.on_pose)

        self.cli_save = self.create_client(SaveMap, "/slam_toolbox/save_map")
        # Nav2 只跑 planner_server,沒有 controller/bt_navigator ——
        # 直接要一條路徑,不驅動輪子。
        self.ac_plan = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        # 真正會驅動控制器的那一個。點地圖只做預覽,要按「開始導航」才送這個。
        self.ac_nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # 監看控制器實際下的速度。這是「路線和轉向對不對」唯一看得出來的地方
        # —— 路徑畫得漂亮不代表控制器會照著走。
        for topic in CMD_TOPICS:
            self.create_subscription(Twist, topic, self.on_cmd, 10)
        self.create_timer(0.5, self.on_cmd_age)

        os.makedirs(MAP_DIR, exist_ok=True)
        state["node"] = self
        self.get_logger().info(f"map_web_viewer 啟動 — HTTP :{HTTP_PORT}  地圖存放 {MAP_DIR}")

    def on_cmd(self, msg):
        with state["lock"]:
            state["cmd"] = (msg.linear.x, msg.angular.z)
            state["cmd_age"] = time.time()

    def on_cmd_age(self):
        # 控制器沒在跑的時候不會發 0,而是完全不發。沒有這個逾時判斷,
        # 畫面會一直停在最後一筆速度,看起來像車還在動。
        with state["lock"]:
            if state["cmd_age"] and time.time() - state["cmd_age"] > 1.0:
                state["cmd"] = (0.0, 0.0)

    def on_map(self, msg):
        g = {
            "w": msg.info.width,
            "h": msg.info.height,
            "res": msg.info.resolution,
            "ox": msg.info.origin.position.x,
            "oy": msg.info.origin.position.y,
            "data": np.asarray(msg.data, dtype=np.int8).tobytes(),
        }
        with state["lock"]:
            state["grid"] = g

    def on_pose(self):
        # base_lidar,不是 base_link。base_link 屬於底盤那棵樹,底盤沒開就查不到
        # —— 而且就算開了也不該用,那是輪速里程計推出來的位置。
        try:
            t = self.tf_buf.lookup_transform("map", "base_lidar", rclpy.time.Time())
        except Exception:
            with state["lock"]:
                state["pose_ok"] = False
            return
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with state["lock"]:
            state["pose"] = (t.transform.translation.x, t.transform.translation.y, yaw)
            state["pose_ok"] = True


PAGE = """<!doctype html><meta charset="utf-8">
<title>2D Occupancy Map — slam_toolbox</title>
<style>
  :root{--bg:#0a0f16;--fg:#e6edf5;--dim:#7d8da3;--accent:#4ea1ff;--line:rgba(120,150,190,.20)}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);
       font:13px/1.5 ui-sans-serif,system-ui,"Noto Sans TC",sans-serif;overflow:hidden}
  /* inset:0 會鋪滿整個視窗。它在 DOM 裡排在 #panel 後面,兩者又都是
     position:fixed / z-index:auto,所以 #wrap 會疊在面板上把點擊全部攔走
     —— 症狀就是按鈕看得到卻按不到。用 pointer-events 讓它不吃事件。 */
  #wrap{position:fixed;inset:0;display:grid;place-items:center;pointer-events:none}
  #wrap canvas{pointer-events:auto}
  canvas{image-rendering:pixelated;border:1px solid var(--line);border-radius:8px;
         background:#11161d;max-width:96vw;max-height:92vh;cursor:crosshair;
         transform-origin:center center;will-change:transform}
  canvas.drag{cursor:grabbing}
  #zoom{position:fixed;bottom:14px;left:14px;z-index:10;display:flex;gap:6px;
        align-items:center;background:rgba(16,22,30,.82);border:1px solid var(--line);
        border-radius:8px;padding:5px 8px;backdrop-filter:blur(12px);font-size:11px}
  #zoom button{width:auto;margin:0;padding:3px 9px;font-size:11px;
               background:rgba(78,161,255,.10);border-color:var(--line)}
  #zoom button:hover{background:rgba(78,161,255,.24);border-color:var(--accent)}
  #zoom span{color:var(--dim);min-width:44px;text-align:center;
             font-variant-numeric:tabular-nums}
  #panel{position:fixed;top:14px;left:14px;z-index:10;background:rgba(16,22,30,.82);
         border:1px solid var(--line);border-radius:10px;padding:10px 14px 12px;
         backdrop-filter:blur(12px);min-width:210px;max-width:250px}
  /* 收起時只留標題列。用 display:none 而不是 height:0 —— 後者留下的
     padding 和 border 還是會擋住地圖,而這個面板正好蓋在走道起點上。 */
  #panel.min{min-width:0;padding:6px 10px}
  #panel.min #body{display:none}
  #panel.min h1{margin:0}
  h1{margin:0 0 10px;font-size:12px;letter-spacing:.12em;text-transform:uppercase;
     color:var(--accent);font-weight:600;display:flex;align-items:center;gap:8px;
     cursor:pointer;user-select:none}
  h1 .tw{margin-left:auto;font-size:11px;color:var(--dim);transition:transform .15s}
  #panel.min h1 .tw{transform:rotate(-90deg)}
  h1:hover{color:#7fc0ff}
  .row{display:flex;justify-content:space-between;gap:16px;padding:2px 0}
  .row span:first-child{color:var(--dim)}
  #hint{position:fixed;bottom:14px;right:14px;color:var(--dim);font-size:11px;
        text-align:right;line-height:1.7;opacity:.72}
  button{width:100%;margin-top:9px;background:rgba(255,77,94,.10);color:var(--fg);
         border:1px solid rgba(255,77,94,.42);border-radius:7px;padding:8px;
         cursor:pointer;font:inherit;font-size:12px;transition:.15s}
  button:hover{background:rgba(255,77,94,.26);border-color:#ff4d5e}
  button:disabled{opacity:.45;cursor:default}
  .btns{display:flex;gap:7px;margin-top:11px}
  .btns button{margin-top:0}
  button.ghost{background:rgba(78,161,255,.10);border-color:var(--line)}
  button.ghost:hover{background:rgba(78,161,255,.24);border-color:var(--accent)}
  button.on{background:var(--accent);color:#04101f;border-color:var(--accent);font-weight:600}
  button.go{background:rgba(57,217,138,.14);border-color:rgba(57,217,138,.45)}
  button.go:hover{background:#39d98a;color:#04101f;font-weight:600;border-color:#39d98a}
  button.go.on{background:#39d98a;color:#04101f;font-weight:600}
  #drybar{margin-top:8px;padding:4px 7px;border-radius:5px;font-size:10.5px;
          text-align:center;letter-spacing:.06em;
          background:rgba(255,182,72,.13);border:1px solid rgba(255,182,72,.4);
          color:#ffb648}
  #drybar.live{background:rgba(255,77,94,.16);border-color:#ff4d5e;color:#ff8a95}
  .row span:last-child{font-variant-numeric:tabular-nums}
  #msg{margin-top:9px;font-size:11px;line-height:1.5;min-height:15px;color:var(--dim)}
  #msg.ok{color:#5fd08a} #msg.err{color:#ff8a95}
</style>
<div id="panel">
  <h1 id="h-toggle">Occupancy Map<span class="tw">▾</span></h1>
  <div id="body">
  <div class="row"><span>尺寸</span><span id="s-size">—</span></div>
  <div class="row"><span>解析度</span><span id="s-res">—</span></div>
  <div class="row"><span>已知格數</span><span id="s-known">—</span></div>
  <div class="row"><span>機器 X</span><span id="s-x">—</span></div>
  <div class="row"><span>機器 Y</span><span id="s-y">—</span></div>
  <div class="row"><span>更新</span><span id="s-age">—</span></div>
  <div class="row"><span>距目標</span><span id="s-remain">—</span></div>
  <div class="row"><span>指令 前進</span><span id="s-v">—</span></div>
  <div class="row"><span>指令 轉向</span><span id="s-w">—</span></div>
  <div class="btns">
    <button id="b-nav" class="go">開始導航</button>
    <button id="b-stop" class="ghost">停止</button>
  </div>
  <div id="drybar">DRY —— 速度只顯示,不送到輪子</div>
  <button id="b-save" class="ghost">儲存地圖 (.pgm/.yaml)</button>
  <button id="b-clear" class="ghost">清除目標</button>
  <button id="b-reset">清空地圖,重新蒐集</button>
  <div id="msg"></div>
  </div>
</div>
<div id="zoom">
  <button id="z-out">−</button><span id="z-val">100%</span><button id="z-in">+</button>
  <button id="z-fit">適應視窗</button>
</div>
<div id="wrap"><canvas id="c" width="400" height="400"></canvas></div>
<div id="hint"><b style="color:#4ea1ff">點一下 = 設定導航目標</b> &nbsp;·&nbsp;
拖曳 = 平移 &nbsp;·&nbsp; 滾輪 = 縮放<br>
白 = 可通行 &nbsp; 黑 = 障礙 &nbsp; 灰 = 未探索<br>
紅箭頭 = 機器位置與朝向 &nbsp; 藍線 = 規劃路徑 &nbsp; 綠圈 = 目標</div>

<script>
const c = document.getElementById('c'), ctx = c.getContext('2d');
const $ = id => document.getElementById(id);

/* ---------------- 平移 / 縮放 ----------------
   用 CSS transform 而不是重畫 canvas:繪圖程式碼完全不用動,而且
   getBoundingClientRect() 會把 transform 算進去,所以「點擊 -> 世界座標」
   那段換算原封不動就仍然正確。
   transform-origin 是 center,而 canvas 被 grid 置中於視窗,
   所以版面中心就是視窗中心。 */
const view = {tx: 0, ty: 0, z: 1};
const ZMIN = 0.2, ZMAX = 12;

function applyView(){
  c.style.transform =
    `translate(${view.tx}px, ${view.ty}px) scale(${view.z})`;
  $('z-val').textContent = Math.round(view.z * 100) + '%';
}
function zoomAt(sx, sy, factor){
  const cx = innerWidth / 2, cy = innerHeight / 2;
  const nz = Math.min(ZMAX, Math.max(ZMIN, view.z * factor));
  const k = nz / view.z;
  // 讓游標底下那一點保持不動
  view.tx = (sx - cx) - ((sx - cx) - view.tx) * k;
  view.ty = (sy - cy) - ((sy - cy) - view.ty) * k;
  view.z = nz;
  applyView();
}
function fitView(){ view.tx = 0; view.ty = 0; view.z = 1; applyView(); }

c.addEventListener('wheel', (e) => {
  e.preventDefault();
  zoomAt(e.clientX, e.clientY, e.deltaY < 0 ? 1.15 : 1 / 1.15);
}, {passive: false});

$('z-in').onclick  = () => zoomAt(innerWidth/2, innerHeight/2, 1.3);
$('z-out').onclick = () => zoomAt(innerWidth/2, innerHeight/2, 1/1.3);
$('z-fit').onclick = fitView;

let drag = null, moved = 0;
c.addEventListener('pointerdown', (e) => {
  if (e.button !== 0) return;
  drag = {x: e.clientX, y: e.clientY, tx: view.tx, ty: view.ty};
  moved = 0;
  c.setPointerCapture(e.pointerId);   // 拖到畫布外面也不會斷
  c.classList.add('drag');
});
c.addEventListener('pointermove', (e) => {
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
  moved = Math.max(moved, Math.hypot(dx, dy));
  view.tx = drag.tx + dx; view.ty = drag.ty + dy;
  applyView();
});
c.addEventListener('pointerup', (e) => {
  if (!drag) return;
  drag = null;
  c.classList.remove('drag');
  // 移動不到 5 px 才算「點選目標」—— 否則每次拖曳都會誤設一個導航目標
  if (moved < 5) pickGoal(e);
});

/* 面板收起。地圖起點常常就在左上角,面板不收起來就是看不到。 */
$('h-toggle').onclick = () => $('panel').classList.toggle('min');
let last = 0;
let geo = null;      // 目前地圖的幾何,點擊時用來反算世界座標
let nav = null;      // {path:[[x,y]...], goal:[x,y]}

// 世界座標 -> 畫布像素。OccupancyGrid 第 0 列在下方,canvas 的 y 往下,所以要翻。
function w2c(x, y){
  return [ (x - geo.ox) / geo.res * geo.scale,
           (geo.h - (y - geo.oy) / geo.res) * geo.scale ];
}
// 畫布像素 -> 世界座標(上面的反函數)
function c2w(px, py){
  return [ geo.ox + px / geo.scale * geo.res,
           geo.oy + (geo.h - py / geo.scale) * geo.res ];
}

async function tick(){
  try{
    try { nav = await (await fetch('/nav.json?t=' + Date.now())).json(); } catch(_) {}
    const r = await fetch('/map.bin?t=' + Date.now());
    if (r.status === 204) { $('s-age').textContent = '等待 /map…'; return; }
    const b = new DataView(await r.arrayBuffer());

    const w = b.getUint32(4, true), h = b.getUint32(8, true);
    const res = b.getFloat32(12, true);
    const rx = b.getFloat32(24, true), ry = b.getFloat32(28, true), ryaw = b.getFloat32(32, true);
    const ox = b.getFloat32(16, true), oy = b.getFloat32(20, true);
    const cells = new Int8Array(b.buffer, 36, w*h);

    // 螢幕上放大到看得清楚,但別超過視窗
    const scale = Math.max(1, Math.floor(Math.min(1400/w, 900/h)));
    if (c.width !== w*scale || c.height !== h*scale){ c.width = w*scale; c.height = h*scale; }

    const img = ctx.createImageData(w, h);
    let known = 0;
    for (let y = 0; y < h; y++){
      // OccupancyGrid 第 0 列在原點(下方),canvas 的 y 往下,所以要翻轉
      const src = (h - 1 - y) * w;
      for (let x = 0; x < w; x++){
        const v = cells[src + x];
        const o = (y*w + x) * 4;
        let g;
        if (v < 0) { g = 58; }                       // 未知
        else { known++; g = 245 - Math.round(v*2.35); }  // 0 -> 亮, 100 -> 暗
        img.data[o] = img.data[o+1] = img.data[o+2] = g;
        img.data[o+3] = 255;
      }
    }
    const off = new OffscreenCanvas(w, h);
    off.getContext('2d').putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0,0,c.width,c.height);
    ctx.drawImage(off, 0, 0, c.width, c.height);

    // 存起來供點擊時反算世界座標
    geo = {scale, w, h, res, ox, oy};

    // 規劃路徑(畫在機器人下層,箭頭才不會被蓋住)
    if (nav && nav.path && nav.path.length > 1) {
      ctx.strokeStyle = '#4ea1ff'; ctx.lineWidth = Math.max(2, scale * 0.6);
      ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.beginPath();
      nav.path.forEach((p, i) => {
        const c = w2c(p[0], p[1]);
        i ? ctx.lineTo(c[0], c[1]) : ctx.moveTo(c[0], c[1]);
      });
      ctx.stroke();
    }
    if (nav && nav.goal) {
      const c = w2c(nav.goal[0], nav.goal[1]);
      ctx.strokeStyle = '#39d98a'; ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.arc(c[0], c[1], 9, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(c[0] - 13, c[1]); ctx.lineTo(c[0] + 13, c[1]);
      ctx.moveTo(c[0], c[1] - 13); ctx.lineTo(c[0], c[1] + 13);
      ctx.stroke();
    }

    // 機器位置:世界座標 -> 格子 -> 畫布
    const gx = (rx - ox) / res, gy = (ry - oy) / res;
    const px = gx * scale, py = (h - gy) * scale;
    ctx.save();
    ctx.translate(px, py); ctx.rotate(-ryaw);
    ctx.fillStyle = '#ff4d5e';
    ctx.beginPath();
    ctx.moveTo(11,0); ctx.lineTo(-7,7); ctx.lineTo(-3,0); ctx.lineTo(-7,-7);
    ctx.closePath(); ctx.fill();
    ctx.restore();

    $('s-size').textContent = w + ' x ' + h;
    $('s-res').textContent = res.toFixed(3) + ' m';
    $('s-known').textContent = known.toLocaleString();
    $('s-x').textContent = rx.toFixed(2) + ' m';
    $('s-y').textContent = ry.toFixed(2) + ' m';

    // 導航狀態:重新規劃是伺服器端每 1.5 秒自動跑的,這裡只負責顯示
    if (nav && nav.goal) {
      $('s-remain').textContent = nav.remain.toFixed(2) + ' m';
      if (nav.msg) say(nav.msg, nav.path && nav.path.length > 1);
    } else {
      $('s-remain').textContent = '—';
      if (nav && nav.msg === '已抵達目標') { say('已抵達目標', true); nav.msg = ''; }
    }

    last = Date.now();
    $('s-age').textContent = '剛剛';
  } catch(e){ $('s-age').textContent = '連線中斷'; }
}
/* ---------------- 互動按鈕 ---------------- */
function say(text, ok){
  const m = $('msg');
  m.textContent = text;
  m.className = ok === undefined ? '' : (ok ? 'ok' : 'err');
}

// 每個動作都把伺服器回來的訊息顯示出來,按下去到底有沒有反應一目了然。
async function act(path, btn, busyText, restoreText, holdMs){
  btn.disabled = true;
  const old = btn.textContent;
  btn.textContent = busyText;
  say('執行中…');
  try{
    const r = await fetch(path, {method:'POST'});
    const j = await r.json();
    say(j.msg, j.ok);
  } catch(e){ say('連線失敗:' + e, false); }
  setTimeout(() => { btn.disabled = false;
                     btn.textContent = restoreText || old; }, holdMs || 0);
}

$('b-reset').onclick = (e) => {
  if (!confirm('清空目前的 2D 地圖並重新開始蒐集?')) return;
  $('s-age').textContent = '重建中…';
  // slam_toolbox 重啟約 6 秒,期間 /map 沒有資料,按鈕先鎖住避免連按
  act('/reset', e.target, '重啟 slam_toolbox…', '清空地圖,重新蒐集', 9000);
};
$('b-save').onclick  = (e) => act('/save',  e.target, '儲存中…', '儲存地圖 (.pgm/.yaml)', 1200);
$('b-clear').onclick = (e) => act('/clear_goal', e.target, '清除中…', '清除目標', 400);

// 點地圖選導航目標。canvas 有 CSS 縮放,要用 getBoundingClientRect 換算,
// 直接拿 offsetX 會在畫面被縮小時算錯位置。
async function pickGoal(ev){
  if (!geo) return;
  const r = c.getBoundingClientRect();
  const px = (ev.clientX - r.left) * (c.width / r.width);
  const py = (ev.clientY - r.top) * (c.height / r.height);
  const [x, y] = c2w(px, py);
  say(`規劃到 (${x.toFixed(2)}, ${y.toFixed(2)})…`);
  try{
    const res = await fetch('/goal', {method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({x, y})});
    const j = await res.json();
    say(j.msg, j.ok);
    nav = await (await fetch('/nav.json?t=' + Date.now())).json();
  } catch(e){ say('規劃失敗:' + e, false); }
}

applyView();
/* 導航控制。點地圖只是預覽路徑,要按這裡才會真的叫 bt_navigator ——
   分開是刻意的:拖曳或誤點都不該啟動導航。 */
$('b-nav').onclick  = (e) => act('/navigate',   e.target, '啟動中…', '開始導航', 600);
$('b-stop').onclick = (e) => act('/cancel_nav', e.target, '停止中…', '停止', 400);

function updateNavUI(){
  if (!nav) return;
  const v = nav.v || 0, w = nav.w || 0;
  // 控制器沒在動的時候顯示 0.00 而不是空白 —— 空白分不出「沒指令」和「零速」
  $('s-v').textContent = v.toFixed(3) + ' m/s';
  $('s-w').textContent = w.toFixed(3) + ' rad/s';
  $('s-v').style.color = Math.abs(v) > 0.001 ? '#39d98a' : '';
  $('s-w').style.color = Math.abs(w) > 0.001 ? '#39d98a' : '';
  $('b-nav').classList.toggle('on', !!nav.nav);
  $('b-nav').textContent = nav.nav ? '導航中…' : '開始導航';
  if (nav.pose_ok === false){
    $('s-x').textContent = 'TF 查不到';
    $('s-y').textContent = 'map→base_lidar';
  }
}
setInterval(updateNavUI, 500);

setInterval(tick, 1000); tick();
setInterval(() => {
  if (last) { const s = Math.round((Date.now()-last)/1000);
              if (s > 2) $('s-age').textContent = s + ' 秒前'; }
}, 1000);
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

    def _json(self, ok, msg, **extra):
        body = json.dumps({"ok": ok, "msg": msg, **extra})
        self._send(200, body, "application/json; charset=utf-8")

    def do_POST(self):
        node = state["node"]

        if self.path == "/reset":
            # 先把手上的舊圖丟掉,網頁才會立刻顯示「重建中」而不是停在舊畫面
            with state["lock"]:
                state["grid"] = None
                state["paused"] = False
            subprocess.Popen(
                ["bash", os.path.expanduser("~/slam2d/restart_slam.sh")],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            return self._json(True, "已清空,slam_toolbox 重啟中(約 6 秒)")

        # /pause 端點刻意不提供。
        # slam_toolbox 2.6.10 的 pause_new_measurements 文件上說是 toggle,
        # 但實測(ros2 service call 直接叫也一樣)連續三次都回 status=True,
        # 也就是只能暫停、無法恢復。留一個按下去回不來的按鈕只會讓人
        # 誤以為 2D 地圖壞了,所以介面上不放。要重新開始請用「清空地圖」。

        if self.path == "/goal":
            if node is None:
                return self._json(False, "節點尚未就緒")
            try:
                n = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(n) or b"{}")
                gx, gy = float(req["x"]), float(req["y"])
            except Exception as e:
                return self._json(False, f"目標格式錯誤:{e}")

            pts, msg = plan_to(node, gx, gy)
            with state["lock"]:
                state["path"] = pts
                state["goal"] = (gx, gy) if pts else None
            return self._json(bool(pts), msg)

        if self.path == "/navigate":
            node = state["node"]
            with state["lock"]:
                goal = state["goal"]
            if node is None:
                return self._json(False, "節點尚未就緒")
            if goal is None:
                return self._json(False, "先在地圖上點一個目標")
            ok, msg = start_navigation(node, goal[0], goal[1])
            return self._json(ok, msg)

        if self.path == "/cancel_nav":
            ok, msg = cancel_navigation()
            return self._json(ok, msg)

        if self.path == "/clear_goal":
            with state["lock"]:
                state["path"] = []
                state["goal"] = None
            return self._json(True, "已清除目標")

        if self.path == "/save":
            if node is None:
                return self._json(False, "節點尚未就緒")
            name = os.path.join(MAP_DIR, "map_" + time.strftime("%Y%m%d_%H%M%S"))
            req = SaveMap.Request()
            req.name = String(data=name)
            r = call_service(node.cli_save, req, timeout=20.0)
            if r is None:
                return self._json(False, "呼叫 save_map 服務逾時")
            if r.result != 0:
                return self._json(False, f"儲存失敗,result={r.result}")
            return self._json(True, f"已存到 {name}.pgm / .yaml")

        return self._json(False, "unknown endpoint")

    def do_GET(self):
        if self.path.startswith("/nav.json"):
            with state["lock"]:
                pts, goal = list(state["path"]), state["goal"]
                msg, (rx, ry, _) = state["nav_msg"], state["pose"]
            with state["lock"]:
                nav_on = state["navigating"]
                cv, cw = state["cmd"]
                pose_ok = state["pose_ok"]
            remain = math.dist((rx, ry), goal) if goal else 0.0
            self._send(200, json.dumps({"path": pts, "goal": goal,
                                        "msg": msg, "remain": remain,
                                        "nav": nav_on, "v": cv, "w": cw,
                                        "pose_ok": pose_ok}),
                       "application/json")
            return

        if self.path.startswith("/map.bin"):
            with state["lock"]:
                g = state["grid"]
                rx, ry, ryaw = state["pose"]
            if g is None:
                self.send_response(204)
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            head = b"MAP1" + struct.pack(
                "<IIffffff", g["w"], g["h"], g["res"], g["ox"], g["oy"], rx, ry, ryaw
            )
            self._send(200, head + g["data"], "application/octet-stream")
        else:
            self._send(200, PAGE, "text/html; charset=utf-8")


def main():
    rclpy.init()
    node = MapBridge()
    threading.Thread(
        target=lambda: ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler).serve_forever(),
        daemon=True,
    ).start()
    threading.Thread(target=replan_loop, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
