#!/usr/bin/env python3
"""感測器位置量測工具 —— 用基準面標註距離,自動解出三維座標。

  python chassis/place_tool/server.py     然後開 http://127.0.0.1:8095/

原理:
  拿捲尺量東西的時候,你不會去想「相對輪軸中心的 XYZ 是多少」——
  你會量「距離盒子前面板 8 公分」「距離上蓋 12 公分」「置中」。
  每一個這樣的量測就是一個平面約束:

      n . p = d + t        n 是基準面法線,d 是基準面位置,t 是你量到的距離

  三個方向不平行的約束,就唯一決定了一個三維點。解 3x3 線性方程組即可。

  邊 = 兩個相鄰面的交線,所以選一條邊等於一次給兩個約束 ——
  這正好對應「從角落量」這個實際動作。

只處理位置。姿態(roll/pitch/yaw)要靠 IMU 和 ICP 量,捲尺量不出 1 度的精度。
"""
import json
import pathlib
import re
import shutil
import sys
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

PORT = 8095
HERE = pathlib.Path(__file__).resolve().parent
DESC = HERE.parent / "chassis-ros2-driver" / "chassis_description"
XACRO = DESC / "urdf" / "qbot_sensors.xacro"
CHASSIS_XACRO = DESC / "urdf" / "chassis_DD-M.xacro"

# 字串模板,不是編譯好的 pattern —— 每個 key 要代進去之後才編譯
PROP_TMPL = r'(<xacro:property\s+name="%s"\s+value=")([^"]*)(")'
ANY_PROP = re.compile(r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"')


def read_props():
    d = {}
    for f in (CHASSIS_XACRO, XACRO):
        d.update({m.group(1): m.group(2)
                  for m in ANY_PROP.finditer(f.read_text(encoding="utf-8"))})
    return d


# 每次寫入前,把被覆蓋的舊值推進來。選錯感測器是很容易犯的錯,
# 而且錯了之後光看 xacro 不知道原本是什麼 —— 所以復原必須存在。
# 只存在記憶體:伺服器重開就沒了,不過那時候你也早就發現寫錯了。
HISTORY = []
HISTORY_MAX = 50

# 檔案備份。復原按鈕只活在記憶體裡,伺服器一關就沒了 ——
# 所以磁碟上也要留一份,而且要分兩種:
#   .orig.xacro   永遠是「這個工具第一次動它之前」的樣子,只寫一次,絕不覆蓋
#   backup/*.xacro 每次寫入前的快照,帶時間戳,保留最近 30 份
# 只有 .orig 能回答「原廠/手寫的版本長什麼樣」,時間戳快照回答不了。
BACKUP_DIR = XACRO.parent / "backup"
BACKUP_KEEP = 30


def snapshot():
    orig = XACRO.parent / (XACRO.stem + ".orig" + XACRO.suffix)
    if not orig.exists():
        shutil.copy2(XACRO, orig)
        print("建立原始備份:", orig.name)
    BACKUP_DIR.mkdir(exist_ok=True)
    dst = BACKUP_DIR / ("%s.%s%s" % (XACRO.stem, time.strftime("%Y%m%d-%H%M%S"),
                                     XACRO.suffix))
    shutil.copy2(XACRO, dst)
    old = sorted(BACKUP_DIR.glob(XACRO.stem + ".*" + XACRO.suffix))
    for p in old[:-BACKUP_KEEP]:
        p.unlink()


def write_props(updates, record=True):
    """只改 value,其餘位元組原封不動 —— 不要用 XML 程式庫重寫整個檔案,
    那會把註解格式和縮排全部打亂,之後 diff 完全看不出改了什麼。"""
    txt = XACRO.read_text(encoding="utf-8")
    old = {m.group(1): m.group(2) for m in ANY_PROP.finditer(txt)}

    missing = [k for k in updates if k not in old]
    if missing:
        raise KeyError("xacro 裡找不到這些 property:" + ", ".join(missing))

    if record:
        snapshot()

    changed = []
    for k, v in updates.items():
        pat = re.compile(PROP_TMPL % re.escape(k))
        # repl 用函式而不是字串 —— 字串形式會把 v 裡的反斜線當成反向參照
        txt = pat.sub(lambda m: m.group(1) + v + m.group(3), txt, count=1)
        changed.append("%s=%s" % (k, v))

    XACRO.write_text(txt, encoding="utf-8")
    if record:
        HISTORY.append({k: old[k] for k in updates})
        del HISTORY[:-HISTORY_MAX]
    return changed


def undo():
    if not HISTORY:
        raise IndexError("沒有可復原的步驟")
    prev = HISTORY.pop()
    # 復原本身不進歷史,否則按兩次會在兩個狀態之間來回跳
    return write_props(prev, record=False)


def history_label():
    if not HISTORY:
        return None
    h = HISTORY[-1]
    who = "光達" if any(k.startswith("LIDAR") for k in h) else "相機"
    return "%s  ->  %s" % (who, "  ".join("%s=%s" % kv for kv in h.items()))


class H(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=str(HERE), **kw)

    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path.startswith("/params.json"):
            d = read_props()
            d["_undo_n"] = len(HISTORY)
            d["_undo_label"] = history_label()
            self._json(d)
            return
        if self.path.startswith("/mesh/"):
            name = self.path.split("/mesh/", 1)[1].split("?")[0]
            p = DESC / "meshes" / "DD-M" / name
            if not p.exists() or ".." in name:
                self.send_error(404)
                return
            b = p.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
            return
        SimpleHTTPRequestHandler.do_GET(self)

    def do_POST(self):
        if self.path.startswith("/undo"):
            try:
                changed = undo()
            except Exception as e:
                self._json({"ok": False, "err": str(e)}, 400)
                return
            print("復原:", ", ".join(changed))
            self._json({"ok": True, "changed": changed, "left": len(HISTORY)})
            return

        if not self.path.startswith("/write"):
            self.send_error(404)
            return
        n = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except Exception as e:
            self._json({"ok": False, "err": str(e)}, 400)
            return
        upd = {k: ("%.4f" % float(v)) for k, v in body.items()
               if re.fullmatch(r"(LIDAR|CAM)_(X|Y|Z|ROLL|PITCH|YAW)", k)}
        if not upd:
            self._json({"ok": False, "err": "沒有可寫入的欄位"}, 400)
            return
        # 任何例外都要變成 JSON 回去。讓它炸穿 handler 的話連線會被重置,
        # 瀏覽器只看得到 "Failed to fetch",真正的原因留在伺服器端 ——
        # 2026-08-08 的 PROP % 那個 bug 就是這樣被藏住的。
        try:
            changed = write_props(upd)
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._json({"ok": False, "err": "%s: %s" % (type(e).__name__, e)}, 500)
            return
        print("寫入 qbot_sensors.xacro:", ", ".join(changed))
        self._json({"ok": True, "changed": changed})


def main():
    if not XACRO.exists():
        print("找不到", XACRO)
        return 1
    ThreadingHTTPServer.allow_reuse_address = True
    print("感測器位置量測工具  http://127.0.0.1:%d/" % PORT)
    print("寫入目標:", XACRO)
    print("Ctrl+C 結束")
    ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()


if __name__ == "__main__":
    sys.exit(main())
