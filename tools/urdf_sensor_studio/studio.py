#!/usr/bin/env python3
"""URDF Sensor Studio —— 在任意 URDF 上擺放感測器,匯出成新的 URDF。

  雙擊 run.cmd,或在 cmd 裡:
      python studio.py
      python studio.py <urdf 路徑>

啟動後會問你 URDF 的路徑,然後開瀏覽器。在網頁上:
  1. 新增感測器(預設一個方塊,可以改尺寸)
  2. 或匯入 STL 當作感測器外型
  3. 選父連桿、調位置與角度,即時預覽
  4. 輸入輸出資料夾,匯出

匯出的東西是**自足的**:新的 URDF + 所有用到的 mesh 都複製一份進去,
路徑改寫成相對的 meshes/。這樣那個資料夾可以整包搬走、寄給別人,
不會因為 package:// 找不到套件而開不起來。

原始檔也會複製一份進輸出資料夾,檔名標記 .original —— 之後要對照
「我到底改了什麼」的時候會需要它。
"""
import http.server
import json
import os
import pathlib
import re
import shutil
import socketserver
import sys
import threading
import time
import urllib.parse
import webbrowser
import xml.etree.ElementTree as ET

PORT = 8096
HERE = pathlib.Path(__file__).resolve().parent
WEB = HERE / "web"

MODEL = {}          # 解析後的 URDF
ASSETS = []         # index -> {"name":..., "path": Path}  or {"name":..., "data": bytes}
SRC = None          # 來源 URDF 的路徑


# ----------------------------------------------------------------- URDF 解析

def resolve_mesh(fn, base):
    """把 URDF 的 mesh filename 變成實際檔案路徑。

    package://pkg/a/b.stl 是 ROS 的寫法,Windows 上沒有 ROS 的套件索引,
    所以從 URDF 所在位置往上找名字叫 pkg 的資料夾 —— 對絕大多數
    「套件資料夾就在附近」的情況都成立。
    """
    fn = fn.strip()
    if fn.startswith("file://"):
        fn = fn[7:]
    if fn.startswith("package://"):
        rest = fn[len("package://"):]
        pkg, _, tail = rest.partition("/")
        d = base
        for _ in range(6):
            for cand in (d / pkg / tail, d / tail):
                if cand.exists():
                    return cand.resolve()
            d = d.parent
        return None
    p = (base / fn) if not os.path.isabs(fn) else pathlib.Path(fn)
    return p.resolve() if p.exists() else None


def fl(s, n, default=0.0):
    if not s:
        return [default] * n
    v = [float(x) for x in s.replace(",", " ").split()]
    return (v + [default] * n)[:n]


def parse_visual(el, base):
    g = el.find("geometry")
    if g is None:
        return None
    o = el.find("origin")
    xyz = fl(o.get("xyz") if o is not None else None, 3)
    rpy = fl(o.get("rpy") if o is not None else None, 3)
    col = None
    m = el.find("material/color")
    if m is not None:
        col = fl(m.get("rgba"), 4, 1.0)

    out = {"xyz": xyz, "rpy": rpy, "color": col}
    if (b := g.find("box")) is not None:
        out.update(type="box", size=fl(b.get("size"), 3))
    elif (c := g.find("cylinder")) is not None:
        out.update(type="cylinder", radius=float(c.get("radius", 0.05)),
                   length=float(c.get("length", 0.1)))
    elif (s := g.find("sphere")) is not None:
        out.update(type="sphere", radius=float(s.get("radius", 0.05)))
    elif (me := g.find("mesh")) is not None:
        p = resolve_mesh(me.get("filename", ""), base)
        if p is None or p.suffix.lower() != ".stl":
            # 只支援 STL。其他格式(dae/obj)先跳過,不要整份載入失敗 ——
            # 少畫一個外殼不影響擺放感測器。
            return None
        ASSETS.append({"name": p.name, "path": p})
        out.update(type="mesh", asset=len(ASSETS) - 1,
                   scale=fl(me.get("scale"), 3, 1.0),
                   src=str(p))
    else:
        return None
    return out


def parse_urdf(path):
    txt = path.read_text(encoding="utf-8", errors="replace")
    if "xacro:" in txt:
        raise ValueError(
            "這是 xacro 檔,裡面有巨集和 ${} 運算式,不是純 URDF。\n"
            "  請先展開成 .urdf 再載入(ROS 上:xacro a.xacro > a.urdf)。")
    root = ET.fromstring(txt)
    base = path.parent
    ASSETS.clear()

    links, joints = [], []
    for L in root.findall("link"):
        vs = [v for v in (parse_visual(e, base) for e in L.findall("visual")) if v]
        links.append({"name": L.get("name"), "visuals": vs})
    for J in root.findall("joint"):
        o = J.find("origin")
        joints.append({
            "name": J.get("name"), "type": J.get("type", "fixed"),
            "parent": J.find("parent").get("link"),
            "child": J.find("child").get("link"),
            "xyz": fl(o.get("xyz") if o is not None else None, 3),
            "rpy": fl(o.get("rpy") if o is not None else None, 3)})

    children = {j["child"] for j in joints}
    roots = [l["name"] for l in links if l["name"] not in children]
    return {"name": root.get("name", "robot"), "path": str(path),
            "links": links, "joints": joints,
            "root": roots[0] if roots else (links[0]["name"] if links else None)}


# ----------------------------------------------------------------- 匯出

def export(out_dir, sensors):
    out = pathlib.Path(out_dir).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    mesh_dir = out / "meshes"

    src_txt = SRC.read_text(encoding="utf-8", errors="replace")
    root = ET.fromstring(src_txt)

    copied = {}

    def put_asset(i):
        """把 mesh 複製進輸出資料夾,回傳相對路徑。同一個檔只複製一次。"""
        if i in copied:
            return copied[i]
        a = ASSETS[i]
        mesh_dir.mkdir(exist_ok=True)
        name = a["name"]
        dst = mesh_dir / name
        n = 1
        while dst.exists() and dst.stat().st_size != _size(a):
            dst = mesh_dir / ("%s_%d%s" % (pathlib.Path(name).stem, n,
                                           pathlib.Path(name).suffix))
            n += 1
        if "path" in a:
            shutil.copy2(a["path"], dst)
        else:
            dst.write_bytes(a["data"])
        rel = "meshes/" + dst.name
        copied[i] = rel
        return rel

    def _size(a):
        return a["path"].stat().st_size if "path" in a else len(a["data"])

    # 原本的 mesh 路徑改寫成相對的,輸出資料夾才能整包搬走
    base = SRC.parent
    for me in root.iter("mesh"):
        p = resolve_mesh(me.get("filename", ""), base)
        if p is None:
            continue
        idx = next((i for i, a in enumerate(ASSETS)
                    if a.get("path") == p), None)
        if idx is None:
            ASSETS.append({"name": p.name, "path": p})
            idx = len(ASSETS) - 1
        me.set("filename", put_asset(idx))

    def fmt(v):
        return " ".join("%.6g" % x for x in v)

    for s in sensors:
        link = ET.SubElement(root, "link")
        link.set("name", s["name"])
        vis = ET.SubElement(link, "visual")
        ET.SubElement(vis, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geom = ET.SubElement(vis, "geometry")
        if s.get("asset") is not None:
            ET.SubElement(geom, "mesh", {"filename": put_asset(int(s["asset"])),
                                         "scale": fmt(s.get("scale", [1, 1, 1]))})
        else:
            ET.SubElement(geom, "box", {"size": fmt(s["size"])})
        mat = ET.SubElement(vis, "material", {"name": s["name"] + "_color"})
        ET.SubElement(mat, "color", {"rgba": fmt(list(s.get("color", [.3, .6, .9])) + [1])})

        j = ET.SubElement(root, "joint")
        j.set("name", s["name"] + "_joint")
        j.set("type", "fixed")
        ET.SubElement(j, "parent", {"link": s["parent"]})
        ET.SubElement(j, "child", {"link": s["name"]})
        ET.SubElement(j, "origin", {"xyz": fmt(s["xyz"]), "rpy": fmt(s["rpy"])})

    ET.indent(root, space="  ")
    stem = SRC.stem
    dst = out / (stem + "_with_sensors.urdf")
    if dst.exists():
        bak = out / ("%s_with_sensors.%s.urdf" % (stem, time.strftime("%Y%m%d-%H%M%S")))
        shutil.copy2(dst, bak)
    dst.write_bytes(b'<?xml version="1.0" encoding="utf-8"?>\n'
                    + ET.tostring(root, encoding="utf-8"))

    # 原始檔留一份,檔名標記 —— 之後要對照「改了什麼」一定會需要
    orig = out / (stem + ".original" + SRC.suffix)
    if not orig.exists():
        shutil.copy2(SRC, orig)

    (out / "README.txt").write_text(
        "由 URDF Sensor Studio 產生  %s\n\n"
        "  來源      %s\n"
        "  輸出      %s\n"
        "  原始備份  %s\n"
        "  mesh      meshes/  (路徑已改寫成相對,整包可搬走)\n\n"
        "新增的感測器:\n%s\n"
        % (time.strftime("%Y-%m-%d %H:%M:%S"), SRC, dst.name, orig.name,
           "".join("  %-22s parent=%-18s xyz=%s  rpy=%s\n"
                   % (s["name"], s["parent"], fmt(s["xyz"]), fmt(s["rpy"]))
                   for s in sensors) or "  (無)\n"),
        encoding="utf-8")

    files = [dst.name, orig.name, "README.txt"]
    if mesh_dir.exists():
        files += ["meshes/" + p.name for p in sorted(mesh_dir.iterdir())]
    return {"dir": str(out), "main": dst.name, "files": files}


# ----------------------------------------------------------------- HTTP

class H(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=str(WEB), **kw)

    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        b = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        if u.path == "/model.json":
            self._json(MODEL)
            return
        if u.path == "/asset":
            try:
                i = int(urllib.parse.parse_qs(u.query)["k"][0])
                a = ASSETS[i]
                b = a["data"] if "data" in a else a["path"].read_bytes()
            except Exception:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
            return
        super().do_GET()

    def do_POST(self):
        u = urllib.parse.urlparse(self.path)
        n = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(n)
        try:
            if u.path == "/upload":
                q = urllib.parse.parse_qs(u.query)
                name = q.get("name", ["imported.stl"])[0]
                ASSETS.append({"name": os.path.basename(name), "data": body})
                self._json({"ok": True, "key": len(ASSETS) - 1, "name": name})
                return
            if u.path == "/export":
                d = json.loads(body or b"{}")
                if not d.get("out_dir"):
                    self._json({"ok": False, "err": "請填輸出資料夾"}, 400)
                    return
                r = export(d["out_dir"], d.get("sensors", []))
                print("已匯出 ->", r["dir"])
                self._json({"ok": True, **r})
                return
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._json({"ok": False, "err": "%s: %s" % (type(e).__name__, e)}, 500)
            return
        self.send_error(404)


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


# ----------------------------------------------------------------- 進入點

def ask_path(argv):
    if len(argv) > 1:
        return argv[1]
    print()
    print("=" * 62)
    print("  URDF Sensor Studio")
    print("=" * 62)
    print("  輸入 URDF 檔案路徑(可以直接把檔案拖進這個視窗)")
    print("  只吃純 .urdf;xacro 請先展開")
    print()
    return input("  路徑> ").strip()


def main():
    global MODEL, SRC
    raw = ask_path(sys.argv)
    raw = raw.strip().strip('"').strip("'")
    if not raw:
        print("沒有輸入路徑,結束")
        return 1
    p = pathlib.Path(raw).expanduser()
    if not p.exists():
        print("找不到檔案:", p)
        return 1

    SRC = p.resolve()
    try:
        MODEL = parse_urdf(SRC)
    except Exception as e:
        print()
        print("載入失敗:", e)
        return 1

    print()
    print("  已載入", SRC.name)
    print("    連桿 %d   關節 %d   mesh %d   根節點 %s"
          % (len(MODEL["links"]), len(MODEL["joints"]), len(ASSETS), MODEL["root"]))
    miss = sum(1 for l in MODEL["links"] if not l["visuals"])
    if miss:
        print("    (%d 個連桿沒有可顯示的外型,通常是純座標系或非 STL 格式)" % miss)

    url = "http://127.0.0.1:%d/" % PORT
    srv = Server(("127.0.0.1", PORT), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print()
    print("  開啟", url)
    print("  關閉這個視窗即結束")
    try:
        webbrowser.open(url)
    except Exception:
        pass
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n結束")
    return 0


if __name__ == "__main__":
    sys.exit(main())
