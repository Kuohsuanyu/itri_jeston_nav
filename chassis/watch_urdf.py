#!/usr/bin/env python3
"""監看 qbot_sensors.xacro,一存檔就重新產生 qbot_preview.urdf。

為什麼需要這支:
  展示器讀的是 qbot_preview.urdf,而你改的是 qbot_sensors.xacro。
  中間隔著產生器是因為 xacro 有巨集和 ${} 運算式,瀏覽器看不懂。
  忘了重跑產生器 -> 畫面不動 -> 以為改錯了。這支把那一步自動化。

用法:  python chassis/watch_urdf.py
       (開著就好,Ctrl+C 結束)
"""
import pathlib
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE / "chassis-ros2-driver" / "chassis_description" / "urdf" / "qbot_sensors.xacro"
GEN = HERE / "gen_preview_urdf.py"


def run():
    r = subprocess.run([sys.executable, str(GEN)], capture_output=True, text=True)
    out = (r.stdout or "") + (r.stderr or "")
    print(out.rstrip())
    if r.returncode != 0:
        print("  ↑ 產生失敗,先把 xacro 改回能解析的樣子")


def main():
    if not SRC.exists():
        print("找不到", SRC)
        return 1
    print("監看中:", SRC.name)
    print("存檔後會自動重產,然後重新整理瀏覽器即可。Ctrl+C 結束。")
    print("-" * 60)
    run()
    last = SRC.stat().st_mtime
    try:
        while True:
            time.sleep(0.5)
            m = SRC.stat().st_mtime
            if m != last:
                last = m
                # 編輯器有時會分兩次寫檔(先清空再寫),等它寫完
                time.sleep(0.25)
                print()
                print("[%s] 偵測到變更" % time.strftime("%H:%M:%S"))
                run()
    except KeyboardInterrupt:
        print("\n結束")
    return 0


if __name__ == "__main__":
    sys.exit(main())
