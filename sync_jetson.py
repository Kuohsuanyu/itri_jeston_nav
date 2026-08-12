#!/usr/bin/env python3
"""Windows 這份 repo 和 Jetson 之間的雙向同步 + 差異檢查。

── 為什麼需要 ────────────────────────────────────────────────────
之前的做法是「改完檔案手動 pscp 幾個檔上去」,問題是:

  1. 漏傳不會有任何警告。改了三個檔只傳兩個,系統照跑,行為卻是混的。
  2. Jetson 上有 repo 沒有的檔(slam_params.yaml 就是),
     只能遠端 sed —— 改了什麼沒有版本記錄。
  3. 「傳上去」不等於「生效」。行程是啟動時讀設定的,不重啟就還是舊的。
     2026-08-11 實測:slam_params.yaml 15:20 改好,而跑著的 slam_toolbox
     是 14:47 啟動的,用著 base_frame: box_link 跑了一個多小時。

所以這支做三件事,而且**先比對再動手**:

    python sync_jetson.py check     兩邊逐檔比 SHA256,列出差異(不改任何東西)
    python sync_jetson.py pull      把 Jetson 上的檔抓回 repo(含 repo 沒有的)
    python sync_jetson.py push      把 repo 推上 Jetson
    python sync_jetson.py push --restart   推完順便重啟受影響的行程

★ check 是預設,不帶參數就只比對。任何會改東西的動作都要明講。
"""
import argparse
import base64
import hashlib
import pathlib
import subprocess
import sys

HOST, USER, PW = "192.168.40.98", "andykuo", "2919"
HOSTKEY = "SHA256:ph8AvnetrS39dH8fnsuW8FPp9tsIg3wGecSXe/egfI4"
PLINK = r"C:\Program Files\PuTTY\plink.exe"
PSCP = r"C:\Program Files\PuTTY\pscp.exe"

# Windows 主控台預設是 cp950,印不出 ✓ ✗ ≠ 這些字會直接丟例外 ——
# 而且是在錯誤處理裡丟,把真正的錯誤訊息蓋掉。
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.stderr.reconfigure(encoding="utf-8", errors="replace")

ROOT = pathlib.Path(__file__).resolve().parent

# (repo 目錄, Jetson 目錄, 要同步的副檔名)
PAIRS = [
    (ROOT / "jetson_deploy" / "scripts", "/home/andykuo/slam2d",
     (".sh", ".py", ".yaml", ".xml")),
    (ROOT / "jetson_deploy" / "nav2", "/home/andykuo/nav2",
     (".sh", ".py", ".yaml", ".xml")),
]

# 改了這些檔就要重啟對應的東西。空字串代表「只要重跑一次腳本」。
RESTART_HINT = {
    "robot_tf.sh": "bash ~/slam2d/start_slam2d.sh",
    "odom_cov_relay.py": "bash ~/slam2d/start_slam2d.sh",
    "ekf_multi.yaml": "bash ~/slam2d/start_slam2d.sh",
    "slam_params.yaml": "bash ~/slam2d/start_slam2d.sh",
    "start_slam2d.sh": "bash ~/slam2d/start_slam2d.sh",
    "tf_static_repeat.py": "bash ~/slam2d/start_slam2d.sh",
    "nav2_control.yaml": "bash ~/nav2/start_nav2_control.sh",
    "twist_mux.yaml": "bash ~/nav2/start_nav2_control.sh",
    "start_zenoh_jetson.sh": "bash ~/slam2d/start_zenoh_jetson.sh",
    "fastdds_peers.xml": "bash ~/slam2d/start_zenoh_jetson.sh",
}


def ssh(script, timeout=120):
    b64 = base64.b64encode(script.encode("utf-8")).decode("ascii")
    r = subprocess.run(
        [PLINK, "-ssh", "-batch", "-hostkey", HOSTKEY, "-pw", PW,
         f"{USER}@{HOST}", f"echo {b64} | base64 -d | bash"],
        capture_output=True, text=True, encoding="utf-8", errors="replace",
        timeout=timeout)
    if r.returncode != 0 and not r.stdout.strip():
        raise RuntimeError("SSH 失敗:%s" % (r.stderr or "")[:200])
    return r.stdout


def scp(src, dst, timeout=120):
    r = subprocess.run(
        [PSCP, "-batch", "-pw", PW, "-hostkey", HOSTKEY, str(src), str(dst)],
        capture_output=True, text=True, encoding="utf-8", errors="replace",
        timeout=timeout)
    if r.returncode != 0:
        raise RuntimeError("scp 失敗 %s -> %s:%s" % (src, dst, (r.stderr or "")[:150]))


def local_hashes(d, exts):
    out = {}
    if not d.is_dir():
        return out
    for f in sorted(d.iterdir()):
        if f.is_file() and f.suffix in exts:
            # 統一成 LF 再算,不然 CRLF/LF 的差異會讓每個檔都「不一樣」
            data = f.read_bytes().replace(b"\r\n", b"\n")
            out[f.name] = hashlib.sha256(data).hexdigest()
    return out


def remote_hashes(rd, exts):
    pat = " -o ".join('-name "*%s"' % e for e in exts)
    script = (
        'cd %s 2>/dev/null || exit 0\n'
        'find . -maxdepth 1 -type f \\( %s \\) -printf "%%f\\n" | sort | '
        'while read f; do printf "%%s %%s\\n" "$(tr -d "\\r" < "$f" | '
        'sha256sum | cut -d" " -f1)" "$f"; done\n' % (rd, pat))
    out = {}
    for line in ssh(script).splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) == 2 and len(parts[0]) == 64:
            out[parts[1]] = parts[0]
    return out


def compare():
    """回傳 [(本機目錄, 遠端目錄, only_local, only_remote, differ)]"""
    result = []
    for ld, rd, exts in PAIRS:
        L, R = local_hashes(ld, exts), remote_hashes(rd, exts)
        only_l = sorted(set(L) - set(R))
        only_r = sorted(set(R) - set(L))
        differ = sorted(k for k in set(L) & set(R) if L[k] != R[k])
        result.append((ld, rd, only_l, only_r, differ))
    return result


def cmd_check(_args):
    bad = 0
    for ld, rd, only_l, only_r, differ in compare():
        print("\n=== %s  <->  %s ===" % (ld.name, rd))
        if not (only_l or only_r or differ):
            print("  完全一致")
            continue
        bad += 1
        for f in differ:
            print("  ≠ 內容不同      %s   %s" % (f, RESTART_HINT.get(f, "")))
        for f in only_l:
            print("  → 只有 repo 有  %s   (push 會送上去)" % f)
        for f in only_r:
            print("  ← 只有 Jetson 有 %s   (pull 會抓回來,建議納入版控)" % f)
    print()
    if bad:
        print("★ 有差異。push 之前先想清楚方向:")
        print("   repo 是正本 -> python sync_jetson.py push")
        print("   Jetson 上有手改過的 -> 先 pull 存進 repo,再決定")
    else:
        print("★ 兩邊一致")
    return 1 if bad else 0


def cmd_pull(_args):
    for ld, rd, only_l, only_r, differ in compare():
        ld.mkdir(parents=True, exist_ok=True)
        for f in sorted(set(only_r) | set(differ)):
            scp("%s@%s:%s/%s" % (USER, HOST, rd, f), ld / f)
            print("  ← %s" % f)
    print("\n完成。用 git diff 看 Jetson 上被改了什麼。")
    return 0


def cmd_push(args):
    changed = []
    for ld, rd, only_l, only_r, differ in compare():
        for f in sorted(set(only_l) | set(differ)):
            scp(ld / f, "%s@%s:%s/" % (USER, HOST, rd))
            print("  → %s" % f)
            changed.append(f)
    if not changed:
        print("  沒有要傳的")
        return 0

    # Windows 的 CRLF 會讓 bash 在 \r 上炸掉,一律轉掉
    ssh("cd ~/slam2d && sed -i 's/\\r$//' *.sh 2>/dev/null; "
        "cd ~/nav2 && sed -i 's/\\r$//' *.sh 2>/dev/null; true")

    hints = sorted({RESTART_HINT[f] for f in changed if f in RESTART_HINT})
    print("\n★ 傳上去 ≠ 生效。行程是啟動時讀設定的,要重啟才會用新的:")
    for h in hints or ["(這批檔案沒有對應的重啟指令)"]:
        print("   %s" % h)
    if args.restart and hints:
        print("\n=== 執行重啟 ===")
        for h in hints:
            print("--- %s ---" % h)
            # setsid:脫離 SSH session。不這樣做的話 plink 斷線時
            # systemd 會把 --scope 起的行程一起收掉,看起來重啟了其實沒有。
            print(ssh("source /opt/ros/humble/setup.bash\n"
                      "source ~/ws_livox/install/setup.bash 2>/dev/null\n"
                      "source ~/chassis_ws/install/setup.bash 2>/dev/null\n"
                      "export ROS_DOMAIN_ID=0\n"
                      "setsid %s < /dev/null 2>&1 | tail -25\n" % h,
                      timeout=400))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("check", help="比對兩邊,不改任何東西(預設)")
    sub.add_parser("pull", help="把 Jetson 上的檔抓回 repo")
    p = sub.add_parser("push", help="把 repo 推上 Jetson")
    p.add_argument("--restart", action="store_true", help="推完順便重啟受影響的行程")
    args = ap.parse_args()
    fn = {"pull": cmd_pull, "push": cmd_push}.get(args.cmd, cmd_check)
    try:
        return fn(args)
    except Exception as e:
        print("✗ %s" % e)
        return 2


if __name__ == "__main__":
    sys.exit(main())
