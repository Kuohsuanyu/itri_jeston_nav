#!/usr/bin/env bash
# Measure the real pull throughput from nvcr.io.
# The image is 12.59 GB / 94 layers, so this number decides whether the
# container route is viable at all.
REPO=nvidia/isaac/ros
TAG=aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c
OUT=/mnt/ssd/data/blob.part

TOK=$(curl -sS "https://nvcr.io/proxy_auth?scope=repository:${REPO}:pull" \
      | sed -n 's/.*"token":"\([^"]*\)".*/\1/p')
DIG=$(python3 -c "import json;print(json.load(open('/tmp/manifest.json'))['layers'][0]['digest'])")

echo "=== 跟隨重導向,下載前 150MB ==="
timeout 240 curl -sSL -H "Authorization: Bearer $TOK" \
  -H "Range: bytes=0-157286399" \
  -o "$OUT" \
  -w "  HTTP %{http_code}\n  下載 %{size_download} bytes\n  平均速度 %{speed_download} B/s\n  總時間 %{time_total} s\n" \
  "https://nvcr.io/v2/${REPO}/blobs/${DIG}"

SZ=$(stat -c%s "$OUT" 2>/dev/null || echo 0)
T=$(cat /tmp/.dlt 2>/dev/null || echo 0)
truncate -s 0 "$OUT" 2>/dev/null
unlink "$OUT" 2>/dev/null

echo
if [ "$SZ" -gt 1000000 ]; then
    python3 - "$SZ" <<'PY'
import sys
sz = int(sys.argv[1])
print("  實際下載 %.1f MB" % (sz / 1e6))
PY
else
    echo "  下載量太少($SZ bytes),連線有問題"
fi

echo
echo "=== 對照:同時量 WiFi 實際吞吐(下載一個大檔) ==="
timeout 60 curl -sSL -o /dev/null \
  -w "  speedtest 參考: %{speed_download} B/s\n" \
  "https://speed.cloudflare.com/__down?bytes=100000000" 2>/dev/null \
  || echo "  (無法連 speed.cloudflare.com)"

echo
echo "=== WiFi 連線品質 ==="
iw dev wlP1p1s0 link 2>/dev/null | grep -E "signal|bitrate|SSID" | sed 's/^/  /'
