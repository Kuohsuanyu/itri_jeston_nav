#!/usr/bin/env bash
# Can we actually download a layer blob? Isolates "registry serves data"
# from "docker can pull". The manifest already fetches fine anonymously,
# so auth is ruled out.
REPO=nvidia/isaac/ros
TAG=aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c

TOK=$(curl -sS "https://nvcr.io/proxy_auth?scope=repository:${REPO}:pull" \
      | sed -n 's/.*"token":"\([^"]*\)".*/\1/p')
[ -z "$TOK" ] && { echo "no token"; exit 1; }

echo "=== manifest 裡的分層大小 ==="
curl -sS -H "Authorization: Bearer $TOK" \
  -H "Accept: application/vnd.docker.distribution.manifest.v2+json" \
  "https://nvcr.io/v2/${REPO}/manifests/${TAG}" > /tmp/manifest.json
python3 - <<'PY'
import json
m = json.load(open("/tmp/manifest.json"))
ls = m.get("layers", [])
tot = sum(l["size"] for l in ls)
print("  分層數 %d,總大小 %.2f GB" % (len(ls), tot/1e9))
for l in sorted(ls, key=lambda x: -x["size"])[:5]:
    print("    %8.1f MB  %s" % (l["size"]/1e6, l["digest"][:24]))
open("/tmp/first_blob", "w").write(ls[0]["digest"])
PY

DIG=$(cat /tmp/first_blob)
echo
echo "=== 直接下載第一層的前 50MB,量速度 ==="
timeout 90 curl -sS -H "Authorization: Bearer $TOK" \
  -H "Range: bytes=0-52428799" \
  -o /tmp/blob.part -w "  HTTP %{http_code}  下載 %{size_download} bytes  平均 %{speed_download} B/s  總時間 %{time_total}s\n" \
  "https://nvcr.io/v2/${REPO}/blobs/${DIG}"
ls -la /tmp/blob.part 2>/dev/null | sed 's/^/  /'
rm -f /tmp/blob.part

echo
echo "=== docker 版本與 snapshotter 設定 ==="
docker version --format '  client {{.Client.Version}} / server {{.Server.Version}}' 2>/dev/null
docker info 2>/dev/null | grep -iE "storage driver|snapshotter|buildkit" | sed 's/^/  /'
