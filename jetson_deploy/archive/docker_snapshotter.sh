#!/usr/bin/env bash
# docker pull stalls on nvcr.io while curl gets 8.86 MB/s from the same
# registry. Docker 29.7.1 runs the containerd snapshotter by default; that
# path is the prime suspect for the stall on 307-redirecting registries.
# Turn it off and retest. One config line, fully reversible.
PW=2919
sudo_() { echo "$PW" | sudo -S -p "" "$@"; }

echo "=== 目前設定 ==="
docker info 2>/dev/null | grep -iE "storage driver|driver-type" | sed 's/^/  /'

sudo_ cp /etc/docker/daemon.json /etc/docker/daemon.json.snapshotter.bak
cat > /tmp/daemon.json <<'JSON'
{
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    },
    "default-runtime": "nvidia",
    "data-root": "/mnt/ssd/docker",
    "features": {
        "containerd-snapshotter": false
    }
}
JSON
sudo_ cp /tmp/daemon.json /etc/docker/daemon.json
sudo_ systemctl restart docker
sleep 12

echo
echo "=== 改後設定 ==="
docker info 2>/dev/null | grep -iE "storage driver|driver-type|Server Version" | sed 's/^/  /'

echo
echo "=== 重試 pull(90 秒觀察是否開始下載) ==="
timeout 90 docker pull nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c 2>&1 \
  | grep -iE "Downloading|Extracting|Download complete|Pull complete|error|denied" \
  | tail -12 | sed 's/^/  /'

echo
echo "=== 有沒有實際寫入 ==="
df -h /mnt/ssd | tail -1 | sed 's/^/  /'
sudo_ du -sh /mnt/ssd/docker 2>/dev/null | sed 's/^/  docker: /'
