#!/usr/bin/env bash
echo "=== 網路是否真的在下載(10 秒取樣) ==="
A=$(cat /sys/class/net/wlP1p1s0/statistics/rx_bytes)
D0=$(df -m /mnt/ssd | tail -1 | awk '{print $3}')
sleep 10
B=$(cat /sys/class/net/wlP1p1s0/statistics/rx_bytes)
D1=$(df -m /mnt/ssd | tail -1 | awk '{print $3}')
echo "  網路 rx : $(( (B-A)/10/1024 )) KB/s"
echo "  SSD 增加: $(( D1-D0 )) MB / 10s"

echo
echo "=== 誰在吃 CPU ==="
ps -eo pcpu,pmem,rss,comm --sort=-pcpu | head -10 | sed 's/^/  /'

echo
echo "=== dockerd 最近的訊息 ==="
echo 2919 | sudo -S -p "" journalctl -u docker --no-pager -n 25 2>/dev/null \
  | tail -20 | sed 's/^/  /'

echo
echo "=== docker root dir 實際內容(需要 root) ==="
echo 2919 | sudo -S -p "" du -sh /mnt/ssd/docker 2>/dev/null | sed 's/^/  /'
echo 2919 | sudo -S -p "" ls -1 /mnt/ssd/docker 2>/dev/null | head | sed 's/^/    /'
echo "  /var/lib/docker(舊位置,應該是空的):"
echo 2919 | sudo -S -p "" du -sh /var/lib/docker 2>/dev/null | sed 's/^/    /'

echo
echo "=== 能連到 nvcr.io 嗎 ==="
timeout 15 curl -sSI https://nvcr.io/v2/ 2>&1 | head -3 | sed 's/^/  /'
echo "  DNS: $(getent hosts nvcr.io | head -1)"

echo
echo "=== pull 行程的狀態 ==="
for p in $(pgrep -f "docker pull"); do
    echo "  pid $p state=$(ps -o stat= -p $p) etime=$(ps -o etime= -p $p)"
done
