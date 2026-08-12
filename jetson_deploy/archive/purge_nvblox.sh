#!/usr/bin/env bash
# Remove everything nvblox / Isaac ROS. ASCII only (base64|bash mangles CJK).
#
#   bash purge_nvblox.sh            full purge, images included
#   bash purge_nvblox.sh --dry-run  report only, delete nothing
#   bash purge_nvblox.sh --keep-images   remove workspace + containers, keep images
#
# Tier 3 (images) is the one that costs real time to undo: rebuilding the
# Isaac ROS 3.2 image took most of a day. Everything else is minutes.

MODE="${1:-full}"
DRY=0
KEEP_IMG=0
case "$MODE" in
    --dry-run)     DRY=1 ;;
    --keep-images) KEEP_IMG=1 ;;
esac

run() {
    if [ "$DRY" = "1" ]; then echo "    [dry-run] $*"; else eval "$@"; fi
}

echo "============ before ============"
echo "-- disk --"
df -h / /mnt/ssd 2>/dev/null | sed 's/^/  /'
echo "-- docker --"
docker system df 2>/dev/null | sed 's/^/  /'
echo "-- memory --"
free -m | head -2 | sed 's/^/  /'

echo
echo "[1/4] containers"
for c in nvblox nvblox_mesh_web isaac_ros_dev-aarch64-container; do
    if docker inspect "$c" > /dev/null 2>&1; then
        echo "  removing container: $c"
        run "docker rm -f $c > /dev/null 2>&1"
    fi
done
# Anything left that was launched from an isaac image
LEFT=$(docker ps -aq --filter "ancestor=isaac_ros_dev-aarch64:nvblox-rs" 2>/dev/null)
[ -n "$LEFT" ] && run "docker rm -f $LEFT > /dev/null 2>&1"
sleep 2

echo
echo "[2/4] host-side launch scripts"
for f in ~/start_nvblox_rs.sh ~/start_nvblox.sh ~/start_mesh_web.sh ~/enter_isaac.sh; do
    [ -f "$f" ] && { echo "  removing $f"; run "rm -f $f"; }
done

echo
echo "[3/4] SSD workspace /mnt/ssd/ws"
if [ -d /mnt/ssd/ws ]; then
    du -sh /mnt/ssd/ws 2>/dev/null | sed 's/^/  size: /'
    # Keep a copy of the two files that took real debugging to get right, in
    # case the Isaac path is ever revisited. They are a few KB.
    if [ "$DRY" != "1" ]; then
        mkdir -p ~/isaac_archive
        cp -f /mnt/ssd/ws/fastdds_udp_only.xml ~/isaac_archive/ 2>/dev/null
        cp -f /mnt/ssd/ws/nvblox_d435.yaml     ~/isaac_archive/ 2>/dev/null
        cp -f /mnt/ssd/ws/nvblox_rs.launch.py  ~/isaac_archive/ 2>/dev/null
        cp -f /mnt/ssd/ws/mesh_server.py       ~/isaac_archive/ 2>/dev/null
        echo "  kept config copies in ~/isaac_archive (a few KB, saves rediscovering"
        echo "  the UDP-only profile and the queue-length fix)"
    fi
    run "rm -rf /mnt/ssd/ws"
else
    echo "  not present"
fi

echo
echo "[4/4] docker images"
if [ "$KEEP_IMG" = "1" ]; then
    echo "  --keep-images given, skipping"
else
    docker images --format '  {{.Repository}}:{{.Tag}}  {{.Size}}' 2>/dev/null \
        | grep -iE 'isaac|nvblox' | sed 's/^/ /'
    IDS=$(docker images -q --filter "reference=isaac_ros_dev-aarch64*" 2>/dev/null)
    IDS="$IDS $(docker images -q --filter 'reference=nvcr.io/nvidia/isaac*' 2>/dev/null)"
    IDS=$(echo $IDS | tr ' ' '\n' | sort -u | tr '\n' ' ')
    if [ -n "$(echo $IDS | tr -d ' ')" ]; then
        run "docker rmi -f $IDS > /dev/null 2>&1"
    else
        echo "  no matching images"
    fi
    run "docker builder prune -af > /dev/null 2>&1"
    run "docker system prune -af --volumes > /dev/null 2>&1"
fi

echo
echo "============ after ============"
echo "-- disk --"
df -h / /mnt/ssd 2>/dev/null | sed 's/^/  /'
echo "-- docker --"
docker system df 2>/dev/null | sed 's/^/  /'
docker images 2>/dev/null | sed 's/^/  /' | head -10
echo "-- memory --"
free -m | head -2 | sed 's/^/  /'
echo
echo "-- anything left referencing nvblox? --"
pgrep -af nvblox | sed 's/^/  /' || echo "  no processes"
ls -d /mnt/ssd/ws 2>/dev/null | sed 's/^/  LEFTOVER: /' || echo "  workspace gone"
