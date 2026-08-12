#!/usr/bin/env bash
# ASCII only.
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0

pkill -9 -f "python3 overlay_server.py" 2>/dev/null
sleep 1

cd ~/calib_view || exit 1
setsid nohup python3 overlay_server.py > /tmp/overlay.log 2>&1 < /dev/null &
sleep 12

pgrep -f "python3 overlay_server.py" > /dev/null \
  && echo "  overlay_server running" || { echo "  DEAD"; tail -20 /tmp/overlay.log; exit 1; }

curl -sS -o /dev/null -w "  /  HTTP %{http_code}\n" http://127.0.0.1:8094/ 2>/dev/null
echo "  state:"
curl -sS --max-time 3 http://127.0.0.1:8094/state.json 2>/dev/null | sed 's/^/    /'
echo
echo "  stats:"
curl -sS --max-time 3 http://127.0.0.1:8094/stats.json 2>/dev/null | sed 's/^/    /'
echo
echo "  open  http://192.168.40.98:8094/"
