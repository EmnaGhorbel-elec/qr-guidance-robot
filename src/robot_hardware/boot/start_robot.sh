#!/usr/bin/env bash
# Runs INSIDE the ROS container. Called by the systemd service (see install_boot.sh).
#   bash start_robot.sh         start the robot: camera, servo, IMU, motors, QR-command autonomy
#   bash start_robot.sh stop    stop it cleanly (the wheels get a stop command)
PIDFILE=/tmp/robot_launch.pid

running_pid() {      # pid of a launch started by this script, if it is still alive
  local pid
  pid=$(cat "$PIDFILE" 2>/dev/null) || return 1
  [ -n "$pid" ] && grep -q 'ros2' "/proc/$pid/cmdline" 2>/dev/null && echo "$pid"
}

if [ "$1" = "stop" ]; then
  pid=$(running_pid) && kill -INT "$pid"
  exit 0
fi

# a previous launch still alive (service restarted): stop it first, two robots = fight
if pid=$(running_pid); then
  kill -INT "$pid"
  sleep 4
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
set +u
source /opt/ros/humble/setup.bash
source "${WS:-/ws}/install/setup.bash"

# At power-on the USB devices can take a while to appear. Wait up to 40 s for the Mega and the
# camera, then go on anyway (the bridge keeps retrying the Mega by itself).
CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
for _ in $(seq 40); do
  if ls /dev/ttyACM* /dev/ttyUSB* >/dev/null 2>&1 && [ -e "$CAMERA_DEV" ]; then break; fi
  sleep 1
done
ls /dev/ttyACM* /dev/ttyUSB* >/dev/null 2>&1 || echo "start_robot: no Mega found yet, the bridge will keep trying"
[ -e "$CAMERA_DEV" ] || echo "start_robot: no camera at $CAMERA_DEV, starting without it"

echo $$ > "$PIDFILE"
exec ros2 launch robot_hardware robot_hw.launch.py mode:=auto autostart:=true port:=auto
