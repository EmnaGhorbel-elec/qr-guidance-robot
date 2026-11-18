#!/usr/bin/env bash
# Run ONCE on the Pi, OUTSIDE Docker:   sudo bash install_boot.sh <container-name>
# It makes the robot start by itself every time the Pi gets power.
set -e
C="$1"
if [ -z "$C" ]; then
  echo "usage: sudo bash install_boot.sh <container-name>"
  echo "your containers:"; docker ps -a --format '  {{.Names}}   ({{.Image}})'
  exit 1
fi
docker inspect "$C" >/dev/null

PRIV=$(docker inspect -f '{{.HostConfig.Privileged}}' "$C")
DEVS=$(docker inspect -f '{{range .HostConfig.Devices}}{{.PathOnHost}} {{end}}' "$C")
NET=$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$C")
echo "container $C: privileged=$PRIV devices=[${DEVS}] network=$NET"
if [ "$PRIV" != "true" ] && [ -n "$DEVS" ]; then
  echo "WARNING: the container maps fixed devices. If the Mega or camera is not enumerated when"
  echo "         Docker starts it at power-on, the container FAILS to start. Recreate it with"
  echo "         --privileged -v /dev:/dev  (no --device flags)."
fi
[ "$NET" = "host" ] || echo "NOTE: network is not 'host': the PC may not see the robot's ROS topics."

docker start "$C" >/dev/null
docker exec "$C" test -f /ws/src/robot_hardware/boot/start_robot.sh || {
  echo "ERROR: /ws/src/robot_hardware/boot/start_robot.sh not found in the container."
  echo "       Copy the new robot_hardware to ~/projet_be/src and check that it is mounted at /ws."
  exit 1; }
docker exec "$C" test -f /ws/install/setup.bash || { echo "ERROR: /ws is not built yet (colcon build)"; exit 1; }

docker update --restart unless-stopped "$C" >/dev/null

cat > /etc/systemd/system/robot.service << UNIT
[Unit]
Description=QR robot (ROS 2 in Docker container $C)
After=docker.service
Requires=docker.service

[Service]
Type=simple
ExecStartPre=-/usr/bin/docker start $C
ExecStart=/usr/bin/docker exec $C bash /ws/src/robot_hardware/boot/start_robot.sh
ExecStop=/usr/bin/docker exec $C bash /ws/src/robot_hardware/boot/start_robot.sh stop
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
UNIT

systemctl daemon-reload
systemctl enable robot.service
echo
echo "Installed. It starts at the next power-on. NOT started now (the robot would drive)."
echo "  test now : sudo systemctl start robot      (watch: journalctl -u robot -f)"
echo "  stop     : sudo systemctl stop robot"
echo "  disable  : sudo systemctl disable robot"
