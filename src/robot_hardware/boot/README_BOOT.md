# Power-on start

Goal: plug in the battery, and the Pi boots, starts the robot by itself, and the autonomous run starts.

## What happens at power-on
| Time | What |
|---|---|
| 0 s | Pi boots, Docker starts the container, `robot.service` runs `start_robot.sh` |
| +30-60 s | ROS launches. The bridge opens the Mega (it keeps retrying if the USB is late) |
| +3 s | The Mega has rebooted. The IMU streams. The gyro calibrates for 2 s (robot must be STILL) |
| then | `/robot/ready` = true: log says `robot ready` |
| +5 s | `autonomy_node` starts by itself (`start_delay` in `qr_autonomy/config/autonomy_hw.yaml`): servo to 90, sweep, first command QR |

The robot never moves before `/robot/ready`. If the Mega or the IMU is missing, the robot stays
still and the log says why (`journalctl -u robot -f`). If the Mega drops out later (brownout, cable)
the wheels stop, the bridge reconnects, the gyro recalibrates, and the run restarts from a sweep (the ids already executed are remembered).

## Install (once)
PC: copy the new `robot_hardware` and `qr_autonomy` (and delete the old `qr_navigation`) to the Pi (`scp -r ... ~/projet_be/src/`),
then on the Pi outside Docker: `rm -rf ~/projet_be/build ~/projet_be/install ~/projet_be/src/qr_navigation`.

Pi, inside the container: rebuild.
```bash
cd /ws && colcon build --symlink-install --packages-select qr_autonomy robot_hardware
```
The Arduino firmware changed (servo range 0-180): re-flash `robot_firmware.ino` once.

Pi, outside Docker: find the container name, then install the service.
```bash
docker ps -a --format '{{.Names}}'
sudo bash ~/projet_be/src/robot_hardware/boot/install_boot.sh <container-name>
```
The script checks the container, sets `--restart unless-stopped`, writes `/etc/systemd/system/robot.service`
and enables it. It does NOT start the robot.

## Test
1. Wheels off the ground. `sudo systemctl start robot`, then `journalctl -u robot -f`.
   Expect: `Mega connected`, `IMU streaming`, `gyro calibrated`, `robot ready`, `INIT -> SWEEP`,
   then `QR C01 found ...` once a command QR is in view.
2. Then pull the power and plug it in again: the same thing must happen by itself.

## Daily use
- Stop it: `sudo systemctl stop robot`. Do this BEFORE any manual `ros2 launch`: two launches
  at the same time fight over the serial port and the wheels.
- Disable auto-start: `sudo systemctl disable robot`.
- Keep the robot still for 3 s after power-on (gyro calibration), wherever you like:
  there is no start pose any more, the robot only reacts to the command QR it sees.
- When the run ends (`DONE` after a STOP QR, or `FAILED`) the robot stops and waits. Power-cycle it, or
  `ros2 topic pub --once /mission/stop std_msgs/msg/Empty "{}"` to stop it earlier.

## If it does not start
- `systemctl status robot` and `journalctl -u robot -n 100`.
- Container fails to start at boot: it was created with `--device /dev/...` and the device was
  not there yet. Recreate it with `--privileged -v /dev:/dev` (and `--network host`, `-v ~/projet_be:/ws`).
- `waiting for /robot/ready` forever: the gyro keeps restarting (robot is moving) or `no IMU data`
  (wiring, see README_HW.md).
