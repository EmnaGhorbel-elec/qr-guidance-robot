# robot_hardware: bring-up and calibration

## Wiring (Arduino Mega)
| Part | Connection |
|---|---|
| MPU-6050 (GY-521) | VCC to 5V, GND to GND, SDA to pin 20, SCL to pin 21, AD0 to GND |
| Servo signal | pin 9 |
| Servo power | its own 5V buck converter. Buck GND also to a Mega GND (common ground) |
| Motor channels | CH0: EN 5, IN 22/23. CH1: EN 6, IN 24/25. CH2: EN 7, IN 26/27. CH3: EN 8, IN 28/29. Which wheel each one drives (and its direction) is set in the WIRING TABLE of `robot_firmware.ino`, found with `motor_test.ino` |
| L298N jumpers | remove the ENA and ENB jumpers on both boards |

If on your boards channel A is the RIGHT wheel, set `swap_ab: true` in the bridge instead of rewiring.

Mount the IMU flat, Z axis up, fixed to the chassis. If the chip is upside down set `imu_yaw_sign: -1.0`.

## Test order (each step needs the previous one)
1. Flash `firmware/robot_firmware/robot_firmware.ino`. Serial Monitor **9600**: you should see `I:<ms>,<gz>` lines at 25 Hz (`E:IMU` = wiring problem).
   The whole link runs at 9600 baud (firmware `Serial.begin(9600)`, bridge parameter `baud`). The IMU line is short on purpose: the old 7-field line at 50 Hz needed 2300 B/s and 9600 baud carries 960 B/s.
2. `ros2 launch robot_hardware robot_hw.launch.py mode:=teleop` then keep the robot STILL for 3 s. The log says `gyro calibrated`.
3. Gyro check: `ros2 topic echo /odom --field pose.pose.orientation`, rotate the robot by hand 90 degrees to the left. z should go to about 0.7 and w to about 0.7.
4. Servo sign: `ros2 topic pub --once /servo/command std_msgs/msg/String "{data: 'G120'}"`. If the camera turns RIGHT set `servo_sign: -1.0` (scan_controller) in `qr_autonomy/config/autonomy_hw.yaml`.
5. Wheels off the ground: upload `firmware/motor_test/motor_test.ino` (ROS stopped), Serial Monitor 9600,
   note for CH0..CH3 which wheel turns and FORWARD/BACKWARD. Fill the WIRING TABLE at the top of
   `robot_firmware.ino` (side LEFT/RIGHT, pol +1 forward / -1 backward), re-upload `robot_firmware.ino`,
   and keep `swap_ab`, `invert_a`, `invert_b` all false. Then teleop on `/cmd_vel_raw` to confirm.
   Then ON THE FLOOR, the closed-loop check that teleop alone does not show:
   `ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/msg/Twist "{angular: {z: 0.8}}"` must turn the robot
   COUNTER-CLOCKWISE seen from above, and `ros2 topic echo /imu/data --field angular_velocity.z` must be
   POSITIVE. If the two disagree, the bridge stops the wheels and logs `TURN DIRECTION ERROR` with the
   fix (`imu_yaw_sign: -1.0` or `swap_ab: true`). If it logs `does not rotate`, raise `min_pwm`.
6. Measure `max_speed`: drive 2 s at a known PWM on the floor with a tape measure, then set `max_speed` (bridge) = distance / time at PWM 255 scaled.
7. Servo range: the Mega now allows 0-180 deg (re-flash `robot_firmware.ino` once). At power-on the camera
   points straight ahead (90). `ros2 topic pub --once /servo/command std_msgs/msg/String "{data: 'G0'}"` then `G180`:
   if it buzzes at an end stop, set `sweep_extent: 85.0` in `qr_autonomy/config/autonomy_hw.yaml`.
8. Everything about the autonomous run (command QR, `servo_k`, first run) is in
   `../qr_autonomy/README_AUTONOMY.md`. There is no QR table any more.

## Safety checks in the nodes
| Log message | Node | Meaning |
|---|---|---|
| `TURN DIRECTION ERROR` | arduino_bridge_node | robot turns against angular.z: gyro sign or motor sides wrong. Wheels stopped until restart |
| `N publishers on cmd_vel / cmd_vel_raw / odom` | bridge / motor_node / autonomy_node | two launches, or a simulation on the same ROS_DOMAIN_ID. Robot stopped while it lasts |
| `Could not exclusively lock port` | arduino_bridge_node | another process holds the Mega (robot.service + manual launch) |

All three can be switched off with the parameters `turn_check` / `writer_guard` (not recommended).

## Power-on start
See `boot/README_BOOT.md`. Summary: `sudo bash boot/install_boot.sh <container>` once; after that the Pi
starts the whole robot at power-on and the autonomous run begins `start_delay` s after the gyro is calibrated
(`/robot/ready`, needs `autostart: true`). `sudo systemctl stop robot` before any manual launch.

Wiring needed for a safe power-on: a 10k pull-down from each of Mega pins 5, 6, 7, 8 to GND
(the L298N enable inputs float while the Mega boots and the motors can twitch).
