# qr_autonomy: the robot drives itself from commands written in QR codes

No table, no map, no known QR position. The only knowledge the robot has is the text of the QR it is looking at.

## The loop
```
power-on, gyro calibrated
  INIT      servo to 90 deg (camera straight ahead, aligned with the robot)
  SWEEP     camera: 90 deg RIGHT, then 90 deg LEFT, then back to 90 (stops at the first command QR)
  ALIGN     the servo angle on the QR = its bearing: the robot turns on the spot to face it (gyro)
  APPROACH  drives toward it, the camera keeps tracking it (also while the robot turns), stops at 0.8 m
  READ      stands still, reads the text
  EXECUTE   TURN_L / TURN_R / MOVE / WAIT / FOLLOW / STOP
  back to SWEEP ... until a STOP QR (DONE) or an error (FAILED)
Nothing in view after a sweep: turn 90 deg on the spot, sweep again (4 times = full turn, then FAILED).
QR lost for 3 s while turning/driving toward it: STOP, sweep again. The robot never drives blind.
```
Nodes and topics (unchanged names where they existed):
`qr_detector_node` /qr/detections -> `scan_controller_node` (sweep, lock, /scan/lock, /scan/state, /servo/command)
-> `autonomy_node` -> /cmd_vel_raw -> `motor_node` -> /cmd_vel -> `arduino_bridge_node` -> Mega (9600 baud, M/G lines, I: IMU lines).
`autonomy_node` is the only writer of /cmd_vel_raw (command_node is no longer launched).

## Command QR format: `<id>|<ACTION>|<value>`
| Text | Meaning |
|---|---|
| `C02|TURN_L|60` | turn 60 deg left on the spot (gyro, 1..180) |
| `C04|TURN_R|90` | turn 90 deg right |
| `C03|MOVE|1.0` | drive 1.0 m straight (0.05..5, open loop, see limits) |
| `C06|WAIT|3` | stand still 3 s |
| `C01|FOLLOW` | no action: a stepping stone, the robot goes there then looks for the next QR |
| `C07|STOP` | end of the run |
The id (`C02`) must be UNIQUE: executed ids are remembered and ignored afterwards. Chaining `C08|TURN_L|60|MOVE|1.0` also works.
An invalid text (`C09|DANCE|3`) is logged and skipped. Print: `python3 tools/make_command_qrs.py tools/example_commands.txt out/`
(PNG per QR + `sheet.pdf`, one A4 page each at 15 cm, every text is decoded back before it is written).

## First run, in order
1. Re-flash `robot_firmware.ino` (servo range is now 0-180). Rebuild from clean: `rm -rf build install log && colcon build --symlink-install`.
2. Check README_HW.md steps 1-5 again (they still hold): baud, gyro, servo sign, wheels, turn direction.
3. **Measure `servo_k`**: QR at exactly 0.8 m from the camera, `ros2 topic echo /qr/detections`, `k = w_px x 0.8`, put it in `config/autonomy_hw.yaml` (`servo_k`). With a wrong k the robot stops at the wrong distance. Use the SAME printed size as in the room.
4. **QR size and range**: OpenCV decoded the example QR only down to about 100 px wide on a 640x480 image. The robot can therefore only discover a QR closer than `k / 100` metres: about 0.8 m for a 15 cm QR, about 1.4 m for 25 cm, 1.7 m for 30 cm. Print them big (25 cm), keep the text short, and place the next QR within that range of where the previous command leaves the robot.
5. Teleop check of the servo: `G0`, `G90`, `G180` on /servo/command (buzzing at an end stop: `sweep_extent: 85.0`).
6. Wheels off the ground: `ros2 launch robot_hardware robot_hw.launch.py`, put a `C07|STOP` QR in front, then `ros2 topic pub --once /mission/start std_msgs/msg/Empty "{}"`. Watch `ros2 topic echo /autonomy/status` and `/scan/state`.
7. On the floor, one QR (`C01|FOLLOW`), then two, then a real course. `autostart:=true` starts by itself after `start_delay`.

## Tuning (config/autonomy_hw.yaml)
| Symptom | Parameter |
|---|---|
| stops too close / too far from the QR | `servo_k` (measure), then `standoff` |
| turns overshoot or hunt | lower `k_turn`, raise `turn_tol` |
| does not rotate (`stall` warning) | raise `w_min` and `min_pwm` in the bridge |
| `MOVE 1.0` goes 0.8 or 1.2 m | `max_speed` in the bridge (README_HW.md step 6), `odom_linear_scale` |
| sweep misses a QR | `sweep_dwell` up (0.6), `sweep_step` down (8) |
| QR seen too late / never | bigger QR, `min_width_px`, better light (decoding is the limit) |

## Limits to know
- Distance is dead reckoning (no encoders): `MOVE` is accurate to roughly 10 %, drift is not corrected because there is no map. Turns use the gyro and are good to a few degrees.
- No obstacle sensor: MOVE and APPROACH do not detect walls. Keep the course clear.
- The first QR seen during the sweep wins (right to left), not the nearest. Ids already executed are skipped.
- A power glitch (`/robot/ready` false) pauses the run, then it restarts from a sweep. Executed ids are kept, a half-executed command is dropped.
- Tests: `PYTHONPATH=qr_autonomy:robot_hardware:robot_control python3 -m pytest qr_autonomy/test robot_hardware/test` (the closed-loop room simulation in `test_navigator_sim.py` runs the real navigator without ROS).
