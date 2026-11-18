// robot_firmware.ino  --  Arduino Mega: 4 DC motors (2 x L298N, one per AXLE) + camera servo
// + MPU-6050 IMU, driven by the Pi over USB serial.
//
// Commands (one per line, '\n' terminated), Pi -> Mega:
//   M<l>,<r>   LEFT side and RIGHT side PWM, -255..255      e.g. "M150,-150"
//   X          stop all motors
//   S / H      servo sweep start / halt (legacy: the Pi now sweeps with G commands)
//   G<angle>   servo to a fixed angle (also halts the sweep)   e.g. "G90"
// Mega -> Pi:
//   A:<angle>  servo moved to <angle> (sweep, and echo of G)
//   OK:SCAN / OK:HALT / OK:STOP
//   E:WATCHDOG motors stopped because no M command arrived in time
//   I:<ms>,<gz>   MPU-6050 gyro Z raw counts, 25 Hz (millis() timestamp)
//                 (short on purpose: the link is 9600 baud = 960 B/s, the old 7-field line at 50 Hz needed 2300 B/s)
//   OK:IMU / E:IMU   IMU found / not found at start
//
// WIRING
//   4 motor channels: CH0 = pins 5/22/23, CH1 = 6/24/25, CH2 = 7/26/27, CH3 = 8/28/29.
//   Which wheel each channel drives, and its direction, is set in the WIRING TABLE below.
//   Find the values with firmware/motor_test/motor_test.ino (wheels off the ground).
//   Remove the ENA / ENB jumpers on both L298N boards (the Mega drives them with PWM).
//   Servo signal -> pin 9 (servo power from its own 5V buck converter, common GND)
//   MPU-6050 (HW-123 / GY-521): VCC->5V, GND->GND, SDA->pin 20, SCL->pin 21, AD0->GND

#include <Servo.h>
#include <Wire.h>

// ---- Pins ----
const int SERVO_PIN = 9;

const int ENA_1 = 5, IN1_1 = 22, IN2_1 = 23;   // driver 1 (front) channel A: LEFT
const int ENB_1 = 6, IN3_1 = 24, IN4_1 = 25;   // driver 1 (front) channel B: RIGHT
const int ENA_2 = 7, IN1_2 = 26, IN2_2 = 27;   // driver 2 (rear)  channel A: LEFT
const int ENB_2 = 8, IN3_2 = 28, IN4_2 = 29;   // driver 2 (rear)  channel B: RIGHT

// ---- WIRING TABLE: fill it from motor_test.ino ----
// side: LEFT or RIGHT wheel(s) of the robot.
// pol : +1 if the wheel turned FORWARD in motor_test, -1 if it turned BACKWARD.
// The defaults below are the old fixed wiring (CH0, CH2 = left; CH1, CH3 = right; all +1).
// After editing this table, set swap_ab / invert_a / invert_b back to false on the Pi.
enum Side { LEFT = 0, RIGHT = 1 };
struct Channel { int en, inA, inB; Side side; int pol; };
Channel CH[4] = {
  //  EN     IN       IN      side    pol
  {ENA_1, IN1_1, IN2_1, RIGHT,  -1},   // CH0: FR, backward
  {ENB_1, IN3_1, IN4_1, LEFT,   +1},   // CH1: FL, forward
  {ENA_2, IN1_2, IN2_2, LEFT,   -1},   // CH2: RL, backward
  {ENB_2, IN3_2, IN4_2, RIGHT,  +1},   // CH3: RR, forward
};

// ---- Servo range ----
// Full +-90 deg around the straight-ahead position (90). The search sweep itself is now driven
// by the Pi with G<angle> commands (scan_controller_node), so S / H below are no longer used.
// If the servo buzzes at an end stop, reduce `sweep_extent` on the Pi (e.g. 85), no re-flash needed.
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const int STEP_DEG  = 5;
const unsigned long DWELL_MS = 700;

// ---- IMU (MPU-6050) ----
const uint8_t MPU_ADDR = 0x68;                // AD0 low
const unsigned long IMU_PERIOD_MS = 40;       // 25 Hz
bool imuOk = false;
unsigned long lastImu = 0;

// ---- Safety ----
const unsigned long MOTOR_TIMEOUT_MS = 500;   // no M command for this long -> motors off

Servo servo;
int angle = 90;
int dir = 1;
bool scanning = false;
unsigned long lastMove = 0;

unsigned long lastMotorCmd = 0;
bool motorsRunning = false;

char buf[32];
uint8_t len = 0;

// One motor channel. speed: -255 (full reverse) .. 255 (full forward)
void moteur(int enablePin, int inA, int inB, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(inA, speed > 0);
  digitalWrite(inB, speed < 0);
  analogWrite(enablePin, abs(speed));
}

// Left/right side speeds -> the 4 channels, using the WIRING TABLE
void cote(int left, int right) {
  for (uint8_t i = 0; i < 4; i++) {
    int s = (CH[i].side == LEFT) ? left : right;
    moteur(CH[i].en, CH[i].inA, CH[i].inB, CH[i].pol * s);
  }
}

void stopMotors() {
  cote(0, 0);
  motorsRunning = false;
}

bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool imuBegin() {
  Wire.begin();
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);            // never hang the motors if the sensor stalls the bus
#endif
  bool ok = mpuWrite(0x6B, 0x01);             // wake up, clock = gyro X PLL
  ok = ok && mpuWrite(0x1A, 0x03);            // digital low-pass ~44 Hz
  ok = ok && mpuWrite(0x1B, 0x08);            // gyro +-500 deg/s  (65.5 LSB per deg/s)
  ok = ok && mpuWrite(0x1C, 0x00);            // accel +-2 g       (16384 LSB per g)
  return ok;
}

int16_t read16() {                            // explicit order: C++ does not define it for a | b
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

void imuPoll() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);                           // GYRO_ZOUT_H: only the yaw rate is used by the Pi
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)2) != 2) return;
  int16_t gz = read16();
  Serial.print("I:");
  Serial.print(millis());  Serial.print(',');
  Serial.println(gz);
}

void handleCommand(char* cmd) {
  switch (cmd[0]) {
    case 'M': {
      char* comma = strchr(cmd + 1, ',');
      if (comma == NULL) return;                  // malformed: ignore
      int l = atoi(cmd + 1);
      int r = atoi(comma + 1);
      cote(l, r);
      motorsRunning = (l != 0 || r != 0);
      lastMotorCmd = millis();
      break;
    }
    case 'X':
      stopMotors();
      Serial.println("OK:STOP");
      break;
    case 'S':
      scanning = true;
      Serial.println("OK:SCAN");
      break;
    case 'H':
      scanning = false;
      Serial.println("OK:HALT");
      break;
    case 'G':
      angle = constrain(atoi(cmd + 1), MIN_ANGLE, MAX_ANGLE);
      servo.write(angle);
      scanning = false;
      Serial.print("A:");                       // echo so the Pi knows where the camera points
      Serial.println(angle);
      break;
  }
}

void setup() {
  const int outputs[] = {ENA_1, IN1_1, IN2_1, ENB_1, IN3_1, IN4_1,
                         ENA_2, IN1_2, IN2_2, ENB_2, IN3_2, IN4_2};
  for (uint8_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++) pinMode(outputs[i], OUTPUT);
  stopMotors();                                   // motors off before anything else

  Serial.begin(9600);
  servo.attach(SERVO_PIN);
  servo.write(angle);

  imuOk = imuBegin();
  Serial.println(imuOk ? "OK:IMU" : "E:IMU");
}

void loop() {
  // --- read commands (non-blocking) ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[len] = '\0';
      if (len > 0) handleCommand(buf);
      len = 0;
    } else if (c != '\r') {
      if (len < sizeof(buf) - 1) buf[len++] = c;
      else len = 0;                               // line too long: drop it
    }
  }

  // --- watchdog: the Pi crashed, the cable fell out... stop the robot ---
  if (motorsRunning && millis() - lastMotorCmd > MOTOR_TIMEOUT_MS) {
    stopMotors();
    Serial.println("E:WATCHDOG");
  }

  // --- IMU at 25 Hz ---
  if (imuOk && millis() - lastImu >= IMU_PERIOD_MS) {
    lastImu = millis();
    imuPoll();
  }

  // --- servo sweep: step, then pause ---
  if (scanning && millis() - lastMove >= DWELL_MS) {
    lastMove = millis();
    angle += dir * STEP_DEG;
    if (angle >= MAX_ANGLE) { angle = MAX_ANGLE; dir = -1; }
    if (angle <= MIN_ANGLE) { angle = MIN_ANGLE; dir = 1; }
    servo.write(angle);
    Serial.print("A:");
    Serial.println(angle);
  }
}
