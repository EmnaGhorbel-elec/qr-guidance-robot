// motor_test.ino  --  find out which wheel each L298N channel drives, and in which direction.
//
// WHEELS OFF THE GROUND. Stop ROS first (the Pi must not hold the serial port).
// Upload, open the Serial Monitor at 9600 baud. Each channel turns alone for 3 s with a
// POSITIVE command, one after the other. For each line printed, write down:
//     which wheel turns (FL / FR / RL / RR)   and   FORWARD or BACKWARD.
// Send any character in the Serial Monitor to run the sequence again.
// Then put the results in the WIRING TABLE of robot_firmware.ino and re-upload that sketch.

struct Ch { const char* name; int en, inA, inB; };
const Ch CH[4] = {
  {"CH0  (pins EN 5, IN 22/23)", 5, 22, 23},
  {"CH1  (pins EN 6, IN 24/25)", 6, 24, 25},
  {"CH2  (pins EN 7, IN 26/27)", 7, 26, 27},
  {"CH3  (pins EN 8, IN 28/29)", 8, 28, 29},
};

void drive(const Ch& c, int speed) {          // same logic as robot_firmware.ino
  digitalWrite(c.inA, speed > 0);
  digitalWrite(c.inB, speed < 0);
  analogWrite(c.en, abs(speed));
}

void runSequence() {
  for (int i = 0; i < 4; i++) {
    Serial.print("NOW: ");
    Serial.print(CH[i].name);
    Serial.println("  -> which wheel? forward or backward?");
    drive(CH[i], 160);
    delay(3000);
    drive(CH[i], 0);
    delay(1500);
  }
  Serial.println("Done. Send any character to run again.");
}

void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(CH[i].en, OUTPUT); pinMode(CH[i].inA, OUTPUT); pinMode(CH[i].inB, OUTPUT);
    drive(CH[i], 0);
  }
  Serial.begin(9600);
  delay(2000);
  Serial.println("motor_test: wheels OFF the ground");
  runSequence();
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    runSequence();
  }
}
