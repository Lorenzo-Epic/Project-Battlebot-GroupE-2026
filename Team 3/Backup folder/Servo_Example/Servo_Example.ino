#define SERVO 10 ///Pin 12
#define GRIPPER_OPEN 1820
#define GRIPPER_CLOSE 1000

void setup() {
  pinMode(SERVO, OUTPUT);
  digitalWrite(SERVO, 0);
}

void loop() {
  for (int i = 0; i < 1000; i++) {
    gripper(GRIPPER_OPEN);
    delay(1);
  }
  for (int i = 0; i < 1000; i++) {
    gripper(GRIPPER_CLOSE);
    delay(1);
  }

}

void gripper(int newPulse) {
  static unsigned long timer;
  static int pulse;
  if (millis() > timer) {
    if (newPulse > 0) {
      pulse = newPulse;
    }
    digitalWrite(SERVO, 1); ///HIGH
    delayMicroseconds(pulse);
    digitalWrite(SERVO, 0);
    timer = timer + 20;
  }
}
