#define SERVO 11 //servo pin, GR to D11
#define gripperOpenPulse 1600 //pulse width needed to open gripper
#define gripperClosePulse 1100 //pulse width needed to close gripper

#define MOTOR_A_FW 10 //A1 to D10, this is motor A forwards
#define MOTOR_A_BW 9 //A2 to D9, this is motor A backwards
#define MOTOR_B_BW 6 //B1 to D6, this is motor B backwards
#define MOTOR_B_FW 5 //B2 to D5, this is motor B forwards

#define SPEED_A_FW 249 //motor A forwards speed
#define SPEED_A_BW 253 //motor A backwards speed
#define SPEED_B_BW 255 //motor B backwards speed
#define SPEED_B_FW 255 //motor B forwards speed

#define FORWARDS_TIME 1200 //to move forwards for 25cm, it takes FORWARDS_TIME milliseconds

bool isFinished = false;

unsigned long startTime;

void setup() {
  //initialising motors as OUTPUTs
  pinMode(MOTOR_A_FW, OUTPUT);
  pinMode(MOTOR_A_BW, OUTPUT);
  pinMode(MOTOR_B_BW, OUTPUT);
  pinMode(MOTOR_B_FW, OUTPUT);

  //initial state of motors should be LOW
  digitalWrite(MOTOR_A_FW, LOW);
  digitalWrite(MOTOR_A_BW, LOW);
  digitalWrite(MOTOR_B_BW, LOW);
  digitalWrite(MOTOR_B_FW, LOW);

  //initialising servo as OUTPUT
  pinMode(SERVO, OUTPUT);

  //initial state of servo should be LOW
  digitalWrite(SERVO, LOW);

  startTime = millis();
}

void loop() {
  if (isFinished) { //if loop has already run once
    stop();
    gripper(gripperClosePulse);
    return;
  }

  unsigned long loopTime = millis() - startTime;

  if (loopTime < 1000) {
    gripper(gripperOpenPulse);
  }

  else if (loopTime < 2000) {
    gripper(gripperClosePulse);
  }

  else if (loopTime < 3000) {
    gripper(gripperOpenPulse);
  }

  else if (loopTime < 3000 + FORWARDS_TIME) {
    gripper(gripperOpenPulse);
    moveForwardsNoStop();
  }

  else if (loopTime < 4200 + FORWARDS_TIME) {
    stop();
    gripper(gripperClosePulse);
  }

  else if (loopTime < 4200 + FORWARDS_TIME * 2) {
    moveForwardsNoStop();
    gripper(gripperClosePulse);
  }

  else {
    stop();
    gripper(gripperClosePulse);
    isFinished = true;
  }
}

void moveForwardsNoStop() {
  analogWrite(MOTOR_A_BW, 0); //motor A backwards DISABLED
  analogWrite(MOTOR_B_BW, 0); //motor B backwards DISABLED

  analogWrite(MOTOR_A_FW, SPEED_A_FW);
  analogWrite(MOTOR_B_FW, SPEED_B_FW);
}

void stop() {
  analogWrite(MOTOR_A_FW, 0); //motor A forwards DISABLED
  analogWrite(MOTOR_B_FW, 0); //motor B forwards DISABLED
  analogWrite(MOTOR_B_BW, 0); //motor A backwards DISABLED
  analogWrite(MOTOR_A_BW, 0); //motor B backwards DISABLED
}

void gripper(int newPulse) {
  //static -> the value is not changing
  //unsigned -> the value stored is unsigned, therefore can only be positive, doubling the range of available values
  //long -> large value possible
  static unsigned long timer;
  static int oldPulse;
  
  if (millis() >= timer) { //has enough time passed for next pulse to be sent?
    if (newPulse > 0) { //always executing, because both possible newPulses are positive
      oldPulse = newPulse;
    }

    //set SERVO to HIGH -> wait oldPulse milliseconds -> set SERVO to LOW
    digitalWrite(SERVO, HIGH);
    delayMicroseconds(oldPulse);
    digitalWrite(SERVO, LOW);
    timer = millis() + 20; //wait 20 milliseconds before next pulse
  }
}