const int MOTOR_A_FW = 11; //A1 to D12, this is motor A forwards
const int MOTOR_A_BW = 10; //A2 to D11, this is motor A backwards
const int MOTOR_B_BW = 9; //B1 to D10, this is motor B backwards
const int MOTOR_B_FW = 6; //B2 to D9, this is motor B forwards

const int TURN_TIME = 558; //a 90° turn takes roughly 775ms (=0.775s)
const int SPEED_A_FW = 140; //left motor speed
const int SPEED_B_FW = 255; //right motor speed
const int SPEED_A_BW = 250;
const int SPEED_B_BW = 255;

void setup() {
  pinMode(MOTOR_A_FW, OUTPUT);
  pinMode(MOTOR_A_BW, OUTPUT);
  pinMode(MOTOR_B_BW, OUTPUT);
  pinMode(MOTOR_B_FW, OUTPUT);
}

void loop() {
  moveForwards();
  moveBackwards();
  turnRight90Deg(TURN_TIME);
  turnLeft90Deg(TURN_TIME);
}

void moveForwards() {
  digitalWrite(MOTOR_A_FW, HIGH); //motor A forwards ENABLED
  digitalWrite(MOTOR_A_BW, LOW); //motor A backwards DISABLED
  digitalWrite(MOTOR_B_BW, LOW); //motor B backwards DISABLED
  digitalWrite(MOTOR_B_FW, HIGH); //motor B forwards ENABLED

  analogWrite(MOTOR_A_FW, SPEED_A_FW);
  analogWrite(MOTOR_B_FW, SPEED_B_FW);

  delay(3000);
  stop(); //motors turn off
}

void moveBackwards() {
  digitalWrite(MOTOR_A_FW, LOW); //motor A forwards DISABLED
  digitalWrite(MOTOR_A_BW, HIGH); //motor A backwards ENABLED
  digitalWrite(MOTOR_B_BW, HIGH); //motor B backwards ENABLED
  digitalWrite(MOTOR_B_FW, LOW); //motor B forwards DISABLED

  analogWrite(MOTOR_A_BW, SPEED_A_BW);
  analogWrite(MOTOR_B_BW, SPEED_B_BW);

  delay(3000); //the movement happens for 3 seconds
  stop(); //motors turn off
}

void turnRight90Deg(int TURN_TIME) {
  digitalWrite(MOTOR_A_FW, HIGH); //motor A forwards ENABLED
  digitalWrite(MOTOR_A_BW, LOW); //motor A backwards DISABLED
  digitalWrite(MOTOR_B_BW, HIGH); //motor B backwards ENABLED
  digitalWrite(MOTOR_B_FW, LOW); //motor B forwards DISABLED
  delay(TURN_TIME); //the movement happens for 3 seconds
  stop(); //motors turn off
}

void turnLeft90Deg(int TURN_TIME) {
  digitalWrite(MOTOR_A_FW, LOW); //motor A forwards DISABLED
  digitalWrite(MOTOR_A_BW, HIGH); //motor A backwards ENABLED
  digitalWrite(MOTOR_B_BW, LOW); //motor B backwards DISABLED
  digitalWrite(MOTOR_B_FW, HIGH); //motor B forwards ENABLED
  delay(TURN_TIME); //the movement happens for 3 seconds
  stop(); //motors turn off
}

void stop() {
  digitalWrite(MOTOR_A_FW, LOW);
  digitalWrite(MOTOR_A_BW, LOW);
  digitalWrite(MOTOR_B_BW, LOW);
  digitalWrite(MOTOR_B_FW, LOW);
  delay(500);
}