const int MOTOR_A_FW = 10; //A1 to D10
const int MOTOR_A_BW = 9; //A2 to D9
const int MOTOR_B_BW = 6; //B1 to D6
const int MOTOR_B_FW = 5; //B2 to D5

const int TRIG_SENS = 7; //TRIGGER (input pin) is wired to D7
const int ECHO_SENS = 4; //ECHO (output pin) is wired to D4

const int SPEED_A_FW = 252; //motor A forwards speed
const int SPEED_A_BW = 253; //motor A backwards speed
const int SPEED_B_BW = 255; //motor B backwards speed
const int SPEED_B_FW = 255; //motor B forwards speed

const int SPEED_A_R = 255; //motor A right turn speed
const int SPEED_B_R = 0; //motor B right turn speed
const int SPEED_A_L = 0; //motor A left turn speed
const int SPEED_B_L = 240; //motor B left turn speed

//declaring variable to be used to store turning times, to make the code more flexible
int turnTime;

const float SOUND_SPEED = 0.0343; //on Earth, the speed of sound is 0.0343 microseconds/cm
//if needed, other sound speed variables may be declared

const int TURN_TIME = 800; //a 90° turn takes roughly 800ms
const int MOVE_FOR = 500; //movement time

const int OBJECT_DIST = 20; //distance in cm to trigger object avoidance
const int CLEAR_DIST = 30; //distance in cm to consider path clear

bool isAvoiding = false; //to determine if robot is already in object avoidance state

void setup() {
  pinMode(MOTOR_A_FW, OUTPUT);
  pinMode(MOTOR_A_BW, OUTPUT);
  pinMode(MOTOR_B_BW, OUTPUT);
  pinMode(MOTOR_B_FW, OUTPUT);

  digitalWrite(MOTOR_A_FW, LOW);
  digitalWrite(MOTOR_A_BW, LOW);
  digitalWrite(MOTOR_B_BW, LOW);
  digitalWrite(MOTOR_B_FW, LOW);
  
  pinMode(TRIG_SENS, OUTPUT);
  pinMode(ECHO_SENS, INPUT);

  digitalWrite(TRIG_SENS, LOW);
  digitalWrite(ECHO_SENS, LOW);
}

void loop() {
  long distance = getDistanceInCm();

  if (distance < OBJECT_DIST && !isAvoiding) { //if object is closer than OBJECT_DIST and robot is not already avoiding
    isAvoiding = true;
    stop();
    avoidObject();
  } else if (distance > CLEAR_DIST) { //else if distance is larger than CLEAR_DIST, avoiding state is disabled
    isAvoiding = false;
  }
  
  if (!isAvoiding) { //if robot is not avoiding then it should move forwards without stopping
    moveForwardsNoStop();
  }
  
  delay(50); //to stabilise sensor reading interval
}

void moveForwardsNoStop() {
  analogWrite(MOTOR_A_BW, 0); //motor A backwards DISABLED
  analogWrite(MOTOR_B_BW, 0); //motor B backwards DISABLED

  analogWrite(MOTOR_A_FW, SPEED_A_FW);
  analogWrite(MOTOR_B_FW, SPEED_B_FW);
}

void moveForwardsFor(int MOVE_FOR) {
  analogWrite(MOTOR_A_BW, 0); //motor A backwards DISABLED
  analogWrite(MOTOR_B_BW, 0); //motor B backwards DISABLED

  analogWrite(MOTOR_A_FW, SPEED_A_FW);
  analogWrite(MOTOR_B_FW, SPEED_B_FW);

  delay(MOVE_FOR); //the movement happens for MOVE_FOR milliseconds
  stop(); //motors turn off
}

void turnRight90Deg() {
  turnTime = TURN_TIME;

  analogWrite(MOTOR_A_BW, 0); //motor A backwards DISABLED
  analogWrite(MOTOR_B_BW, 0); //motor B backwards DISABLED
  
  analogWrite(MOTOR_A_FW, SPEED_A_R);
  analogWrite(MOTOR_B_FW, SPEED_B_R);

  delay(turnTime); //the movement happens for turnTime milliseconds
  stop(); //motors turn off
}

void turnLeft90Deg() {
  turnTime = TURN_TIME;

  analogWrite(MOTOR_A_BW, 0); //motor A backwards DISABLED
  analogWrite(MOTOR_B_BW, 0); //motor B backwards DISABLED
  
  analogWrite(MOTOR_A_FW, SPEED_A_L);
  analogWrite(MOTOR_B_FW, SPEED_B_L);

  delay(turnTime); //the movement happens for turnTime milliseconds
  stop(); //motors turn off
}

long getDistanceInCm() {
  digitalWrite(TRIG_SENS, LOW); //trigger pin starts LOW
  delayMicroseconds(2); //stabilising sensor before sending any signal

  //sending a 10 microsecond pulse to the sensor so the sensor emits an ultrasonic soundwave
  digitalWrite(TRIG_SENS, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_SENS, LOW);

  long duration = pulseIn(ECHO_SENS, HIGH); //pulseIn measures how long the echo pin stays high and stores its value in microseconds in duration

  if (duration == 0) { //therefore no soundwave was received -> there is no object to bounce off the waves from
    return 999; //means that no object was detected
  }

  long distance = duration * SOUND_SPEED / 2;
  //multiplying by speed of sound (on Earth: 0.0343 microseconds/cm) and dividing by 2 because sound travels to and from the object

  return distance; //distance is in cm
}

void stop() {
  analogWrite(MOTOR_A_FW, 0);
  analogWrite(MOTOR_A_BW, 0);
  analogWrite(MOTOR_B_BW, 0);
  analogWrite(MOTOR_B_FW, 0);
}

void avoidObject() {
  stop();

  turnRight90Deg();
  moveForwardsFor(MOVE_FOR);
  
  turnLeft90Deg();
  moveForwardsFor(MOVE_FOR * 2.5);

  turnLeft90Deg();
  moveForwardsFor(MOVE_FOR);

  turnRight90Deg();
}