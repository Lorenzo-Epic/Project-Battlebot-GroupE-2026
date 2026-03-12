///TODO: if sensor reading is 1000, then it should be ignored or some action should be taken
///TODO: if motor moves forward but sensor doesn't see it, then the robot should move back as a recovery condition
///TODO: make robot shut down maze logic if it senses black line

///~~~~~~~~~~~~~~~~~~~~~~ MOTORS VALUES ~~~~~~~~~~~~~~~~~~~~~~
const int LEFT_FORWARD_PIN  = 6;
const int LEFT_BACKWARD_PIN = 10;
const int RIGHT_FORWARD_PIN = 11;
const int RIGHT_BACKWARD_PIN = 9;

const int ROTATION_LEFT_PIN = 2;
const int ROTATION_RIGHT_PIN = 3;

// Motor speeds for straight driving
const int LEFT_FORWARD_SPEED = 255;
const int LEFT_BACKWARD_SPEED = 255;
const int RIGHT_FORWARD_SPEED = 223;
const int RIGHT_BACKWARD_SPEED = 200;

// Motor speeds for turning
const int LEFT_TURN_SPEED = 210;
const int RIGHT_TURN_SPEED = 200;

// Wheel and encoder values
const float WHEEL_DIAMETER_CM = 6.5;
const float WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const float TICKS_PER_REVOLUTION = 40.0;
const float TICKS_PER_CM = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_CM;

// Ignore encoder pulses that happen too quickly (unsigned long because it's what micros() uses)
const unsigned long MINIMUM_EDGE_TIME_US = 150;

// 90 degree turn calibration
const int TURN_90_TARGET_TICKS = 32;
const int TURN_5_TARGET_TICKS = 2;

// Encoder counters (volatile to ensure it's always upated)
volatile unsigned long leftTickCount = 0;
volatile unsigned long rightTickCount = 0;

// Time of previous encoder pulse
volatile unsigned long lastLeftEdgeTime = 0;
volatile unsigned long lastRightEdgeTime = 0;

///~~~~~~~~~~~~~~~~~~~~~~ ULTRASOUND SENSORS VALUES ~~~~~~~~~~~~~~~~~~~~~~ 
// front
const int TRIG_FRONT = 12;
const int ECHO_FRONT = 13;
// left
const int TRIG_LEFT = 7;
const int ECHO_LEFT = 4;
// right
const int TRIG_RIGHT = A3;
const int ECHO_RIGHT = 8;

// distance to wall 
const int WALL_DISTANCE = 20; // cm
const int TOO_CLOSE_TO_WALLS = 8; // cm


void setup() {

  Serial.begin(9600);

///~~~~~~~~~~~~~~~~~~~~~~ MOTORS SETUP ~~~~~~~~~~~~~~~~~~~~~~
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  pinMode(ROTATION_LEFT_PIN, INPUT_PULLUP);
  pinMode(ROTATION_RIGHT_PIN, INPUT_PULLUP);

  // Converts encoder pin numbers into interrupt pins which are needed by attach interrupt
  int leftInterruptPin = digitalPinToInterrupt(ROTATION_LEFT_PIN);
  int rightInterruptPin = digitalPinToInterrupt(ROTATION_RIGHT_PIN);

  // when there is a change in the encoder pins, call the function leftEncoderInterrupt or rightEncoderInterrupt respectively
  attachInterrupt(leftInterruptPin, leftEncoderInterrupt, CHANGE);
  attachInterrupt(rightInterruptPin, rightEncoderInterrupt, CHANGE);

  stopMotors();

///~~~~~~~~~~~~~~~~~~~~~~ ULTRASOUND SENSORS SETUP ~~~~~~~~~~~~~~~~~~~~~~
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
}

void loop() {

// Constrained because sometimes the trig doesn't recieve the echo and prints out a number above 1000
  float frontDistance = constrain(getDistance(TRIG_FRONT, ECHO_FRONT), 0, 1000);
  float leftDistance  = constrain(getDistance(TRIG_LEFT, ECHO_LEFT), 0, 1000);
  float rightDistance = constrain(getDistance(TRIG_RIGHT, ECHO_RIGHT), 0, 1000);

  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.print("  Left: ");
  Serial.print(leftDistance);
  Serial.print("  Right: ");
  Serial.println(rightDistance);

  // small adjustment if too close to wall
  if(leftDistance < rightDistance) {
    // turn slightly right
    turnRightForward5Degrees();
  }
  
  if(rightDistance < leftDistance) {
  // turn slightly left
    turnLeftForward5Degrees();
  }

  // Convenience booleans
  bool isWallOnLeft = leftDistance < WALL_DISTANCE || leftDistance == 1000;
  bool isWallOnRight = rightDistance < WALL_DISTANCE || rightDistance == 1000;
  bool isWallForward = frontDistance < WALL_DISTANCE || frontDistance == 1000;

  // follows the left wall all the time
// if it's at a dead end, escape
  if(isWallOnLeft && isWallOnRight && isWallForward) {
    turnRightBackward90Degrees();
    turnLeftForward90Degrees();
    Serial.print(" Turning 180 degrees ");
  }
//  else if left is open, go left
  else if(!isWallOnLeft) {
    turnLeftForward90Degrees();
    moveForwardCm(10);
//    startLeftForwardTurn();
    Serial.print(" Going left ");
  }
//  if forward is open, go forward
  else if (!isWallForward) {
    moveForwardCm(5);
    Serial.print(" Going forward ");
  }
// if right is open, go right (!isWallOnRight)
  else {
    turnRightForward90Degrees();
    moveForwardCm(10);
//    startRightForwardTurn();
    Serial.print(" Going right ");
  }

}

///~~~~~~~~~~~~~~~~~~~~~~ MOTORS FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
// Round a float to the nearest int by adding or removing 0.5 from the float because int to float doesn't round normally
int roundFloatToInt(float value) {
  if (value >= 0) {
    return (int)(value + 0.5);
  } else {
    return (int)(value - 0.5);
  }
}

// This function runs automatically every time the left encoder signal changes
void leftEncoderInterrupt() {
  unsigned long currentTime = micros();

  if (currentTime - lastLeftEdgeTime >= MINIMUM_EDGE_TIME_US) {
    leftTickCount = leftTickCount + 1;
    lastLeftEdgeTime = currentTime;
  }
}

// This function runs automatically every time the right encoder signal changes
void rightEncoderInterrupt() {
  unsigned long currentTime = micros();

  if (currentTime - lastRightEdgeTime >= MINIMUM_EDGE_TIME_US) {
    rightTickCount = rightTickCount + 1;
    lastRightEdgeTime = currentTime;
  }
}

// Reset both encoder counters to zero, turns interrupts off for writing to values then back on when function ends
void resetEncoderCounts() {
  noInterrupts(); 
  leftTickCount = 0;
  rightTickCount = 0;
  lastLeftEdgeTime = 0;
  lastRightEdgeTime = 0;
  interrupts();
}

// Get current left encoder count, put it into value, and return value, because you cannot resume interrupts after a return
unsigned long getLeftTicks() {
  long value;

  noInterrupts();
  value = leftTickCount;
  interrupts();

  return value;
}

// Get current right encoder count
unsigned long getRightTicks() {
  unsigned long value;

  noInterrupts();
  value = rightTickCount;
  interrupts();

  return value;
}

// Stop all motors
void stopMotors() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}

// Start moving forward
void startForwardMotion() {
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);

  analogWrite(LEFT_FORWARD_PIN, LEFT_FORWARD_SPEED);
  analogWrite(RIGHT_FORWARD_PIN, RIGHT_FORWARD_SPEED);
}

// Start moving backward
void startBackwardMotion() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);

  analogWrite(LEFT_BACKWARD_PIN, LEFT_BACKWARD_SPEED);
  analogWrite(RIGHT_BACKWARD_PIN, RIGHT_BACKWARD_SPEED);
}

// Start turning left
void startLeftForwardTurn() {
  analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, RIGHT_TURN_SPEED);
}

// Start turning left backwards
void startLeftBackwardTurn() {
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, RIGHT_TURN_SPEED);
}

// Start turning right
void startRightForwardTurn() {
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(LEFT_FORWARD_PIN, LEFT_TURN_SPEED);
}

// Start turning right backwards
void startRightBackwardTurn() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, LEFT_TURN_SPEED);
}

// direction = 1 = forward
void correctMotorSpeeds(int baseSpeed, int direction) {

// See how much the wheel rotations have differed
  int difference = getRightTicks() - getLeftTicks();
// If left is going faster, this returns a negative number

// Multiplied difference (difference between ticks * X) is the difference that will be written to PWM
  difference = (difference * 35);

// Slow down right and speed up left (because right is the stronger motor)
  int rightSpeed = baseSpeed - difference;
  int leftSpeed = baseSpeed + difference;

// Constrain corrected speed between 0 and 255
  rightSpeed = constrain(rightSpeed, 0, 255);
  leftSpeed = constrain(leftSpeed, 0, 255);

// Write corrected speeds to motors
  if (direction == 1) {
    analogWrite(LEFT_BACKWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, 0);

    analogWrite(LEFT_FORWARD_PIN, leftSpeed);
    analogWrite(RIGHT_FORWARD_PIN, rightSpeed);
  } else {
    analogWrite(LEFT_FORWARD_PIN, 0);
    analogWrite(RIGHT_FORWARD_PIN, 0);

    analogWrite(LEFT_BACKWARD_PIN, leftSpeed);
    analogWrite(RIGHT_BACKWARD_PIN, rightSpeed);
  }
}

// Move forward a given distance in centimeters
void moveForwardCm(float distanceCm) {
  int targetTicks = roundFloatToInt(distanceCm * TICKS_PER_CM);

  if (targetTicks <= 0) {
    return;
  }

  resetEncoderCounts();
  startForwardMotion();

  while (true) {
    unsigned long leftValue = getLeftTicks();
    unsigned long rightValue = getRightTicks();
    unsigned long averageTicks = (leftValue + rightValue) / 2;
    correctMotorSpeeds(240, 1);

    if (averageTicks >= (unsigned long)targetTicks) {
      break;
    }
  }

  stopMotors();
}

// Move backward a given distance in centimeters
void moveBackwardCm(float distanceCm) {
  int targetTicks = roundFloatToInt(distanceCm * TICKS_PER_CM);

  if (targetTicks <= 0) {
    return;
  }

  resetEncoderCounts();
  startBackwardMotion();

  while (true) {
    unsigned long leftValue = getLeftTicks();
    unsigned long rightValue = getRightTicks();
    unsigned long averageTicks = (leftValue + rightValue) / 2;
    correctMotorSpeeds(240, 0);

    if (averageTicks >= (unsigned long)targetTicks) {
      break;
    }
  }

  stopMotors();
}
 
// Turn left by about 90 degrees
void turnLeftForward90Degrees() {
  int targetTicks = TURN_90_TARGET_TICKS;

  resetEncoderCounts();
  startLeftForwardTurn();

  while (true) {
    unsigned long rightValue = getRightTicks();

    // During a left turn, the right wheel is the moving wheel
    if (rightValue >= (unsigned long)targetTicks) {
      break;
    }
  }

  stopMotors();
}

void turnLeftForward5Degrees() {
  int targetTicks = TURN_5_TARGET_TICKS;

  resetEncoderCounts();
  startLeftForwardTurn();

  while (true) {
    unsigned long rightValue = getRightTicks();

    // During a left turn, the right wheel is the moving wheel
    if (rightValue >= (unsigned long)targetTicks) {
      break;
    }
  }

  stopMotors();
}

void turnLeftBackward90Degrees() {
  int targetTicks = TURN_90_TARGET_TICKS;

  resetEncoderCounts();
  startLeftBackwardTurn();

  while (true) {
    unsigned long rightValue = getRightTicks();

    // During a left turn, the right wheel is the moving wheel
    if (rightValue >= (unsigned long)targetTicks) {
      break;
    }
  }

  stopMotors();
}

// Turn right by about 90 degrees
void turnRightForward90Degrees() {
  int targetTicks = TURN_90_TARGET_TICKS;

  resetEncoderCounts();
  startRightForwardTurn();

  while (true) {
    unsigned long leftValue = getLeftTicks();

    // During a right turn, the left wheel is the moving wheel
    if (leftValue >= (unsigned long)targetTicks) {
      break;
    }
  }

  stopMotors();
}

void turnRightForward5Degrees() {
  int targetTicks = TURN_5_TARGET_TICKS;

  resetEncoderCounts();
  startRightForwardTurn();

  while (true) {
    unsigned long leftValue = getLeftTicks();

    // During a right turn, the left wheel is the moving wheel
    if (leftValue >= (unsigned long)targetTicks) {
      break;
    }
  }

  stopMotors();
}

void turnRightBackward90Degrees() {
  int targetTicks = TURN_90_TARGET_TICKS;

  resetEncoderCounts();
  startRightBackwardTurn();

  while (true) {
    unsigned long leftValue = getLeftTicks();

    // During a right turn, the left wheel is the moving wheel
    if (leftValue >= (unsigned long)targetTicks) {
      break;
    }
  }

  stopMotors();
}


///~~~~~~~~~~~~~~~~~~~~~~ ULTRASOUND SENSORS FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
float getDistance(int trigPin, int echoPin) 
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(50);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(100);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
// this is speed of sound by microseconds so: distance (cm) = duration (µs/microseconds)* 0.017
// 0.017 is speed of sound already divided by 2, so instead of 0.034 its 0.017
  float distance = duration * 0.017; 

  return distance;
}
