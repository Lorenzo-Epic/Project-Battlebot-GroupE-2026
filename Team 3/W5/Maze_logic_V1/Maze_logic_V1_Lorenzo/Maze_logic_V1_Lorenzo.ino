///TODO: if motor moves forward but rotation sensor doesn't see it, then the robot should move back as a recovery condition
///TODO: make robot shut down maze logic if it senses black line
///TODO: a way to make the robot go past the edge of the wall so it doesnt FUCKING BUMP INTO IT FUCKK (system so it sees when the last time the wall was detected, then goes 5cm more forward
///TODO: implement a timeout for all of the rotors, if the robot doesn't go where it ended up wanting to, it boosts the motors to 255 for one second, if that doesn't work if three seconds pass and it doesnt move where it wants to move, it goes opposite of the direction it was going (backwards if forwards, forwards if backwards), turns in the opposite direction in which it was turning (left if right, right is left), and then goes back in its original direction again for the same distance (eg. forwards, backwards)
///TODO: MORE FUCKING SYSTEM OUT LOG MESSAGES
///TODO: recovery code, 
///TODO: problem, recovery funciton doesnt work if the robot never moved at all.
///TODO: when bobot goes back in recovery 180 degree, if stuck, rotate the opposite wheel in the opposite direction (if it was going left back, rotate right back BY CALCULATING THE REMAINING AMOUNT OF TICKS AND THEN CALLING THE FUNCTION WITH THOSE TICKS), AND THIS WILL APPLY FOR BOTH THE FORWARD AND BACK MOTION  


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

// Timeout for recovery
const unsigned long STALL_TIMEOUT_MS = 2000;

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
const int WALL_DISTANCE_SIDE = 18; // cm
const int WALL_DISTANCE_FRONT = 18;
const int FOLLOW_LEFT_WALL_VERY_VERY_CLOSE = 1;
const int FOLLOW_LEFT_WALL_VERY_CLOSE = 3;
const int FOLLOW_LEFT_WALL_CLOSE = 5;
const int FOLLOW_LEFT_WALL_IDEAL = 7; // cm
const int FOLLOW_LEFT_WALL_FAR = 9;
const int FOLLOW_LEFT_WALL_VERY_FAR = 12;
const int FOLLOW_LEFT_WALL_VERY_VERY_FAR = 15;

///~~~~~~~~~~~~~~~~~~~~~~ GRIPPER VALUES ~~~~~~~~~~~~~~~~~~~~~~
#define SERVO_PIN 5 
#define GRIPPER_OPEN_US  1820
#define GRIPPER_CLOSE_US 1050

// This will be the target pulse width for the gripper
volatile int gripperPulseUs = GRIPPER_OPEN_US;
unsigned long lastServoMs = 0;

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

  ///~~~~~~~~~~~~~~~~~~~~~~ GRIPPER SETUP ~~~~~~~~~~~~~~~~~~~~~~
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
}

void loop() {

/// Keeps gripepr servo powered
  closeGripper();
  waitMs(500);

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

  // Left wall correction
  // If very far from left wall, turn right a lot
  if (leftDistance > FOLLOW_LEFT_WALL_VERY_VERY_FAR) {
    turnLeftForwardTicksWithDeadEnd(6, false);
    Serial.println("Very very far from left wall, turning left ");
  }
  else if (leftDistance > FOLLOW_LEFT_WALL_VERY_FAR) {
    Serial.println("Very far from left wall, turning left ");
    turnLeftForwardTicksWithDeadEnd(4, false);
  }
  else if (leftDistance > FOLLOW_LEFT_WALL_FAR) {
    Serial.println("far from left wall, turning left ");
    turnLeftForwardTicksWithDeadEnd(2, false);
  }
  else if (leftDistance >= FOLLOW_LEFT_WALL_IDEAL - 1 && leftDistance <= FOLLOW_LEFT_WALL_IDEAL + 1) {
    Serial.println("ideal distance from left wall, adjustment skipped ");
  }
  else if (leftDistance > FOLLOW_LEFT_WALL_CLOSE) {
    turnRightForwardTicksWithDeadEnd(2, false);
    Serial.println("close to left wall, turning right");
  }
  else if (leftDistance > FOLLOW_LEFT_WALL_VERY_CLOSE) {
    turnRightForwardTicksWithDeadEnd(4, false);
    Serial.println("Very close to left wall, turning right");
  }
  else if (leftDistance > FOLLOW_LEFT_WALL_VERY_VERY_CLOSE) {
    turnRightForwardTicksWithDeadEnd(6, false);
    Serial.println("Very very close to left wall, turning right");
  }

  // Convenience booleans
  bool isWallOnLeft = leftDistance < WALL_DISTANCE_SIDE;
  bool isWallOnRight = rightDistance < WALL_DISTANCE_SIDE || rightDistance == 1000;
  bool isWallForward = frontDistance < WALL_DISTANCE_FRONT || frontDistance == 1000;

  // follows the left wall all the time
// if it's at a dead end, escape
  if(isWallOnLeft && isWallOnRight && isWallForward) {
    if (leftDistance > rightDistance) {
      //  if more space on left, recover turning left
      turnLeftBackwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, true);
      turnLeftForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, true);
      Serial.println("Dead end with left wall further away, Turning 180 degrees left ");
    } else { // else, recover turning right
      turnRightBackwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, true);
      turnRightForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, true);
      Serial.println("Dead end with right wall further away, Turning 180 degrees right ");
    }

  }
//  else if left is open, go left
  else if(!isWallOnLeft) {
    moveForwardCm(3);
    turnLeftForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, false);
    moveForwardCm(5);
//    startLeftForwardTurn();
    Serial.println("No wall on left, Going forward, then left, then forward");
  }
//  if forward is open, go forward
  else if (!isWallForward) {
    moveForwardCm(3);
    Serial.println("Wall left but none forward, Going forward ");
  }
// if right is open, go right (!isWallOnRight)
  else {
    moveForwardCm(3);
    turnRightForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, false);
    moveForwardCm(5);
//    startRightForwardTurn();
    Serial.println("Wall left and forward, Going forward, right, and then forward");
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
  unsigned long value;

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
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, RIGHT_TURN_SPEED);
}

// Start turning left backwards
void startLeftBackwardTurn() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, RIGHT_TURN_SPEED);
}

// Start turning right
void startRightForwardTurn() {
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(LEFT_FORWARD_PIN, LEFT_TURN_SPEED);
}

// Start turning right backwards
void startRightBackwardTurn() {
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
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
  unsigned long targetTicks = roundFloatToInt(distanceCm * TICKS_PER_CM);

  if (targetTicks <= 0) {
    return;
  }

  resetEncoderCounts();
  startForwardMotion();

  unsigned long lastTicks = 0;
  unsigned long lastStallTime = millis();

  while (true) {
    gripperUpdate();
    unsigned long leftValue = getLeftTicks();
    unsigned long rightValue = getRightTicks();
    unsigned long averageTicks = (leftValue + rightValue) / 2;
    correctMotorSpeeds(240, 1);

    if (averageTicks >= targetTicks) {
      break;
    }

//  if robot moved forward, update last ticks and last stall time (because it hasn't stalled)
    if (averageTicks != lastTicks) {
      lastTicks = averageTicks;
      lastStallTime = millis();
    }

// if robot has not been moving for too long, do recovery, then break movement
    if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
      stopMotors();
      moveBackwardCm(5);
      break;
    }
    
  }

  stopMotors();
}

// Move backward a given distance in centimeters
void moveBackwardCm(float distanceCm) {
  unsigned long targetTicks = roundFloatToInt(distanceCm * TICKS_PER_CM);

  if (targetTicks <= 0) {
    return;
  }

  unsigned long lastTicks = 0;
  unsigned long lastStallTime = millis();

  resetEncoderCounts();
  startBackwardMotion();

  while (true) {
    gripperUpdate();
    unsigned long leftValue = getLeftTicks();
    unsigned long rightValue = getRightTicks();
    unsigned long averageTicks = (leftValue + rightValue) / 2;
    correctMotorSpeeds(240, 0);

    if (averageTicks >= targetTicks) {
      break;
    }
    
    if (averageTicks != lastTicks) {
      lastTicks = averageTicks;
      lastStallTime = millis();
    }

    if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
      stopMotors();
      moveForwardCm(5);
      break;
    }
  }

  stopMotors();
}
 
void turnLeftForwardTicksWithDeadEnd(unsigned long targetTicks, boolean isDeadEnd) {

  if (targetTicks <= 0) {
    return;
  }

  unsigned long lastTicks = 0;
  unsigned long lastStallTime = millis();

  resetEncoderCounts();
  startLeftForwardTurn();

  while (true) {
    gripperUpdate();
    
    unsigned long rightValue = getRightTicks();

    // During a left turn, the right wheel is the moving wheel
    if (rightValue >= targetTicks) {
      break;
    }
    
    if (rightValue != lastTicks) {
      lastTicks = rightValue;
      lastStallTime = millis();
    }

    if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
      if (isDeadEnd) {
        unsigned long remainingTicks = targetTicks - rightValue;
        moveBackwardCm(6);
        turnRightBackwardTicksWithDeadEnd(remainingTicks, true);
        break;
      } else {
        stopMotors();
        moveBackwardCm(3);
        turnRightBackwardTicksWithDeadEnd(5, false);
        break;
      }
    }
  }
  stopMotors();
}

void turnRightForwardTicksWithDeadEnd(unsigned long targetTicks, boolean isDeadEnd) {

  if (targetTicks <= 0) {
    return;
  }

  unsigned long lastTicks = 0;
  unsigned long lastStallTime = millis();

  resetEncoderCounts();
  startRightForwardTurn();

  while (true) {
    gripperUpdate();
    
    unsigned long leftValue = getLeftTicks();

    // During a right turn, the left wheel is the moving wheel
    if (leftValue >= targetTicks) {
      break;
    }
    
    if (leftValue != lastTicks) {
      lastTicks = leftValue;
      lastStallTime = millis();
    }

    if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
      if (isDeadEnd) {
        unsigned long remainingTicks = targetTicks - leftValue;
        moveBackwardCm(6);
        turnLeftBackwardTicksWithDeadEnd(remainingTicks, true);
        break;
      } else {
        stopMotors();
        moveBackwardCm(3);
        turnLeftBackwardTicksWithDeadEnd(5, false);
        break;
      }
    }
  }
  stopMotors();
}

void turnLeftBackwardTicksWithDeadEnd(unsigned long targetTicks, boolean isDeadEnd) {

  if (targetTicks <= 0) {
    return;
  }

  unsigned long lastTicks = 0;
  unsigned long remainingTicks = 0;
  unsigned long lastStallTime = millis();

  resetEncoderCounts();
  startRightBackwardTurn();

  while (true) {
    gripperUpdate();
    
    unsigned long leftValue = getLeftTicks();

    // During a right turn, the left wheel is the moving wheel
    if (leftValue >= targetTicks) {
      break;
    }

    if (leftValue != lastTicks) {
      lastTicks = leftValue;
      lastStallTime = millis();
    }

    if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
      if (isDeadEnd) {
        unsigned long remainingTicks = targetTicks - leftValue;
        moveForwardCm(6);
        turnRightForwardTicksWithDeadEnd(remainingTicks, true);
        break;
      } else {
        stopMotors();
        moveForwardCm(3);
        turnRightForwardTicksWithDeadEnd(5, false);
        break; 
      }
    }
  }

  stopMotors();
}

void turnRightBackwardTicksWithDeadEnd(unsigned long targetTicks, boolean isDeadEnd) {

  if (targetTicks <= 0) {
    return;
  }

  unsigned long lastTicks = 0;
  unsigned long lastStallTime = millis();
  
  resetEncoderCounts();
  startLeftBackwardTurn();

  while (true) {
    gripperUpdate();
    
    unsigned long rightValue = getRightTicks();

    // During a left turn, the right wheel is the moving wheel
    if (rightValue >= targetTicks) {
      break;
    }

    if (rightValue != lastTicks) {
      lastTicks = rightValue;
      lastStallTime = millis();
    }

    if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
      if (isDeadEnd) {
        unsigned long remainingTicks = targetTicks - rightValue;
        moveForwardCm(6);
        turnLeftForwardTicksWithDeadEnd(remainingTicks, true);
        break;
      } else {
        stopMotors();
        moveForwardCm(3);
        turnRightForwardTicksWithDeadEnd(5, false);
        break; 
      }
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

///~~~~~~~~~~~~~~~~~~~~~~ GRIPPER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
void gripperUpdate() {
  // Send one servo pulse every 20ms to keep it powered
  unsigned long now = millis();
  // if 20ms have passed, send another pulse to servo
  if (now - lastServoMs >= 20) {
    lastServoMs = now;
    digitalWrite(SERVO_PIN, HIGH);
    // PulseUs will be either the value to close or open the gripper, delay to keep the pulse HIGH for required amount
    delayMicroseconds(gripperPulseUs);
    digitalWrite(SERVO_PIN, LOW);
  }
}

// Functions to update gripperPulseUs depending on if the gripper should be update or closed, this is used in the gripperUpdate()
void openGripper()  {
  gripperPulseUs = GRIPPER_OPEN_US;
}

void closeGripper() {
  gripperPulseUs = GRIPPER_CLOSE_US;
}

// Replace ALL delay(ms) with this (keeps servo powered during waits)
void waitMs(unsigned long ms) {
  unsigned long start = millis();
  // While milis time - start time is less than desired wait time, do nothing and keep updating the gripper
  while (millis() - start < ms) {
    gripperUpdate();
  }
}
