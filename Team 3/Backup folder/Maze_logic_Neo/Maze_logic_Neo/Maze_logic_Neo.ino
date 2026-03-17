#include <Adafruit_NeoPixel.h>

// NeoPixels
const int PIN_NEO = A4;
const int NUM_PIXELS = 4;
// Pixel position mapping
// 0 = back left, 1 = back right, 2 = front right, 3 = front left
const int FRONT_LEFT  = 3;
const int FRONT_RIGHT = 2;
const int BACK_LEFT   = 0;
const int BACK_RIGHT  = 1;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEO, NEO_RGB + NEO_KHZ800);

//  Motors / Encoders 
const int LEFT_FORWARD_PIN  = 6;
const int LEFT_BACKWARD_PIN = 10;
const int RIGHT_FORWARD_PIN = 11;
const int RIGHT_BACKWARD_PIN = 9;

const int ROTATION_LEFT_PIN = 2;
const int ROTATION_RIGHT_PIN = 3;

const int LEFT_FORWARD_SPEED = 255;
const int LEFT_BACKWARD_SPEED = 255;
const int RIGHT_FORWARD_SPEED = 223;
const int RIGHT_BACKWARD_SPEED = 200;

const int LEFT_TURN_SPEED = 210;
const int RIGHT_TURN_SPEED = 200;

// Wheel / encoder
const float WHEEL_DIAMETER_CM = 6.5;
const float WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const float TICKS_PER_REVOLUTION = 40.0;
const float TICKS_PER_CM = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_CM;

const unsigned long MINIMUM_EDGE_TIME_US = 150;
const unsigned long STALL_TIMEOUT_MS = 3000; // if encoders don't change for this long, abort movement

const int TURN_90_TARGET_TICKS = 32;

volatile unsigned long leftTickCount = 0;
volatile unsigned long rightTickCount = 0;
volatile unsigned long lastLeftEdgeTime = 0;
volatile unsigned long lastRightEdgeTime = 0;

//  Ultrasonic 
const int TRIG_FRONT = 12;
const int ECHO_FRONT = 13;
const int TRIG_LEFT  = 7;
const int ECHO_LEFT  = 4;
const int TRIG_RIGHT = A3;
const int ECHO_RIGHT = 8;

const int WALL_DISTANCE_SIDE = 18;
const int WALL_DISTANCE_FRONT = 16;

const int FOLLOW_LEFT_WALL_IDEAL = 7;
const int FOLLOW_LEFT_WALL_FAR = 9;
const int FOLLOW_LEFT_WALL_VERY_FAR = 12;
const int FOLLOW_LEFT_WALL_VERY_VERY_FAR = 15;
const int FOLLOW_LEFT_WALL_CLOSE = 5;
const int FOLLOW_LEFT_WALL_VERY_CLOSE = 3;
const int FOLLOW_LEFT_WALL_VERY_VERY_CLOSE = 1;

//  Forward declarations 
void leftEncoderInterrupt();
void rightEncoderInterrupt();

void stopMotors();
void startForwardMotion();
void startBackwardMotion();
void startLeftForwardTurn();
void startRightForwardTurn();

void moveForwardCm(float distanceCm);
void moveBackwardCm(float distanceCm);

void turnLeftForward90Degrees();
void turnRightForward90Degrees();
void turnLeftForwardTicks(int ticks);
void turnRightForwardTicks(int ticks);

void resetEncoderCounts();
unsigned long getLeftTicks();
unsigned long getRightTicks();
int roundFloatToInt(float value);

float getDistance(int trigPin, int echoPin);

//  Light helpers (simple) 
void clearLights() {
  pixels.clear();
  pixels.show();
}

void showForwardLights() {
  pixels.clear();
  pixels.setPixelColor(FRONT_LEFT, pixels.Color(0,150,0));
  pixels.setPixelColor(FRONT_RIGHT, pixels.Color(0,150,0));
  pixels.show();
}

void showBackwardLights() {
  pixels.clear();
  pixels.setPixelColor(BACK_LEFT, pixels.Color(150,0,0));
  pixels.setPixelColor(BACK_RIGHT, pixels.Color(150,0,0));
  pixels.show();
}

void showLeftLights() {
  pixels.clear();
  pixels.setPixelColor(FRONT_LEFT, pixels.Color(150,120,0));
  pixels.setPixelColor(BACK_LEFT, pixels.Color(150,120,0));
  pixels.show();
}

void showRightLights() {
  pixels.clear();
  pixels.setPixelColor(FRONT_RIGHT, pixels.Color(150,120,0));
  pixels.setPixelColor(BACK_RIGHT, pixels.Color(150,120,0));
  pixels.show();
}

//  Setup 
void setup() {
  Serial.begin(9600);

  pixels.begin();
  pixels.show();

  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  pinMode(ROTATION_LEFT_PIN, INPUT_PULLUP);
  pinMode(ROTATION_RIGHT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ROTATION_LEFT_PIN), leftEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTATION_RIGHT_PIN), rightEncoderInterrupt, CHANGE);

  stopMotors();

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
}

//  Main loop 
void loop() {
  // read distances (constrain to 0..1000 so "no echo" maps to 1000 after our getDistance)
  float frontDistance = constrain(getDistance(TRIG_FRONT, ECHO_FRONT), 0, 1000);
  float leftDistance  = constrain(getDistance(TRIG_LEFT, ECHO_LEFT), 0, 1000);
  float rightDistance = constrain(getDistance(TRIG_RIGHT, ECHO_RIGHT), 0, 1000);

  Serial.print("Front: "); Serial.print(frontDistance);
  Serial.print("  Left: "); Serial.print(leftDistance);
  Serial.print("  Right: "); Serial.println(rightDistance);

  // LEFT WALL CORRECTION - simplified and fixed:
  // If left is very far -> small left correction (turn left a few ticks)
  if (leftDistance > FOLLOW_LEFT_WALL_VERY_VERY_FAR) {
    turnLeftForwardTicks(4);
  }
  else if (leftDistance > FOLLOW_LEFT_WALL_VERY_FAR) {
    turnLeftForwardTicks(3);
  }
  else if (leftDistance > FOLLOW_LEFT_WALL_FAR) {
    turnLeftForwardTicks(2);
  }
  // if within ±1cm of ideal do nothing (stable)
  else if ( (leftDistance >= FOLLOW_LEFT_WALL_IDEAL - 1) && (leftDistance <= FOLLOW_LEFT_WALL_IDEAL + 1) ) {
    // keep current heading — no small correction
  }
  // if slightly close -> small right correction
  else if (leftDistance > FOLLOW_LEFT_WALL_CLOSE) {
    turnRightForwardTicks(2);
  }
  else if (leftDistance > FOLLOW_LEFT_WALL_VERY_CLOSE) {
    turnRightForwardTicks(3);
  }
  else if (leftDistance > FOLLOW_LEFT_WALL_VERY_VERY_CLOSE) {
    turnRightForwardTicks(4);
  }

  // detect presence of walls (treat 1000 as "no reading" = open)
  bool leftValid  = leftDistance < 1000;
  bool rightValid = rightDistance < 1000;
  bool frontValid = frontDistance < 1000;

  bool isWallOnLeft  = leftValid  && (leftDistance < WALL_DISTANCE_SIDE);
  bool isWallOnRight = rightValid && (rightDistance < WALL_DISTANCE_SIDE);
  bool isWallForward = frontValid && (frontDistance < WALL_DISTANCE_FRONT);

  // If a sensor is invalid (1000), treat it as "open" rather than wall.
  // That avoids treating failed sensors as walls and getting stuck.

  // Dead-end escape: choose the side with more space (prefer left if leftDistance > rightDistance)
  if ( (isWallOnLeft || !leftValid) && (isWallOnRight || !rightValid) && (isWallForward || !frontValid) ) {
    // both sides & front are blocked -> compare numeric distances (if available)
    if (leftValid && rightValid) {
      if (leftDistance > rightDistance) {
        // more space on left -> turn left twice
        turnLeftForward90Degrees();
        turnLeftForward90Degrees();
        Serial.println("Dead end: turn 180 left");
      } else {
        turnRightForward90Degrees();
        turnRightForward90Degrees();
        Serial.println("Dead end: turn 180 right");
      }
    } else if (leftValid && !rightValid) {
      // right sensor bad -> try left
      turnLeftForward90Degrees();
      turnLeftForward90Degrees();
      Serial.println("Dead end: right sensor bad, turning left 180");
    } else if (!leftValid && rightValid) {
      turnRightForward90Degrees();
      turnRightForward90Degrees();
      Serial.println("Dead end: left sensor bad, turning right 180");
    } else {
      // both sensors invalid — do a safe 180 to try free space
      turnRightForward90Degrees();
      turnRightForward90Degrees();
      Serial.println("Dead end: both side sensors invalid, turning 180 right");
    }
  }
  // else if left is open -> go left
  else if (!isWallOnLeft) {
    turnLeftForward90Degrees();
    moveForwardCm(10);
    Serial.println("Going left");
  }
  // else if forward open -> go forward
  else if (!isWallForward) {
    moveForwardCm(5);
    Serial.println("Going forward");
  }
  // else -> go right
  else {
    turnRightForward90Degrees();
    moveForwardCm(10);
    Serial.println("Going right");
  }
}

//  Motor / movement 
void stopMotors() {
  clearLights();
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}

void startForwardMotion() {
  showForwardLights();
  // reset stall timers handled inside movement functions
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(LEFT_FORWARD_PIN, LEFT_FORWARD_SPEED);
  analogWrite(RIGHT_FORWARD_PIN, RIGHT_FORWARD_SPEED);
}

void startBackwardMotion() {
  showBackwardLights();
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, LEFT_BACKWARD_SPEED);
  analogWrite(RIGHT_BACKWARD_PIN, RIGHT_BACKWARD_SPEED);
}

void startLeftForwardTurn() {
  showLeftLights();
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, RIGHT_TURN_SPEED);
}

void startRightForwardTurn() {
  showRightLights();
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(LEFT_FORWARD_PIN, LEFT_TURN_SPEED);
}

// safe stall-check helper used inside movement loops
bool checkForStall(unsigned long &lastProgressTicks, unsigned long &lastProgressMs, unsigned long currentTickSum) {
  if (currentTickSum > lastProgressTicks) {
    lastProgressTicks = currentTickSum;
    lastProgressMs = millis();
    return false; // not stalled
  } else {
    if (millis() - lastProgressMs > STALL_TIMEOUT_MS) {
      // stalled
      Serial.println("STALL: encoder didn't progress, aborting movement");
      return true;
    }
    return false;
  }
}

void moveForwardCm(float distanceCm) {
  int targetTicks = roundFloatToInt(distanceCm * TICKS_PER_CM);
  if (targetTicks <= 0) return;

  resetEncoderCounts();
  startForwardMotion();

  unsigned long lastProgressTicks = 0;
  unsigned long lastProgressMs = millis();

  while (true) {
    unsigned long averageTicks = (getLeftTicks() + getRightTicks()) / 2;

    // stall detection
    if (checkForStall(lastProgressTicks, lastProgressMs, averageTicks)) {
      stopMotors();
      return; // abort movement on stall
    }

    if (averageTicks >= (unsigned long)targetTicks) break;
  }

  stopMotors();
}

void moveBackwardCm(float distanceCm) {
  int targetTicks = roundFloatToInt(distanceCm * TICKS_PER_CM);
  if (targetTicks <= 0) return;

  resetEncoderCounts();
  startBackwardMotion();

  unsigned long lastProgressTicks = 0;
  unsigned long lastProgressMs = millis();

  while (true) {
    unsigned long averageTicks = (getLeftTicks() + getRightTicks()) / 2;

    if (checkForStall(lastProgressTicks, lastProgressMs, averageTicks)) {
      stopMotors();
      return;
    }

    if (averageTicks >= (unsigned long)targetTicks) break;
  }

  stopMotors();
}

void turnLeftForward90Degrees() {
  resetEncoderCounts();
  startLeftForwardTurn();

  unsigned long lastProgressTicks = 0;
  unsigned long lastProgressMs = millis();

  while (true) {
    unsigned long rightValue = getRightTicks();

    if (checkForStall(lastProgressTicks, lastProgressMs, rightValue)) {
      stopMotors();
      return;
    }

    if (rightValue >= (unsigned long)TURN_90_TARGET_TICKS) break;
  }

  stopMotors();
}

void turnRightForward90Degrees() {
  resetEncoderCounts();
  startRightForwardTurn();

  unsigned long lastProgressTicks = 0;
  unsigned long lastProgressMs = millis();

  while (true) {
    unsigned long leftValue = getLeftTicks();

    if (checkForStall(lastProgressTicks, lastProgressMs, leftValue)) {
      stopMotors();
      return;
    }

    if (leftValue >= (unsigned long)TURN_90_TARGET_TICKS) break;
  }

  stopMotors();
}

void turnLeftForwardTicks(int ticks) {
  resetEncoderCounts();
  startLeftForwardTurn();

  unsigned long lastProgressTicks = 0;
  unsigned long lastProgressMs = millis();

  while (true) {
    unsigned long rightValue = getRightTicks();

    if (checkForStall(lastProgressTicks, lastProgressMs, rightValue)) {
      stopMotors();
      return;
    }

    if (rightValue >= (unsigned long)ticks) break;
  }

  stopMotors();
}

void turnRightForwardTicks(int ticks) {
  resetEncoderCounts();
  startRightForwardTurn();

  unsigned long lastProgressTicks = 0;
  unsigned long lastProgressMs = millis();

  while (true) {
    unsigned long leftValue = getLeftTicks();

    if (checkForStall(lastProgressTicks, lastProgressMs, leftValue)) {
      stopMotors();
      return;
    }

    if (leftValue >= (unsigned long)ticks) break;
  }

  stopMotors();
}

//  Encoders 
void leftEncoderInterrupt() {
  unsigned long currentTime = micros();
  if (currentTime - lastLeftEdgeTime >= MINIMUM_EDGE_TIME_US) {
    leftTickCount++;
    lastLeftEdgeTime = currentTime;
  }
}

void rightEncoderInterrupt() {
  unsigned long currentTime = micros();
  if (currentTime - lastRightEdgeTime >= MINIMUM_EDGE_TIME_US) {
    rightTickCount++;
    lastRightEdgeTime = currentTime;
  }
}

void resetEncoderCounts() {
  noInterrupts();
  leftTickCount = 0;
  rightTickCount = 0;
  lastLeftEdgeTime = 0;
  lastRightEdgeTime = 0;
  interrupts();
}

unsigned long getLeftTicks() {
  unsigned long value;
  noInterrupts();
  value = leftTickCount;
  interrupts();
  return value;
}

unsigned long getRightTicks() {
  unsigned long value;
  noInterrupts();
  value = rightTickCount;
  interrupts();
  return value;
}

int roundFloatToInt(float value) {
  if (value >= 0) return (int)(value + 0.5);
  return (int)(value - 0.5);
}

//  Ultrasonic (safe) 
// Return distance in cm. If no echo detected, return 1000 to indicate "open / no reading".
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(50);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // short pulse
  digitalWrite(trigPin, LOW);

  // timeout in microseconds (e.g., 30000us = 30ms ~ 5 meters)
  unsigned long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    // no echo within timeout -> treat as "open / no reading"
    return 1000.0;
  }

  float distance = duration * 0.017; // microsecond-to-cm factor (speed of sound / 2)
  return distance;
}
