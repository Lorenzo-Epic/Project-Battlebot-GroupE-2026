#include <Adafruit_NeoPixel.h>

// state
boolean isStartSequenceActive = false;
boolean isMazeNavigationActive = false;

// line sensors
const int LINE_SENSOR_PINS[] = {A4, A5, A6, A7};
#define BLACK_LINE_THRESHOLD 500

// servo
#define SERVO_PIN 11
#define GRIPPER_OPEN_US 1820
#define GRIPPER_CLOSE_US 1000
#define SERVO_CYCLE_REPEAT 10

// ultrasonic sensor
// front
#define FRONT_TRIG 4
#define FRONT_ECHO 7
// left
#define LEFT_TRIG 12
#define LEFT_ECHO 13
// right
#define RIGHT_TRIG A0
#define RIGHT_ECHO A1

// distance sensor classs
class DistanceSensor {
  private:
    int trigPin;
    int echoPin;
    const int MAX_DISTANCE = 100;

    float getPulseDuration() {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      return pulseIn(echoPin, HIGH, 60 * MAX_DISTANCE);
    }

  public:
    DistanceSensor(int tPin, int ePin) {
      trigPin = tPin;
      echoPin = ePin;
      pinMode(trigPin, OUTPUT);
      digitalWrite(trigPin, LOW);
      pinMode(echoPin, INPUT);
    }

    double getCurrentDistance() {
      float pulse = getPulseDuration();
      if (pulse < 100) return MAX_DISTANCE;
      return (pulse * 0.0343) / 2.0;
    }
};

// dinstance sensor
DistanceSensor frontSensor(FRONT_TRIG, FRONT_ECHO);
DistanceSensor leftSensor(LEFT_TRIG, LEFT_ECHO);
DistanceSensor rightSensor(RIGHT_TRIG, RIGHT_ECHO);

int distanceFront;
int distanceLeft;
int distanceRight;

// motors
#define LEFT_FORWARD_PIN 5
#define LEFT_BACKWARD_PIN 10
#define RIGHT_FORWARD_PIN 6
#define RIGHT_BACKWARD_PIN 9
#define MAX_MOTOR_SPEED 255

// encoders
#define ROTATION_LEFT_PIN 2
#define ROTATION_RIGHT_PIN 3
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

// led
#define NEOPIXEL_PIN 8
Adafruit_NeoPixel statusLEDs(4, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);
const uint32_t LED_COLOR_BLUE = statusLEDs.Color(0, 0, 255);

// the helper functions

void updateDistances() {
  distanceFront = frontSensor.getCurrentDistance();
  distanceLeft  = leftSensor.getCurrentDistance();
  distanceRight = rightSensor.getCurrentDistance();
}

void setupMotorPins() {
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);
}

void stopMotors() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}

void driveForward() {
  analogWrite(LEFT_FORWARD_PIN, MAX_MOTOR_SPEED * 0.9);
  analogWrite(RIGHT_FORWARD_PIN, MAX_MOTOR_SPEED);
}

void driveBackward() {
  analogWrite(LEFT_BACKWARD_PIN, MAX_MOTOR_SPEED);
  analogWrite(RIGHT_BACKWARD_PIN, MAX_MOTOR_SPEED);
}

void rotateLeft() {
  analogWrite(LEFT_BACKWARD_PIN, MAX_MOTOR_SPEED * 0.8);
  analogWrite(RIGHT_FORWARD_PIN, MAX_MOTOR_SPEED * 0.8);
}

void rotateRight() {
  analogWrite(LEFT_FORWARD_PIN, MAX_MOTOR_SPEED * 0.8);
  analogWrite(RIGHT_BACKWARD_PIN, MAX_MOTOR_SPEED * 0.8);
}

// isr encoder

void countLeft() { leftEncoderCount++; }
void countRight() { rightEncoderCount++; }

// the moves with the encoder

void driveForwardOnPulses(int target) {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  driveForward();
  while (leftEncoderCount < target && rightEncoderCount < target) {
    updateDistances();
    if (distanceFront < 15) break;
  }
  stopMotors();
}

void turnLeftOnPulses(int target) {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  rotateLeft();
  while (rightEncoderCount < target);
  stopMotors();
}

void turnRightOnPulses(int target) {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  rotateRight();
  while (leftEncoderCount < target);
  stopMotors();
}

// black.

boolean reachedBlackZone() {
  int count = 0;
  for (int i = 0; i < 4; i++)
    if (analogRead(LINE_SENSOR_PINS[i]) > BLACK_LINE_THRESHOLD)
      count++;
  return count >= 3;
}

//servo

void setServo(int pulse) {
  for (int i = 0; i < SERVO_CYCLE_REPEAT; i++) {
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(SERVO_PIN, LOW);
    delay(20);
  }
}

void openGripper() { setServo(GRIPPER_OPEN_US); }
void closeGripper() { setServo(GRIPPER_CLOSE_US); }

// one big setup

void setup() {

  setupMotorPins();
  pinMode(SERVO_PIN, OUTPUT);

  pinMode(ROTATION_LEFT_PIN, INPUT);
  pinMode(ROTATION_RIGHT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ROTATION_LEFT_PIN), countLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTATION_RIGHT_PIN), countRight, CHANGE);

  statusLEDs.begin();
  statusLEDs.fill(LED_COLOR_BLUE, 0, 4);
  statusLEDs.show();
}

// loops

void loop() {

  updateDistances();

  if (isMazeNavigationActive) {

    if (reachedBlackZone()) {
      stopMotors();
      openGripper();
      delay(500);
      isMazeNavigationActive = false;
      isStartSequenceActive = false;
      return;
    }

    const int WALL = 18;

    // LEFT HAND RULE
    if (distanceLeft > WALL) {
      turnLeftOnPulses(35);
      driveForwardOnPulses(25);
    }
    else if (distanceFront > WALL) {
      driveForwardOnPulses(25);
    }
    else if (distanceRight > WALL) {
      turnRightOnPulses(35);
      driveForwardOnPulses(25);
    }
    else {
      // Dead end
      rotateLeft();
      delay(700);
      stopMotors();
    }
  }

  else {

    if (!isStartSequenceActive && distanceFront < 30) {
      openGripper();
      delay(1000);
      isStartSequenceActive = true;
    }

    if (isStartSequenceActive) {
      driveForward();
      delay(800);
      closeGripper();
      turnLeftOnPulses(35);
      driveForwardOnPulses(60);
      isMazeNavigationActive = true;
    }
  }
}
