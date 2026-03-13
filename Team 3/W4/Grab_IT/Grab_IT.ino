// IMPORTANT NOTE FOR THIS CODE:
// gripperUpdate() needs to be called in every long loop to keep servos powered
// waitMs() must be used instead of delay() to keep the servos powwered

///~~~~~~~~~~~~~~~~~~~~~~ MOTORS VALUES ~~~~~~~~~~~~~~~~~~~~~~
#define LEFT_FORWARD_PIN 7
#define LEFT_BACKWARD_PIN 10 
#define RIGHT_FORWARD_PIN 11
#define RIGHT_BACKWARD_PIN 9

#define ROTATION_LEFT_PIN 2
#define ROTATION_RIGHT_PIN 3

// Motor speeds for straight driving
#define LEFT_FORWARD_SPEED 255
#define LEFT_BACKWARD_SPEED 255
#define RIGHT_FORWARD_SPEED 223
#define RIGHT_BACKWARD_SPEED 200

// Motor speeds for turning
#define LEFT_TURN_SPEED 210
#define RIGHT_TURN_SPEED 200

// Wheel and encoder values (not define for float operations)
const float WHEEL_DIAMETER_CM = 6.5;
const float WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const float TICKS_PER_REVOLUTION = 40.0;
const float TICKS_PER_CM = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_CM;

// Ignore encoder pulses that happen too quickly (unsigned long because it's what micros() uses, cannot be #define(d))
const unsigned long MINIMUM_EDGE_TIME_US = 150;

// 90 degree turn calibration
#define TURN_90_TARGET_TICKS 32;

// Encoder counters (volatile to ensure it's always upated)
volatile unsigned long leftTickCount = 0;
volatile unsigned long rightTickCount = 0;

// Time of previous encoder pulse
volatile unsigned long lastLeftEdgeTime = 0;
volatile unsigned long lastRightEdgeTime = 0;

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
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
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

///~~~~~~~~~~~~~~~~~~~~~~ GRIPPER SETUP ~~~~~~~~~~~~~~~~~~~~~~
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
}

void loop() {
  // make sure its closed first
  closeGripper();
  waitMs(800);

  // open, wait a second, close, wait a second, open
  openGripper();
  waitMs(1000);
  closeGripper();
  waitMs(1000);
  openGripper();

  // drive towards cone, gripper open
  moveForwardCmWithGripper(25, false);

  waitMs(1000);
  // grab cone
  closeGripper();
  
  waitMs(1000);
  // drive 25 more cm, gripper closed
  moveForwardCmWithGripper(25, true);
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

// direction = 1 = forward
void correctMotorSpeeds(int baseSpeed, int direction) {

// See how much the wheel rotations have differed
  int difference = getRightTicks() - getLeftTicks();

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
void moveForwardCmWithGripper(float distanceCm, boolean isGripping) {
  int targetTicks = roundFloatToInt(distanceCm * TICKS_PER_CM);

  if (targetTicks <= 0) {
    return;
  }

  resetEncoderCounts();
  startForwardMotion();

  while (true) {
    gripperUpdate();
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
