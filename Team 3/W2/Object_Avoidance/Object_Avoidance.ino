///~~~~~~~~~~~~~~~~~~~~~~ MOTORS VALUES ~~~~~~~~~~~~~~~~~~~~~~
const int LEFT_BACKWARD_PIN = 10;
const int LEFT_FORWARD_PIN = 7;
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

// Encoder counters (volatile to ensure it's always upated)
volatile unsigned long leftTickCount = 0;
volatile unsigned long rightTickCount = 0;

// Time of previous encoder pulse
volatile unsigned long lastLeftEdgeTime = 0;
volatile unsigned long lastRightEdgeTime = 0;

///~~~~~~~~~~~~~~~~~~~~~~ ULTRASOUND SENSOR VALUES ~~~~~~~~~~~~~~~~~~~~~~
// Obstacle trigger range in cm.
const float ULTRASOUND_DISTANCE_MIN = 0.0f;
const float ULTRASOUND_DISTANCE_MAX = 25.0f;

const int ULTRASOUND_TRIG_PIN = 12;
const int ULTRASOUND_ECHO_PIN = 13;

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
  
///~~~~~~~~~~~~~~~~~~~~~~ ULTRASOUND SENSOR SETUP ~~~~~~~~~~~~~~~~~~~~~~
  pinMode(ULTRASOUND_TRIG_PIN, OUTPUT);
  pinMode(ULTRASOUND_ECHO_PIN, INPUT);
}

void loop() {
  if (isObjectNearby()) {
    avoidObject();
  } else {
    moveForwardCm(10);
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
void startLeftTurn() {
  analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, RIGHT_TURN_SPEED);
}

// Start turning right
void startRightTurn() {
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(LEFT_FORWARD_PIN, LEFT_TURN_SPEED);
}

// Avoid object moveset()
void avoidObject() {
    stopMotors();
    delay(200);

    turnRight90Degrees();
    delay(200);

    moveForwardCm(20);
    delay(200);

    turnLeft90Degrees();
    delay(200);

    moveForwardCm(40);
    delay(200);

    turnLeft90Degrees();
    delay(200);

    moveForwardCm(20);
    delay(200);

    turnRight90Degrees();
    delay(200);
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

    if (averageTicks >= (unsigned long)targetTicks) {
      break;
    }
  }

  stopMotors();
}

// Turn left by about 90 degrees
void turnLeft90Degrees() {
  int targetTicks = TURN_90_TARGET_TICKS;

  resetEncoderCounts();
  startLeftTurn();

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
void turnRight90Degrees() {
  int targetTicks = TURN_90_TARGET_TICKS;

  resetEncoderCounts();
  startRightTurn();

  while (true) {
    unsigned long leftValue = getLeftTicks();

    // During a right turn, the left wheel is the moving wheel
    if (leftValue >= (unsigned long)targetTicks) {
      break;
    }
  }

  stopMotors();
}

///~~~~~~~~~~~~~~~~~~~~~~ ULTRASOUND SENSOR FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
// Send a 10 us trigger pulse and measure echo high time.
float getUltrasoundDuration() {
  digitalWrite(ULTRASOUND_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASOUND_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASOUND_TRIG_PIN, LOW);

  return pulseIn(ULTRASOUND_ECHO_PIN, HIGH);
}

float getUltrasoundDistance() {
  // 0.0343 cm/us is speed of sound. Is divided by 2 since it has to make a round trip
  float ultrasoundDuration = getUltrasoundDuration();
  float ultrasoundDistance = (ultrasoundDuration * 0.0343f) / 2.0f;

  Serial.print("Distance: ");
  Serial.println(ultrasoundDistance);

  return ultrasoundDistance;
}

boolean isObjectNearby() {
  float distance = getUltrasoundDistance();
  return distance > ULTRASOUND_DISTANCE_MIN && distance < ULTRASOUND_DISTANCE_MAX;
}
