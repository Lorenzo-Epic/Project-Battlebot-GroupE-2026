#include <string.h>

// Pin mapping.
const int LEFT_BACKWARD_PIN = 10;
const int LEFT_FORWARD_PIN = 5;
const int RIGHT_FORWARD_PIN = 6;
const int RIGHT_BACKWARD_PIN = 9;

const int ROTATION_LEFT_PIN = 2;
const int ROTATION_RIGHT_PIN = 3;

const int ULTRASOUND_TRIG_PIN = 11;
const int ULTRASOUND_ECHO_PIN = 12;

const int LIGHT_SENSOR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int weights[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Motor PWM calibration.
const int CALIBRATION_FORWARD_LEFT = 255;
const int CALIBRATION_BACKWARD_LEFT = 255;
const int CALIBRATION_FORWARD_RIGHT = 243;
const int CALIBRATION_BACKWARD_RIGHT = 210;

// Obstacle trigger range in cm.
const float ULTRASOUND_DISTANCE_MIN = 0.0f;
const float ULTRASOUND_DISTANCE_MAX = 25.0f;

// Encoder and wheel geometry.
const float WHEEL_DIAMETER_CM = 6.5f;
const int SLOTS_PER_REV = 20;
const int EDGES_PER_SLOT = 2;
const int TICKS_PER_REV = SLOTS_PER_REV * EDGES_PER_SLOT;
const float CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const float TICKS_PER_CM = (float)TICKS_PER_REV / CIRCUMFERENCE_CM;

// Ignore encoder edges that arrive too quickly.
const unsigned long EDGE_MIN_US = 150;

// Empirical turn calibration.
const float TURN_SLOTS_FOR_90_DEG = 8.0f;
const float TURN_TICKS_PER_DEG = (TURN_SLOTS_FOR_90_DEG * EDGES_PER_SLOT) / 90.0f;
const long TURN_SLOWDOWN_TICKS = 3;

// Lower PWM near turn target to reduce overshoot.
const int TURN_SLOW_LEFT_FORWARD = 150;
const int TURN_SLOW_LEFT_BACKWARD = 150;
const int TURN_SLOW_RIGHT_FORWARD = 145;
const int TURN_SLOW_RIGHT_BACKWARD = 135;

const unsigned long ULTRASOUND_TIMEOUT_US = 25000UL;

// Shared by ISRs and main loop.
volatile unsigned long g_leftTicks = 0;
volatile unsigned long g_rightTicks = 0;
volatile unsigned long g_lastLeftUs = 0;
volatile unsigned long g_lastRightUs = 0;

static inline long roundToLong(float x) {
  return (x >= 0.0f) ? (long)(x + 0.5f) : (long)(x - 0.5f);
}

// Count each valid encoder edge.
void isrLeft() {
  unsigned long now = micros();
  if (now - g_lastLeftUs >= EDGE_MIN_US) {
    g_leftTicks++;
    g_lastLeftUs = now;
  }
}

void isrRight() {
  unsigned long now = micros();
  if (now - g_lastRightUs >= EDGE_MIN_US) {
    g_rightTicks++;
    g_lastRightUs = now;
  }
}

// Motor control helpers.
void stopMotors() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);

  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
}

void driveForward() {
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);

  analogWrite(LEFT_FORWARD_PIN, CALIBRATION_FORWARD_LEFT);
  analogWrite(RIGHT_FORWARD_PIN, CALIBRATION_FORWARD_RIGHT);
}

void driveBackward() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);

  analogWrite(LEFT_BACKWARD_PIN, CALIBRATION_BACKWARD_LEFT);
  analogWrite(RIGHT_BACKWARD_PIN, CALIBRATION_BACKWARD_RIGHT);
}

void turnLeftInPlace() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);

  analogWrite(LEFT_BACKWARD_PIN, CALIBRATION_BACKWARD_LEFT);
  analogWrite(RIGHT_FORWARD_PIN, CALIBRATION_FORWARD_RIGHT);
}

void turnRightInPlace() {
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);

  analogWrite(LEFT_FORWARD_PIN, CALIBRATION_FORWARD_LEFT);
  analogWrite(RIGHT_BACKWARD_PIN, CALIBRATION_BACKWARD_RIGHT);
}

// Read tick counters atomically because ISRs update them.
void readTicks(unsigned long &l, unsigned long &r) {
  noInterrupts();
  l = g_leftTicks;
  r = g_rightTicks;
  interrupts();
}

void resetTicks() {
  noInterrupts();
  g_leftTicks = 0;
  g_rightTicks = 0;
  g_lastLeftUs = 0;
  g_lastRightUs = 0;
  interrupts();
}

// Send a 10 us trigger pulse and measure echo high time.
float getUltrasoundDuration() {
  digitalWrite(ULTRASOUND_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASOUND_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASOUND_TRIG_PIN, LOW);

  return pulseIn(ULTRASOUND_ECHO_PIN, HIGH, ULTRASOUND_TIMEOUT_US);
}

float getUltrasoundDistance() {
  // 0.0343 cm/us is speed of sound in air. Divide by 2 for round trip.
  float ultrasoundDuration = getUltrasoundDuration();
  float ultrasoundDistance = (ultrasoundDuration * 0.0343f) / 2.0f;

  Serial.print("Distance: ");
  Serial.println(ultrasoundDistance);

  return ultrasoundDistance;
}


void move(float amount, const char *direction) {
  // Input units:
  // - "forward" / "backward": amount is cm.
  // - "left" / "right": amount is robot deg.
  // Conversion math:
  // - Drive ticks = cm * TICKS_PER_CM.
  // - Turn ticks = deg * TURN_TICKS_PER_DEG (empirical calibration).
  const bool IS_DRIVE = (strcmp(direction, "forward") == 0) || (strcmp(direction, "backward") == 0);
  const bool IS_TURN = (strcmp(direction, "left") == 0) || (strcmp(direction, "right") == 0);
  if (!IS_DRIVE && !IS_TURN) return;

  const float ABS_AMOUNT = (amount >= 0.0f) ? amount : -amount;

  long targetTicks = 0;
  if (IS_DRIVE) {
    targetTicks = roundToLong(ABS_AMOUNT * TICKS_PER_CM);
  } else {
    targetTicks = roundToLong(ABS_AMOUNT * TURN_TICKS_PER_DEG);
  }

  if (targetTicks <= 0) return;

  // Measure only this move command.
  resetTicks();

  // Apply direction-specific motor pattern.
  if (strcmp(direction, "forward") == 0) {
    driveForward();
  } else if (strcmp(direction, "backward") == 0) {
    driveBackward();
  } else if (strcmp(direction, "left") == 0) {
    turnLeftInPlace();
  } else if (strcmp(direction, "right") == 0) {
    turnRightInPlace();
  }

  // Timeout scales with target distance/angle to fail safe on bad sensor reads.
  const unsigned long START_MS = millis();
  const unsigned long TIMEOUT_MS = IS_TURN
    ? (700UL + (unsigned long)targetTicks * 80UL)
    : (1500UL + (unsigned long)targetTicks * 120UL);

  bool slowPhase = false;

  while (true) {
    unsigned long l, r;
    readTicks(l, r);

    // Average both wheels so single-wheel bias does not dominate stop timing.
    unsigned long progress = (l + r) / 2;
    long remainingTicks = targetTicks - (long)progress;

    // For turns, reduce PWM for the final ticks to reduce momentum overshoot.
    if (IS_TURN && !slowPhase && remainingTicks <= TURN_SLOWDOWN_TICKS) {
      slowPhase = true;

      if (strcmp(direction, "left") == 0) {
        analogWrite(LEFT_FORWARD_PIN, 0);
        analogWrite(RIGHT_BACKWARD_PIN, 0);
        analogWrite(LEFT_BACKWARD_PIN, TURN_SLOW_LEFT_BACKWARD);
        analogWrite(RIGHT_FORWARD_PIN, TURN_SLOW_RIGHT_FORWARD);
      } else if (strcmp(direction, "right") == 0) {
        analogWrite(LEFT_BACKWARD_PIN, 0);
        analogWrite(RIGHT_FORWARD_PIN, 0);
        analogWrite(LEFT_FORWARD_PIN, TURN_SLOW_LEFT_FORWARD);
        analogWrite(RIGHT_BACKWARD_PIN, TURN_SLOW_RIGHT_BACKWARD);
      }
    }

    if ((long)progress >= targetTicks) break;
    if (millis() - START_MS > TIMEOUT_MS) break;
  }

  stopMotors();

  // Debug print helps tune calibration constants.
  unsigned long l, r;
  readTicks(l, r);
  Serial.print("Move ");
  Serial.print(direction);
  Serial.print(" targetTicks=");
  Serial.print(targetTicks);
  Serial.print(" L=");
  Serial.print(l);
  Serial.print(" R=");
  Serial.println(r);
}

void readSensors() {
  long sum = 0;
  long total = 0;

  for (int i = 0; i < 8; i++) {
    int raw = analogRead(LIGHT_SENSOR_PINS[i]);

    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(raw);
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  pinMode(ROTATION_LEFT_PIN, INPUT_PULLUP);
  pinMode(ROTATION_RIGHT_PIN, INPUT_PULLUP);

  pinMode(ULTRASOUND_TRIG_PIN, OUTPUT);
  pinMode(ULTRASOUND_ECHO_PIN, INPUT);

  const int LEFT_INTERRUPT = digitalPinToInterrupt(ROTATION_LEFT_PIN);
  const int RIGHT_INTERRUPT = digitalPinToInterrupt(ROTATION_RIGHT_PIN);
  if (LEFT_INTERRUPT == NOT_AN_INTERRUPT || RIGHT_INTERRUPT == NOT_AN_INTERRUPT) {
    Serial.println("ERROR: rotation pins must support external interrupts.");
    stopMotors();
    while (true) {
      delay(1000);
    }
  }

  attachInterrupt(LEFT_INTERRUPT, isrLeft, CHANGE);
  attachInterrupt(RIGHT_INTERRUPT, isrRight, CHANGE);

  for (int i = 0; i < 8; i++) pinMode(LIGHT_SENSOR_PINS[i], INPUT);

  stopMotors();
}

void loop() {
  readSensors();
  delay(1000);
}
