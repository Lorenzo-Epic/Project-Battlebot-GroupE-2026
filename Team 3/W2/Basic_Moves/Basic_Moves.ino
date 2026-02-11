#include <Arduino.h>
#include <string.h>

// -------------------- Pins --------------------
// NOTE: Nano PWM pins are 3,5,6,9,10,11. Pin 10 is PWM-capable.
const uint8_t leftBackward  = 10;  // PWM
const uint8_t leftForward   = 5;   // PWM
const uint8_t rightForward  = 6;   // PWM
const uint8_t rightBackward = 9;   // PWM

// Rotation sensors (must be interrupt pins for attachInterrupt on ATmega328P)
const uint8_t rotationLeft  = 2;   // INT0
const uint8_t rotationRight = 3;   // INT1

// -------------------- Calibration --------------------
const uint8_t calibrationForwardLeft   = 255;
const uint8_t calibrationBackwardLeft  = 255;
const uint8_t calibrationForwardRight  = 242;
const uint8_t calibrationBackwardRight = 220;

// -------------------- Encoder / geometry --------------------
const float WHEEL_DIAMETER_CM = 7.0f;
const int   SLOTS_PER_REV     = 20;

// Count BOTH edges (CHANGE) => 2 edges per slit => supports half-slit increments
const int   EDGES_PER_SLOT    = 2;
const int   TICKS_PER_REV     = SLOTS_PER_REV * EDGES_PER_SLOT;

const float CIRCUMFERENCE_CM  = PI * WHEEL_DIAMETER_CM;
const float TICKS_PER_CM      = (float)TICKS_PER_REV / CIRCUMFERENCE_CM;

// Edge noise gate (optional; optical sensors usually clean)
const unsigned long EDGE_MIN_US = 150;

// Turning calibration: you claim 90° robot turn ~= 2.5 slit-triggers per wheel.
// With EDGES_PER_SLOT=2, that’s 2.5 * 2 = 5 ticks for 90°.
const float TURN_SLOTS_FOR_90_DEG = 6.0f;
const float TURN_TICKS_PER_DEG    = (TURN_SLOTS_FOR_90_DEG * EDGES_PER_SLOT) / 90.0f;

// -------------------- Interrupt tick counters --------------------
volatile unsigned long g_leftTicks  = 0;
volatile unsigned long g_rightTicks = 0;

volatile unsigned long g_lastLeftUs  = 0;
volatile unsigned long g_lastRightUs = 0;

static inline long roundToLong(float x) {
  return (x >= 0.0f) ? (long)(x + 0.5f) : (long)(x - 0.5f);
}

// ISR: count every CHANGE (both edges)
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

// -------------------- Motor helpers --------------------
void stopMotors() {
  // Set PWM duty to 0 (do NOT rely on digitalWrite to stop PWM cleanly)
  analogWrite(leftForward, 0);
  analogWrite(leftBackward, 0);
  analogWrite(rightForward, 0);
  analogWrite(rightBackward, 0);

  // Force low for safety
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
}

void driveForward() {
  // Opposite direction pins off
  analogWrite(leftBackward, 0);
  analogWrite(rightBackward, 0);

  // PWM drive pins (NO digitalWrite after analogWrite, or PWM gets cancelled)
  analogWrite(leftForward, calibrationForwardLeft);
  analogWrite(rightForward, calibrationForwardRight);
}

void driveBackward() {
  analogWrite(leftForward, 0);
  analogWrite(rightForward, 0);

  analogWrite(leftBackward, calibrationBackwardLeft);
  analogWrite(rightBackward, calibrationBackwardRight);
}

void turnLeftInPlace() {
  // left backward, right forward
  analogWrite(leftForward, 0);
  analogWrite(rightBackward, 0);

  analogWrite(leftBackward, calibrationBackwardLeft);
  analogWrite(rightForward, calibrationForwardRight);
}

void turnRightInPlace() {
  // left forward, right backward
  analogWrite(leftBackward, 0);
  analogWrite(rightForward, 0);

  analogWrite(leftForward, calibrationForwardLeft);
  analogWrite(rightBackward, calibrationBackwardRight);
}

// Read tick counters atomically
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

// amount:
// - forward/backward => cm
// - left/right       => robot degrees (using your 2.5-slots-per-90deg calibration)
void move(float amount, const char *direction) {
  const bool isDrive = (strcmp(direction, "forward") == 0) || (strcmp(direction, "backward") == 0);
  const bool isTurn  = (strcmp(direction, "left") == 0)    || (strcmp(direction, "right") == 0);
  if (!isDrive && !isTurn) return;

  const float absAmount = (amount >= 0.0f) ? amount : -amount;

  long targetTicks = 0;
  if (isDrive) targetTicks = roundToLong(absAmount * TICKS_PER_CM);
  else         targetTicks = roundToLong(absAmount * TURN_TICKS_PER_DEG);

  if (targetTicks <= 0) return;

  resetTicks();

  // Start motion
  if      (strcmp(direction, "forward") == 0)  driveForward();
  else if (strcmp(direction, "backward") == 0) driveBackward();
  else if (strcmp(direction, "left") == 0)     turnLeftInPlace();
  else if (strcmp(direction, "right") == 0)    turnRightInPlace();

  const unsigned long startMs = millis();
  const unsigned long TIMEOUT_MS = 1500UL + (unsigned long)targetTicks * 120UL;

  while (true) {
    unsigned long l, r;
    readTicks(l, r);

    // Drive: average progress; Turn: require both wheels to reach target (min)
    unsigned long progress = isTurn ? ((l < r) ? l : r) : ((l + r) / 2);

    if ((long)progress >= targetTicks) break;
    if (millis() - startMs > TIMEOUT_MS) break;
  }

  stopMotors();

  // Debug
  unsigned long l, r;
  readTicks(l, r);
  Serial.print("Move "); Serial.print(direction);
  Serial.print(" targetTicks="); Serial.print(targetTicks);
  Serial.print(" L="); Serial.print(l);
  Serial.print(" R="); Serial.println(r);
}

void setup() {
  Serial.begin(9600);

  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);

  pinMode(rotationLeft, INPUT_PULLUP);
  pinMode(rotationRight, INPUT_PULLUP);

  // Interrupts on pins 2 and 3 (Nano/ATmega328P). If pins are changed, validate first.
  const int leftInterrupt  = digitalPinToInterrupt(rotationLeft);
  const int rightInterrupt = digitalPinToInterrupt(rotationRight);
  if (leftInterrupt == NOT_AN_INTERRUPT || rightInterrupt == NOT_AN_INTERRUPT) {
    Serial.println("ERROR: rotationLeft/rotationRight must map to interrupt-capable pins.");
    stopMotors();
    while (true) { delay(1000); }
  }
  attachInterrupt(leftInterrupt,  isrLeft,  CHANGE);
  attachInterrupt(rightInterrupt, isrRight, CHANGE);

  stopMotors();
}

void loop() {
  move(100, "forward");
  delay(1000);

  move(100, "backward");
  delay(1000);

  move(90, "left");
  delay(1000);

  move(90, "right");
  delay(1000);
}
