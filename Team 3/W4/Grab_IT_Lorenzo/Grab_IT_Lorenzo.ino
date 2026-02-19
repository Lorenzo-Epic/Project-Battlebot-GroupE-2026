#define SERVO 11
#define GRIPPER_OPEN 1820
#define GRIPPER_CLOSE 1000

// Pin mapping.
#define LEFT_BACKWARD_PIN 10
#define LEFT_FORWARD_PIN 5
#define RIGHT_FORWARD_PIN 6
#define RIGHT_BACKWARD_PIN 9

#define ROTATION_LEFT_PIN 2
#define ROTATION_RIGHT_PIN 3

// Motor PWM calibration.
#define CALIBRATION_FORWARD_LEFT 255
#define CALIBRATION_BACKWARD_LEFT 255
#define CALIBRATION_FORWARD_RIGHT 238
#define CALIBRATION_BACKWARD_RIGHT 210

// Encoder and wheel geometry.
#define WHEEL_DIAMETER_CM 6.5f
#define SLOTS_PER_REV 20
#define EDGES_PER_SLOT 2
#define TICKS_PER_REV SLOTS_PER_REV * EDGES_PER_SLOT
#define CIRCUMFERENCE_CM PI * WHEEL_DIAMETER_CM
#define TICKS_PER_CM (float)TICKS_PER_REV / CIRCUMFERENCE_CM

// Ignore encoder edges that arrive too quickly.
#define EDGE_MIN_US 150

// Turn calibration.
#define TURN_SLOTS_FOR_90_DEG 8.0f
#define TURN_TICKS_PER_DEG (TURN_SLOTS_FOR_90_DEG * EDGES_PER_SLOT) / 90.0f
#define TURN_SLOWDOWN_TICKS 3

// Lower PWM near turn target to reduce overshoot.
#define TURN_SLOW_LEFT_FORWARD 150
#define TURN_SLOW_LEFT_BACKWARD 150
#define TURN_SLOW_RIGHT_FORWARD 145
#define TURN_SLOW_RIGHT_BACKWARD 135

#define ULTRASOUND_TIMEOUT_US 25000UL

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

// Motor control
///0 = stop
///1 = forward
///2 = backward
///3 = turn left
///4 = turn right
void drive(int driveOption) {
  if (driveOption == 0) {
    analogWrite(LEFT_FORWARD_PIN, 0);
    analogWrite(LEFT_BACKWARD_PIN, 0);
    analogWrite(RIGHT_FORWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, 0);
  
    digitalWrite(LEFT_FORWARD_PIN, LOW);
    digitalWrite(LEFT_BACKWARD_PIN, LOW);
    digitalWrite(RIGHT_FORWARD_PIN, LOW);
    digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  }
  if (driveOption == 1){
    analogWrite(LEFT_BACKWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, 0);
  
    analogWrite(LEFT_FORWARD_PIN, CALIBRATION_FORWARD_LEFT);
    analogWrite(RIGHT_FORWARD_PIN, CALIBRATION_FORWARD_RIGHT);
  }
  if (driveOption == 2){
    analogWrite(LEFT_FORWARD_PIN, 0);
    analogWrite(RIGHT_FORWARD_PIN, 0);
  
    analogWrite(LEFT_BACKWARD_PIN, CALIBRATION_BACKWARD_LEFT);
    analogWrite(RIGHT_BACKWARD_PIN, CALIBRATION_BACKWARD_RIGHT);
  }
  if (driveOption == 3){
    analogWrite(LEFT_FORWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, 0);
  
    analogWrite(LEFT_BACKWARD_PIN, CALIBRATION_BACKWARD_LEFT);
    analogWrite(RIGHT_FORWARD_PIN, CALIBRATION_FORWARD_RIGHT);
  }
  if (driveOption == 4){
    analogWrite(LEFT_BACKWARD_PIN, 0);
    analogWrite(RIGHT_FORWARD_PIN, 0);
  
    analogWrite(LEFT_FORWARD_PIN, CALIBRATION_FORWARD_LEFT);
    analogWrite(RIGHT_BACKWARD_PIN, CALIBRATION_BACKWARD_RIGHT);
  }
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
    drive(1);
  } else if (strcmp(direction, "backward") == 0) {
    drive(2);
  } else if (strcmp(direction, "left") == 0) {
    drive(3);
  } else if (strcmp(direction, "right") == 0) {
    drive(4);
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

  drive(0);
}

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  pinMode(ROTATION_LEFT_PIN, INPUT_PULLUP);
  pinMode(ROTATION_RIGHT_PIN, INPUT_PULLUP);

  const int LEFT_INTERRUPT = digitalPinToInterrupt(ROTATION_LEFT_PIN);
  const int RIGHT_INTERRUPT = digitalPinToInterrupt(ROTATION_RIGHT_PIN);
  if (LEFT_INTERRUPT == NOT_AN_INTERRUPT || RIGHT_INTERRUPT == NOT_AN_INTERRUPT) {
    Serial.println("ERROR: rotation pins must support external interrupts.");
    drive(0);
    while (true) {
      delay(1000);
    }
  }

  attachInterrupt(LEFT_INTERRUPT, isrLeft, CHANGE);
  attachInterrupt(RIGHT_INTERRUPT, isrRight, CHANGE);

  pinMode(SERVO, OUTPUT);
  digitalWrite(SERVO, 0);

  drive(0);
}

void openGripper() {
  for (int i = 0; i < 1000; i++) {
    gripper(GRIPPER_OPEN);
    delay(1);
  }
}

void closeGripper() {
  for (int i = 0; i < 1000; i++) {
    gripper(GRIPPER_CLOSE);
    delay(1);
  }
}

void gripper(int newPulse) {
  static unsigned long timer;
  static int pulse;
  if (millis() > timer) {
    if (newPulse > 0) {
      pulse = newPulse;
    }
    digitalWrite(SERVO, 1); ///HIGH
    delayMicroseconds(pulse);
    digitalWrite(SERVO, 0);
    timer = timer + 20;
  }
}

void loop() {
///make sure its closed first
  closeGripper();
  //open, wait a second, close, wait a second, open
  openGripper();
  delay(1000);
  closeGripper();
  delay(1000);
  openGripper();

///drive towards cone
  move(25, "forward");

///grab cone
  closeGripper();

///drive 25 more cm
  move(25, "forward");
}
