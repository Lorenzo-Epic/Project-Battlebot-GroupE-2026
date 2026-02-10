const int leftBackward = 3;
const int leftForward  = 5;

const int rightForward  = 6;
const int rightBackward = 9;

// rotation sensors (pins)
const int rotationLeft  = 2;
const int rotationRight = 4;

const int calibrationForwardLeft = 255;
const int calibrationBackwardLeft = 255;

const int calibrationForwardRight = 242;
const int calibrationBackwardRight = 220;

// --- wheel/encoder constants ---
const float WHEEL_DIAMETER_CM = 6.5;
const int   SLOTS_PER_REV     = 20;   // 20 slits
const int   EDGES_PER_SLOT    = 1;    // counting only 1->0 OR 0->1 (not both)
const float CIRCUMFERENCE_CM  = PI * WHEEL_DIAMETER_CM;
const float TICKS_PER_CM      = (SLOTS_PER_REV * EDGES_PER_SLOT) / CIRCUMFERENCE_CM;

// --- turn calibration (in-place) ---
// You said: 90° turn ~= 2.5 "1-pulses" per wheel with 1× counting.
// With 2× (both edges), that's 5 ticks.
const float TURN_TICKS_90      = 5.0;
const float TURN_TICKS_PER_DEG = TURN_TICKS_90 / 90.0;

// noise gate (optical sensors usually clean, but this helps)
const unsigned long EDGE_MIN_US = 200;

void setup() {
  Serial.begin(9600);

  //A1 -> 3
  //A2 -> 5
  //B1 -> 6
  //B2 -> 9
  //R1 -> 10
  //R2 -> 11

  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);

  pinMode(rotationLeft, INPUT_PULLUP);
  pinMode(rotationRight, INPUT_PULLUP);
}

void stopMotors() {
  analogWrite(leftForward, 0);
  analogWrite(leftBackward, 0);
  analogWrite(rightForward, 0);
  analogWrite(rightBackward, 0);

  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
}

void driveForward() {
  analogWrite(leftForward, calibrationForwardLeft);
  digitalWrite(leftBackward, LOW);

  analogWrite(rightForward, calibrationForwardRight);
  digitalWrite(rightBackward, LOW);
}

void driveBackward() {
  analogWrite(leftBackward, calibrationBackwardLeft);
  digitalWrite(leftForward, LOW);
  
  analogWrite(rightBackward, calibrationBackwardRight);
  digitalWrite(rightForward, LOW);
}

void driveLeftTurn() {
  // rotate left in place: left wheel backward, right wheel forward
  analogWrite(leftBackward, calibrationBackwardLeft);
  digitalWrite(leftForward, LOW);

  analogWrite(rightForward, calibrationForwardRight);
  digitalWrite(rightBackward, LOW);
}

void driveRightTurn() {
  // rotate right in place: left wheel forward, right wheel backward
  analogWrite(leftForward, calibrationForwardLeft);
  digitalWrite(leftBackward, LOW);

  analogWrite(rightBackward, calibrationBackwardRight);
  digitalWrite(rightForward, LOW);
}

void move(int amount, String direction) {
  bool isTurn = (direction == "left" || direction == "right");
  if (amount <= 0) return;

  // amount = cm for forward/backward, degrees for left/right
  float targetTicksF = isTurn
    ? (amount * TURN_TICKS_PER_DEG)
    : (amount * TICKS_PER_CM);

  long targetTicks = (long)lround(targetTicksF);
  if (targetTicks <= 0) targetTicks = 1;

  long leftTicks = 0;
  long rightTicks = 0;

  int lastL = digitalRead(rotationLeft);
  int lastR = digitalRead(rotationRight);

  unsigned long lastEdgeL = 0;
  unsigned long lastEdgeR = 0;

  if (direction == "forward")       driveForward();
  else if (direction == "backward") driveBackward();
  else if (direction == "left")     driveLeftTurn();
  else if (direction == "right")    driveRightTurn();
  else return; // invalid string

  unsigned long startMs = millis();
  const unsigned long TIMEOUT_MS = 5000UL + (unsigned long)amount * (isTurn ? 40UL : 50UL);

  while (true) {
    int curL = digitalRead(rotationLeft);
    int curR = digitalRead(rotationRight);
    unsigned long nowUs = micros();

    // 2× counting: tick on ANY state change
    if (curL != lastL) {
      if (nowUs - lastEdgeL >= EDGE_MIN_US) {
        leftTicks++;
        lastEdgeL = nowUs;
      }
    }
    if (curR != lastR) {
      if (nowUs - lastEdgeR >= EDGE_MIN_US) {
        rightTicks++;
        lastEdgeR = nowUs;
      }
    }

    lastL = curL;
    lastR = curR;

    if (!isTurn) {
      long avg = (leftTicks + rightTicks) / 2;
      if (avg >= targetTicks) break;
    } else {
      // turning: require BOTH wheels to hit target
      if (leftTicks >= targetTicks && rightTicks >= targetTicks) break;
    }

    if (millis() - startMs > TIMEOUT_MS) break;
  }

  stopMotors();
}

void loop() {
  move(90, "left");      // 90 degrees left
  delay(500);
  move(90, "right");     // 90 degrees right  
}
