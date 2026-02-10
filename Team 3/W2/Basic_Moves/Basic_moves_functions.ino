// Motor pins
//A1 -> 3
//A2 -> 5
//B1 -> 6
//B2 -> 9
//R1 -> 10
//R2 -> 11

const int leftBackward = 3;
const int leftForward  = 5;

const int rightForward  = 6;
const int rightBackward = 9;

// Rotation
const int rotationLeft  = 10;
const int rotationRight = 11;

// Calibration values
const int calibrationForwardLeft  = 255;
const int calibrationBackwardLeft = 255;

const int calibrationForwardRight  = 242;
const int calibrationBackwardRight = 220;

// Encoder calibration (change after tests)
const int ticksPerMeter = 100;
const int ticks90Turn = 30;

// setud

void setup() {
  Serial.begin(9600);

  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);

  pinMode(rotationLeft, INPUT_PULLUP);
  pinMode(rotationRight, INPUT_PULLUP);
}

// motor helper

void stopMotors() {
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
}

// functions for movement

void moveForward1m() {
  int ticks = 0;

  analogWrite(leftForward, calibrationForwardLeft);
  analogWrite(rightForward, calibrationForwardRight);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightBackward, LOW);

  while (ticks < ticksPerMeter) {
    if (digitalRead(rotationLeft) == LOW) {
      ticks++;
      while (digitalRead(rotationLeft) == LOW); // debounce
    }
  }

  stopMotors();
}

void moveBackward1m() {
  int ticks = 0;

  analogWrite(leftBackward, calibrationBackwardLeft);
  analogWrite(rightBackward, calibrationBackwardRight);
  digitalWrite(leftForward, LOW);
  digitalWrite(rightForward, LOW);

  while (ticks < ticksPerMeter) {
    if (digitalRead(rotationLeft) == LOW) {
      ticks++;
      while (digitalRead(rotationLeft) == LOW);
    }
  }

  stopMotors();
}

// left wheel stop, right wheel moves left
void turnLeft90() {
  int ticks = 0;

  stopMotors();
  analogWrite(rightForward, calibrationForwardRight);
  digitalWrite(rightBackward, LOW);

  while (ticks < ticks90Turn) {
    if (digitalRead(rotationRight) == LOW) {
      ticks++;
      while (digitalRead(rotationRight) == LOW);
    }
  }

  stopMotors();
}

// right wheel stop left wheel moves
void turnRight90() {
  int ticks = 0;

  stopMotors();
  analogWrite(leftForward, calibrationForwardLeft);
  digitalWrite(leftBackward, LOW);

  while (ticks < ticks90Turn) {
    if (digitalRead(rotationLeft) == LOW) {
      ticks++;
      while (digitalRead(rotationLeft) == LOW);
    }
  }

  stopMotors();
}

// main loop

void loop() {
  moveForward1m();
  delay(500);

  moveBackward1m();
  delay(500);

  turnLeft90();
  delay(500);

  turnRight90();
  delay(2000); // small pause before repeating
}
