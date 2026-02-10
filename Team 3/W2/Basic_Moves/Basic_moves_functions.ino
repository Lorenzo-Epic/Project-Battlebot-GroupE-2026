// Motor pins
//A1 -> 3
//A2 -> 5
//B1 -> 6
//B2 -> 9
//R1 -> 10
//R2 -> 11

// motor pins
const int leftBackward  = 3;
const int leftForward   = 5;
const int rightForward  = 6;
const int rightBackward = 9;

// rotation sensors
const int rotationLeft  = 2;
const int rotationRight = 4;

// calibration values
const int calibrationForwardLeft  = 255;
const int calibrationBackwardLeft = 255;
const int calibrationForwardRight = 242;
const int calibrationBackwardRight = 220;

// wheel and robot math
const float wheelDiameterCm = 6.5;   // wheel diameter
const int slotsPerRev       = 20;    // encoder slots per wheel
const float wheelBaseCm     = 12.0;  // distance between wheels

const float wheelCircumferenceCm = PI * wheelDiameterCm;
const float distancePerTickCm    = wheelCircumferenceCm / slotsPerRev;

// encoder calibration w math
const int ticksPerMeter = lround(100.0 / distancePerTickCm);  
const int ticks90Turn   = lround((PI * wheelBaseCm / 4.0) / distancePerTickCm); 

void setup() {
  Serial.begin(9600);

  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);

  pinMode(rotationLeft, INPUT_PULLUP);
  pinMode(rotationRight, INPUT_PULLUP);
}

void stopMotors() {
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
}

// movement functions

void moveForward1m() {
  int ticks = 0;

  analogWrite(leftForward, calibrationForwardLeft);
  analogWrite(rightForward, calibrationForwardRight);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightBackward, LOW);

  while (ticks < ticksPerMeter) {
    if (digitalRead(rotationLeft) == LOW || digitalRead(rotationRight) == LOW) {
      ticks++;
      while (digitalRead(rotationLeft) == LOW || digitalRead(rotationRight) == LOW); // debounce
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
    if (digitalRead(rotationLeft) == LOW || digitalRead(rotationRight) == LOW) {
      ticks++;
      while (digitalRead(rotationLeft) == LOW || digitalRead(rotationRight) == LOW); // debounce
    }
  }

  stopMotors();
}

// rotation functions

// spin left 90 left wheel backward, right wheel forward
void rotateLeft90() {
  int ticks = 0;

  analogWrite(leftBackward, calibrationBackwardLeft);
  digitalWrite(leftForward, LOW);

  analogWrite(rightForward, calibrationForwardRight);
  digitalWrite(rightBackward, LOW);

  while (ticks < ticks90Turn) {
    if (digitalRead(rotationLeft) == LOW || digitalRead(rotationRight) == LOW) {
      ticks++;
      while (digitalRead(rotationLeft) == LOW || digitalRead(rotationRight) == LOW); // debounce
    }
  }

  stopMotors();
}

// spin right 90 left wheel forward, right wheel backward
void rotateRight90() {
  int ticks = 0;

  analogWrite(leftForward, calibrationForwardLeft);
  digitalWrite(leftBackward, LOW);

  analogWrite(rightBackward, calibrationBackwardRight);
  digitalWrite(rightForward, LOW);

  while (ticks < ticks90Turn) {
    if (digitalRead(rotationLeft) == LOW || digitalRead(rotationRight) == LOW) {
      ticks++;
      while (digitalRead(rotationLeft) == LOW || digitalRead(rotationRight) == LOW); // debounce
    }
  }

  stopMotors();
}

void loop() {
  moveForward1m();
  delay(500);

  moveBackward1m();
  delay(500);

  rotateLeft90();
  delay(500);

  rotateRight90();
  delay(2000);
}
