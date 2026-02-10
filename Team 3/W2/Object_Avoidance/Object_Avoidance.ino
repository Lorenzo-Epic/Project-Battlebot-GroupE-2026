const int leftBackward = 3;
const int leftForward  = 5;

const int rightForward  = 6;
const int rightBackward = 9;

// rotation sensors (pins)
const int rotationLeft  = 2;
const int rotationRight = 4;

//ultrasound sensor
const int ultrasoundTrig = 10;
const int ultrasoundEcho = 11;
//floats that store the duration and distance of the pulses
float ultrasoundDuration, ultrasoundDistance;

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

  pinMode(ultrasoundTrig, OUTPUT);  
  pinMode(ultrasoundEcho, INPUT);

}

//Movement Functions

void stopMotors() {
  analogWrite(leftBackward, 0);
  digitalWrite(leftForward, LOW);

  analogWrite(rightBackward, 0);
  digitalWrite(rightForward, LOW);
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

void move(int lengthCm, String direction) {
  long targetTicks = lround(lengthCm * TICKS_PER_CM);
  if (targetTicks <= 0) return;

  long leftTicks = 0;
  long rightTicks = 0;

  int lastL = digitalRead(rotationLeft);
  int lastR = digitalRead(rotationRight);

  unsigned long lastEdgeL = 0;
  unsigned long lastEdgeR = 0;

  if (direction == "forward") {
    driveForward();
  } else if (direction == "backward") {
    driveBackward();
  }
  

  // Optional: timeout so you don't deadlock forever if a sensor fails
  unsigned long startMs = millis();
  const unsigned long TIMEOUT_MS = 5000 + (unsigned long)lengthCm * 50; // crude but practical

  while (true) {
    int curL = digitalRead(rotationLeft);
    int curR = digitalRead(rotationRight);
    unsigned long nowUs = micros();

    // Count falling edges (HIGH -> LOW). If your sensor is inverted, swap the condition.
    if (lastL == HIGH && curL == LOW) {
      if (nowUs - lastEdgeL >= EDGE_MIN_US) {
        leftTicks++;
        lastEdgeL = nowUs;
        // Serial.println(leftTicks); // debug only on edge (no spam)
      }
    }

    if (lastR == HIGH && curR == LOW) {
      if (nowUs - lastEdgeR >= EDGE_MIN_US) {
        rightTicks++;
        lastEdgeR = nowUs;
      }
    }

    lastL = curL;
    lastR = curR;

    // Stop condition: use average so one wheel notperfectly matched still works.
    long avg = (leftTicks + rightTicks) / 2;
    if (avg >= targetTicks) break;

    if (millis() - startMs > TIMEOUT_MS) break;
  }

  stopMotors();
}

///Sensor functions

float getUltrasoundDuration() {
  digitalWrite(ultrasoundTrig, LOW);  
  delayMicroseconds(2);  
  digitalWrite(ultrasoundTrig, HIGH);  
  delayMicroseconds(10);  
  digitalWrite(ultrasoundTrig, LOW); 

  return pulseIn(ultrasoundEcho, HIGH);
}

float getUltrasoundDistance() {
  float ultrasoundDuration = getUltrasoundDuration();
  float ultrasoundDistance = (ultrasoundDuration*.0343)/2;  
  Serial.print("Distance: ");  
  Serial.println(ultrasoundDistance);  
  return ultrasoundDistance;
}

void loop() {
//  /if value greater than 1000, ignore
///if value is 10, go into object avoidance, move around the object and continue straight(could be rotate left, forward, right, forward, right, forward, left, forward continuing

  getUltrasoundDistance();
  delay(100);  


}
