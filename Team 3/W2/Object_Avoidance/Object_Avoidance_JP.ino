//A1 -> 3
//A2 -> 5
//B1 -> 6
//B2 -> 9
//R1 -> 10
//R2 -> 11

const int leftBackward = 10;
const int leftForward  = 5;
const int rightForward = 6;
const int rightBackward = 9;

// rotation sensors (pins)
const int rotationLeft  = 2;
const int rotationRight = 3;

// ultrasound sensor
const int ultrasoundTrig = 12;
const int ultrasoundEcho = 13;

float ultrasoundDuration, ultrasoundDistance;

const int calibrationForwardLeft = 255;
const int calibrationBackwardLeft = 255;
const int calibrationForwardRight = 242;
const int calibrationBackwardRight = 220;

// wheel/encoder constants
const float WHEEL_DIAMETER_CM = 6.5;
const int   SLOTS_PER_REV     = 20;
const int   EDGES_PER_SLOT    = 1;
const float CIRCUMFERENCE_CM  = PI * WHEEL_DIAMETER_CM;
const float TICKS_PER_CM      = (SLOTS_PER_REV * EDGES_PER_SLOT) / CIRCUMFERENCE_CM;

const float WHEEL_BASE_CM = 12.0;   // distance between wheels

// noise gate
const unsigned long EDGE_MIN_US = 200;

void setup() 
{

  Serial.begin(9600);

  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);

  pinMode(rotationLeft, INPUT_PULLUP);
  pinMode(rotationRight, INPUT_PULLUP);

  pinMode(ultrasoundTrig, OUTPUT);
  pinMode(ultrasoundEcho, INPUT);
}

void stopMotors() 
{
  analogWrite(leftBackward, 0);
  digitalWrite(leftForward, LOW);

  analogWrite(rightBackward, 0);
  digitalWrite(rightForward, LOW);
}

void driveForward() 
{
  analogWrite(leftForward, calibrationForwardLeft);
  digitalWrite(leftBackward, LOW);

  analogWrite(rightForward, calibrationForwardRight);
  digitalWrite(rightBackward, LOW);
}

void driveBackward() 
{
  analogWrite(leftBackward, calibrationBackwardLeft);
  digitalWrite(leftForward, LOW);

  analogWrite(rightBackward, calibrationBackwardRight);
  digitalWrite(rightForward, LOW);
}

void move(int lengthCm, String direction) 
{

  long targetTicks = lround(lengthCm * TICKS_PER_CM);
  if (targetTicks <= 0) return;

  long leftTicks = 0;
  long rightTicks = 0;

  int lastL = digitalRead(rotationLeft);
  int lastR = digitalRead(rotationRight);

  unsigned long lastEdgeL = 0;
  unsigned long lastEdgeR = 0;

  if (direction == "forward") driveForward();
  if (direction == "backward") driveBackward();

  while (true) 
  {

    int curL = digitalRead(rotationLeft);
    int curR = digitalRead(rotationRight);
    unsigned long nowUs = micros();

    if (lastL == HIGH && curL == LOW && nowUs - lastEdgeL >= EDGE_MIN_US) 
    {
      leftTicks++;
      lastEdgeL = nowUs;
    }

    if (lastR == HIGH && curR == LOW && nowUs - lastEdgeR >= EDGE_MIN_US) 
    {
      rightTicks++;
      lastEdgeR = nowUs;
    }

    lastL = curL;
    lastR = curR;

    long avg = (leftTicks + rightTicks) / 2;
    if (avg >= targetTicks) break;
  }

  stopMotors();
}

// turn function with encoders
void turnDegrees(int degrees) 
{

  float arcLengthCm = (PI * WHEEL_BASE_CM) * (abs(degrees) / 360.0);
  long targetTicks = lround(arcLengthCm * TICKS_PER_CM);

  long leftTicks = 0;
  long rightTicks = 0;

  int lastL = digitalRead(rotationLeft);
  int lastR = digitalRead(rotationRight);

  unsigned long lastEdgeL = 0;
  unsigned long lastEdgeR = 0;

  if (degrees > 0) 
  {
    // turn right
    analogWrite(leftForward, calibrationForwardLeft);
    digitalWrite(leftBackward, LOW);

    analogWrite(rightBackward, calibrationBackwardRight);
    digitalWrite(rightForward, LOW);
  } else 
  {
    // turn left
    analogWrite(leftBackward, calibrationBackwardLeft);
    digitalWrite(leftForward, LOW);

    analogWrite(rightForward, calibrationForwardRight);
    digitalWrite(rightBackward, LOW);
  }

  while (true) 
  {

    int curL = digitalRead(rotationLeft);
    int curR = digitalRead(rotationRight);
    unsigned long nowUs = micros();

    if (lastL == HIGH && curL == LOW && nowUs - lastEdgeL >= EDGE_MIN_US) 
    {
      leftTicks++;
      lastEdgeL = nowUs;
    }

    if (lastR == HIGH && curR == LOW && nowUs - lastEdgeR >= EDGE_MIN_US) 
    {
      rightTicks++;
      lastEdgeR = nowUs;
    }

    lastL = curL;
    lastR = curR;

    long avg = (leftTicks + rightTicks) / 2;
    if (avg >= targetTicks) break;
  }

  stopMotors();
}

// ultra sound (plus ultra) function
float getUltrasoundDuration() 
{

  digitalWrite(ultrasoundTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasoundTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasoundTrig, LOW);

  return pulseIn(ultrasoundEcho, HIGH);
}

float getUltrasoundDistance() 
{

  ultrasoundDuration = getUltrasoundDuration();
  ultrasoundDistance = (ultrasoundDuration * 0.0343) / 2.0;

  Serial.print("Distance: ");
  Serial.println(ultrasoundDistance);

  return ultrasoundDistance;
}

// obstacle avoidance
void loop() 
{

  float distance = getUltrasoundDistance();

  // object detected
  if (distance > 0 && distance < 15) 
  {

    stopMotors();
    delay(200);

    // avoid sequence

    turnDegrees(-90);
    delay(200);

    move(20, "forward");
    delay(200);

    turnDegrees(90);
    delay(200);

    move(30, "forward");
    delay(200);

    turnDegrees(90);
    delay(200);

    move(20, "forward");
    delay(200);

    turnDegrees(-90);
    delay(200);
  }

  else 
  {
    // normal continuous drive
    driveForward();
  }
}
