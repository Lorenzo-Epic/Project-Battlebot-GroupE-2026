const int leftBackward = 3;
const int leftForward  = 5;

const int rightForward  = 6;
const int rightBackward = 9;

// rotation sensors (pins)
const int rotationLeft  = 10;
const int rotationRight = 11;

const int calibrationForwardLeft = 255;
const int calibrationBackwardLeft;

const int calibrationForwardRight = 242;
const int calibrationBackwardRight;

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

void loop() {
//  analogWrite(leftForward, calibrationForwardLeft);
//  digitalWrite(leftBackward, LOW);
  
//  analogWrite(rightForward, calibrationForwardRight);
//  digitalWrite(rightBackward, LOW);
}
