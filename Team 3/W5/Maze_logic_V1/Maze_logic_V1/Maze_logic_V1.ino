// pins 
const int LEFT_FORWARD_PIN  = 3;
const int LEFT_BACKWARD_PIN = 10;
const int RIGHT_FORWARD_PIN = 11;
const int RIGHT_BACKWARD_PIN = 9;

int motorSpeed = 150;

// ultrasonic 
// front
const int TRIG_FRONT = 12;
const int ECHO_FRONT = 13;
// leftie
const int TRIG_LEFT = 7;
const int ECHO_LEFT = 4;
// right
const int TRIG_RIGHT = A3;
const int ECHO_RIGHT = 8;

// distance to wall
const int WALL_DISTANCE = 20; // cm
const int TOO_CLOSE_TO_WALLS = 8; // cm


// setup
void setup() {

  Serial.begin(9600);

  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  stopMotors();
}

// main loop

void loop() {

  float frontDistance = getDistance(TRIG_FRONT, ECHO_FRONT);
  float leftDistance  = getDistance(TRIG_LEFT, ECHO_LEFT);
  float rightDistance = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.print("  Left: ");
  Serial.print(leftDistance);
  Serial.print("  Right: ");
  Serial.println(rightDistance);

  // small adjustment if too close to wall
  if(leftDistance < TOO_CLOSE_TO_WALLS) 
  {
    // turn slightly right
    analogWrite(LEFT_FORWARD_PIN, motorSpeed);
    analogWrite(RIGHT_FORWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, motorSpeed);
    delay(80);
    stopMotors();
  }
  
  if(rightDistance < TOO_CLOSE_TO_WALLS)
  {
  // turn slightly left
    analogWrite(LEFT_BACKWARD_PIN, motorSpeed);
    analogWrite(RIGHT_FORWARD_PIN, motorSpeed);
    delay(80);
    stopMotors();
  }

  // follows the left wall all the time

  if(leftDistance > WALL_DISTANCE) 
  {
    turnLeft90();
    moveForward();
  }
  else if(frontDistance > WALL_DISTANCE) 
  {
    moveForward();
  }
  else if(rightDistance > WALL_DISTANCE) 
  {
    turnRight90();
    moveForward();
  }
  else 
  {
    // dead end
    turnRight90();
    turnRight90();
  }

}

// functions for ultrasonic
float getDistance(int trigPin, int echoPin) 
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(50);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(100);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
// this is speed of sound by microseconds so: distance (cm) = duration (µs/microseconds)* 0.017
// 0.017 is speed of sound already divided by 2, so instead of 0.034 its 0.017
  float distance = duration * 0.017; 

  return distance;
}

// movements
void moveForward() {
  analogWrite(LEFT_FORWARD_PIN, motorSpeed);
  analogWrite(LEFT_BACKWARD_PIN, 0);

  analogWrite(RIGHT_FORWARD_PIN, motorSpeed);
  analogWrite(RIGHT_BACKWARD_PIN, 0);

  delay(400);

  stopMotors();
}

void turnLeft90() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, motorSpeed);

  analogWrite(RIGHT_FORWARD_PIN, motorSpeed);
  analogWrite(RIGHT_BACKWARD_PIN, 0);

  delay(330);

  stopMotors();
}

void turnRight90(){
  analogWrite(LEFT_FORWARD_PIN, motorSpeed);
  analogWrite(LEFT_BACKWARD_PIN, 0);

  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, motorSpeed);

  delay(330);

  stopMotors();
}

void stopMotors(){
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);

  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}
