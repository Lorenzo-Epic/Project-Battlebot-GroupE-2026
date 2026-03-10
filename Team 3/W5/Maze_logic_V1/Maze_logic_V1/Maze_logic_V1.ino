// pins 
const int LEFT_FORWARD_PIN  = 7;
const int LEFT_BACKWARD_PIN = 10;
const int RIGHT_FORWARD_PIN = 11;
const int RIGHT_BACKWARD_PIN = 9;

int motorSpeed = 150;


// ultrasonic headhog
//sensors r prob wrong cause i dont have the pins at home
// front
const int TRIG_FRONT = 2;
const int ECHO_FRONT = 3;
// leftie
const int TRIG_LEFT = 4;
const int ECHO_LEFT = 5;
// right
const int TRIG_RIGHT = 6;
const int ECHO_RIGHT = 8;

// distance to wall
const int WALL_DISTANCE = 20; // cm

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
  delayMicroseconds(5);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);

  float distance = duration * 0.034 / 2;

  return distance;
}


// movements

void moveForward() 
{
  analogWrite(LEFT_FORWARD_PIN, motorSpeed);
  analogWrite(LEFT_BACKWARD_PIN, 0);

  analogWrite(RIGHT_FORWARD_PIN, motorSpeed);
  analogWrite(RIGHT_BACKWARD_PIN, 0);

  delay(350);

  stopMotors();
}

void turnLeft90() 
{
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, motorSpeed);

  analogWrite(RIGHT_FORWARD_PIN, motorSpeed);
  analogWrite(RIGHT_BACKWARD_PIN, 0);

  delay(330);

  stopMotors();
}

void turnRight90()
{
  analogWrite(LEFT_FORWARD_PIN, motorSpeed);
  analogWrite(LEFT_BACKWARD_PIN, 0);

  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, motorSpeed);

  delay(330);

  stopMotors();
}

void stopMotors()
{
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);

  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}
