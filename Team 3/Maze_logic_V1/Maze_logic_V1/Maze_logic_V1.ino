#include <Arduino.h>

// pins for motor
const int LEFT_MOTOR_PWM = 11;
const int LEFT_MOTOR_DIR = A1;
const int RIGHT_MOTOR_PWM = 9;
const int RIGHT_MOTOR_DIR = A0;

// setup for line sensor
const int NUM_SENSORS = 8;
const int LINE_PINS[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7}; // adjust if needed
int sensorWeights[NUM_SENSORS] = {-4000,-2500,-500,-500,500,500,2500,4000};
int weights[NUM_SENSORS] = {-237, -233, -224, -257, -250, -266, -287, -303}; // calibration
int sensorValues[NUM_SENSORS];

// first plus ultra sensor 
#define TRIG_PIN 7
#define ECHO_PIN 10
int distanceFront = 0;

// pid
float Kp = 0.05; // proportional
float Ki = 0.0;  // integral
float Kd = 0.02; // derivative

float lastError = 0;
float integral = 0;

// klancker contorl
int baseSpeed = 120; // motor speed (0-255)
int turnSpeed = 80;

// setup
void setup() {
  Serial.begin(9600);

  // motor pins
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);

  // mine sensors
  for(int i=0;i<NUM_SENSORS;i++){
    pinMode(LINE_PINS[i], INPUT);
  }

  // ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

// the main oop
void loop() {
  readLineSensors();
  readUltrasonic();

  if(distanceFront < 15){
    // wall detected: stop or rotate
    stopMotors();
    delay(200);
    rotate180(); // simple rotation if wall detected tune later yada
  } else {
    followLinePID();
  }
}

// line sensors reader
void readLineSensors(){
  for(int i=0;i<NUM_SENSORS;i++){
    int raw = analogRead(LINE_PINS[i]);
    sensorValues[i] = raw - weights[i]; // apply calibration later yada
  }
}

// calculates the position of the line cant test idk if it works i hope it does
float getLinePosition(){
  long numerator = 0;
  long denominator = 0;
  for(int i=0;i<NUM_SENSORS;i++){
    numerator += (long)sensorValues[i]*sensorWeights[i];
    denominator += sensorValues[i];
  }
  if(denominator == 0) return 0; // line lost
  return (float)numerator/denominator;
}

// pid of line folllowing since physical maze has that start line thingy
void followLinePID(){
  float position = getLinePosition();
  float error = position;
  integral += error;
  float derivative = error - lastError;
  float correction = Kp*error + Ki*integral + Kd*derivative;

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // constrain motor speeds
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(LEFT_MOTOR_PWM, leftSpeed);
  analogWrite(RIGHT_MOTOR_PWM, rightSpeed);
  digitalWrite(LEFT_MOTOR_DIR, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR, HIGH);

  lastError = error;
}

// ultrasonic sensor reader 
void readUltrasonic(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  distanceFront = duration * 0.034 / 2; // convert to cm
}

// functions to control the mootr
void stopMotors(){
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}

// 180 rotation (can replace with better shit later not that tunned im tired)
void rotate180(){
  analogWrite(LEFT_MOTOR_PWM, turnSpeed);
  analogWrite(RIGHT_MOTOR_PWM, turnSpeed);
  digitalWrite(LEFT_MOTOR_DIR, LOW); // reverse left
  digitalWrite(RIGHT_MOTOR_DIR, HIGH); // forward right
  delay(800); // adjust for robot rotation
  stopMotors();
}

// what to improve now 
// 1. 2 extra ultrasonic sensors for right/left wall detection:
//    - just copy readUltrasonic() for two more sensors
//    - adjust maze logic to use all 3 distances
// 2. the bluethoot device is ready to pair:
//    - Use SoftwareSerial or SPI depending on module (saw on other peoples code)
//    - Send line position, speed, and distance for monitoring (again saw in other peoples code idk how to touch this)
// 3. PID tuning for smoother line following blablabla
// 4. add smarter maze decision logic (crossroads or dead ends idk man stupid klancker)
// 5. add timeouts or failsafe if line is lost for too long but we can test so idk aobut this yet
// 6. maybe we will have to integrate gripper later idk i hope we dont i dont wanna do that
