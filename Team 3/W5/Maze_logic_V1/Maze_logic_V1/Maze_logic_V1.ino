#include <Arduino.h>
#include <math.h>
#include <string.h>
// testing github again
// pin mapping
#define LEFT_FORWARD_PIN 5
#define LEFT_BACKWARD_PIN 10
#define RIGHT_FORWARD_PIN 6
#define RIGHT_BACKWARD_PIN 9

#define LEFT_ENCODER_PIN 2
#define RIGHT_ENCODER_PIN 3

#define FRONT_TRIG 4
#define FRONT_ECHO 7
#define LEFT_TRIG 12
#define LEFT_ECHO 13
#define RIGHT_TRIG A0
#define RIGHT_ECHO A1

#define SERVO_PIN 11
#define GRIPPER_OPEN_US 1820
#define GRIPPER_CLOSE_US 1000
#define SERVO_CYCLE_REPEAT 10

// motor and encoder calibration
const int CALIBRATION_FORWARD_LEFT = 255;
const int CALIBRATION_BACKWARD_LEFT = 255;
const int CALIBRATION_FORWARD_RIGHT = 243;
const int CALIBRATION_BACKWARD_RIGHT = 220;

// Encoder and wheel geometry
const float WHEEL_DIAMETER_CM = 6.5f;
const int SLOTS_PER_REV = 20;
const int EDGES_PER_SLOT = 2;
const int TICKS_PER_REV = SLOTS_PER_REV * EDGES_PER_SLOT;
const float CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const float TICKS_PER_CM = (float)TICKS_PER_REV / CIRCUMFERENCE_CM;

// Encoder filtering
const unsigned long EDGE_MIN_US = 150;

// Turn calibration
const float TURN_SLOTS_FOR_90_DEG = 8.0f;
const float TURN_TICKS_PER_DEG = (TURN_SLOTS_FOR_90_DEG * EDGES_PER_SLOT) / 90.0f;
const float SINGLE_WHEEL_TURN_SCALE = 2.0f;
const long TURN_SLOWDOWN_TICKS = 3;
const int TURN_SLOW_LEFT_FORWARD = 150;
const int TURN_SLOW_LEFT_BACKWARD = 150;
const int TURN_SLOW_RIGHT_FORWARD = 145;
const int TURN_SLOW_RIGHT_BACKWARD = 135;

// shared variables
volatile unsigned long g_leftTicks = 0;
volatile unsigned long g_rightTicks = 0;
volatile unsigned long g_lastLeftUs = 0;
volatile unsigned long g_lastRightUs = 0;

boolean isStartSequenceActive = false;
boolean isMazeNavigationActive = false;

// line sensors
const int LINE_SENSOR_PINS[] = {A3, A4, A5, A6};
#define BLACK_LINE_THRESHOLD 800

// distance sensor
class DistanceSensor {
  private:
    int trigPin, echoPin;
    const int MAX_DISTANCE = 100;

    float getPulseDuration() {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      return pulseIn(echoPin, HIGH, 60 * MAX_DISTANCE);
    }

    float getProcessedDistance() {
      float pulse = getPulseDuration();
      return (pulse > 100) ? (pulse * 0.0343) / 2 : MAX_DISTANCE;
    }

  public:
    DistanceSensor(int tPin, int ePin) {
      trigPin = tPin; echoPin = ePin;
      pinMode(trigPin, OUTPUT); digitalWrite(trigPin, HIGH);
      pinMode(echoPin, INPUT);
    }

    double getDistance(int samples = 5) {
      double vals[samples];
      for (int i=0; i<samples; i++) vals[i] = getProcessedDistance();
      // simple average
      double sum=0;
      for (int i=0;i<samples;i++) sum+=vals[i];
      return sum/samples;
    }
};

DistanceSensor distanceFront(FRONT_TRIG, FRONT_ECHO);
DistanceSensor distanceLeft(LEFT_TRIG, LEFT_ECHO);
DistanceSensor distanceRight(RIGHT_TRIG, RIGHT_ECHO);

float distFront, distLeft, distRight;

// encoder isr
void isrLeft() {
  unsigned long now = micros();
  if (now - g_lastLeftUs >= EDGE_MIN_US) { g_leftTicks++; g_lastLeftUs = now; }
}
void isrRight() {
  unsigned long now = micros();
  if (now - g_lastRightUs >= EDGE_MIN_US) { g_rightTicks++; g_lastRightUs = now; }
}

// motor functions
void stopMotors() {
  analogWrite(LEFT_FORWARD_PIN, 0); analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0); analogWrite(RIGHT_BACKWARD_PIN, 0);
  digitalWrite(LEFT_FORWARD_PIN, LOW); digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, LOW); digitalWrite(RIGHT_BACKWARD_PIN, LOW);
}

void driveForward(int leftPWM=CALIBRATION_FORWARD_LEFT, int rightPWM=CALIBRATION_FORWARD_RIGHT) {
  analogWrite(LEFT_BACKWARD_PIN, 0); analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(LEFT_FORWARD_PIN, leftPWM); analogWrite(RIGHT_FORWARD_PIN, rightPWM);
}

void driveBackward(int leftPWM=CALIBRATION_BACKWARD_LEFT, int rightPWM=CALIBRATION_BACKWARD_RIGHT) {
  analogWrite(LEFT_FORWARD_PIN, 0); analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, leftPWM); analogWrite(RIGHT_BACKWARD_PIN, rightPWM);
}

void turnLeftInPlace() {
  analogWrite(LEFT_FORWARD_PIN, 0); analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0); analogWrite(RIGHT_FORWARD_PIN, CALIBRATION_FORWARD_RIGHT);
}

void turnRightInPlace() {
  analogWrite(RIGHT_FORWARD_PIN, 0); analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0); analogWrite(LEFT_FORWARD_PIN, CALIBRATION_FORWARD_LEFT);
}

// ================== HELPER FUNCTIONS ==================
long cmToTicks(float cm) { return round(cm * TICKS_PER_CM); }
long degToTicks(float deg) { return round(deg * TURN_TICKS_PER_DEG * SINGLE_WHEEL_TURN_SCALE); }

void resetTicks() { noInterrupts(); g_leftTicks=0; g_rightTicks=0; g_lastLeftUs=0; g_lastRightUs=0; interrupts(); }
void readTicks(unsigned long &l, unsigned long &r) { noInterrupts(); l=g_leftTicks; r=g_rightTicks; interrupts(); }

void move(float amount, const char *direction) {
  const bool IS_DRIVE = (strcmp(direction,"forward")==0) || (strcmp(direction,"backward")==0);
  const bool IS_TURN  = (strcmp(direction,"left")==0) || (strcmp(direction,"right")==0);
  if (!IS_DRIVE && !IS_TURN) return;

  long targetTicks = IS_DRIVE ? cmToTicks(amount) : degToTicks(amount);
  resetTicks();

  if (strcmp(direction,"forward")==0) driveForward(200,200);
  else if (strcmp(direction,"backward")==0) driveBackward(180,180);
  else if (strcmp(direction,"left")==0) turnLeftInPlace();
  else if (strcmp(direction,"right")==0) turnRightInPlace();

  unsigned long startMS = millis();
  while (true) {
    unsigned long l,r; readTicks(l,r);
    long progress = IS_DRIVE ? (l+r)/2 : ((strcmp(direction,"left")==0)? r : l);
    if ((long)progress>=targetTicks) break;
    if (millis()-startMS>5000) break;
  }

  stopMotors();
  unsigned long l, r;
  readTicks(l,r); // optional debug
  Serial.print("Move "); Serial.print(direction); Serial.print(" ticks="); Serial.println(targetTicks);
}

// gripper
void setServo(int pulse) {
  for(int i=0;i<SERVO_CYCLE_REPEAT;i++){
    digitalWrite(SERVO_PIN,HIGH); delayMicroseconds(pulse);
    digitalWrite(SERVO_PIN,LOW); delay(20);
  }
}
void openGripper(){ setServo(GRIPPER_OPEN_US); }
void closeGripper(){ setServo(GRIPPER_CLOSE_US); }

// line sensor
void lineSensorsInit() { for(int i=0;i<8;i++) pinMode(LINE_SENSOR_PINS[i],INPUT); }
boolean reachedBlackZone() {
  int count=0;
  for(int i=0;i<8;i++){ if(analogRead(LINE_SENSOR_PINS[i])>BLACK_LINE_THRESHOLD) count++; }
  return (count>=6);
}

// setup
void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(LEFT_FORWARD_PIN, OUTPUT); pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT); pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  // Encoder pins
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP); pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), isrRight, CHANGE);

  // Line sensors
  lineSensorsInit();

  // Servo
  pinMode(SERVO_PIN, OUTPUT);
  openGripper();
}

// main looper
void loop() {
  if (!isStartSequenceActive) {
    distFront = distanceFront.getDistance();
    if (distFront<30) {
      openGripper(); delay(1500);
      isStartSequenceActive=true;
    }
  } else if (!isMazeNavigationActive) {
    // start sequence: close gripper, move forward, turn left, forward, start maze
    closeGripper();
    move(15,"forward"); // approach maze
    move(90,"left");    // orient left wall
    move(60,"forward");
    isMazeNavigationActive=true;
  } else {
    // Maze navigation
    distFront = distanceFront.getDistance();
    distLeft = distanceLeft.getDistance();
    distRight = distanceRight.getDistance();

    if (reachedBlackZone()) {
      stopMotors();
      openGripper();
      move(2,"backward");
      delay(60000);
    } else if (distFront>12) {
      if(distLeft>17) {
        move(60,"forward"); move(90,"left"); move(60,"forward");
      } else {
        move(60,"forward");
      }
    } else {
      move(2,"backward"); move(90,"right");
    }
  }
}
