//distance between walls is 30cm
//robot is about 15cm wide (18 to be more precise)
// robot should stop every 60cm to check where it can go, if can go left it will always go left

#include <Adafruit_NeoPixel.h>

/* ================= STATE ================= */
boolean isStartSequenceActive = false;
boolean isMazeNavigationActive = false;

/* ================= LINE SENSORS ================= */
const int LINE_SENSOR_PINS[] = {A0, A1, A2, A3, A4, A5, A6, A7};
#define BLACK_LINE_THRESHOLD 500

/* ================= SERVO ================= */
#define SERVO_PIN 11
#define GRIPPER_OPEN_US 1820
#define GRIPPER_CLOSE_US 1000
#define SERVO_CYCLE_REPEAT 10

/* ================= DISTANCE SENSOR CLASS ================= */
class DistanceSensor
{
  private:
    int trigPin;
    int echoPin;
    const int MAX_DISTANCE = 100;

    float getPulseDuration()
    {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      return pulseIn(echoPin, HIGH, 60 * MAX_DISTANCE);
    }

    float getProcessedDistance()
    {
      float pulse = getPulseDuration();
      if (pulse > 100)
        return (pulse * 0.0343) / 2.0;
      return MAX_DISTANCE;
    }

  public:
    DistanceSensor(int tPin, int ePin)
    {
      trigPin = tPin;
      echoPin = ePin;
      pinMode(trigPin, OUTPUT);
      digitalWrite(trigPin, LOW);
      pinMode(echoPin, INPUT);
    }

    double getCurrentDistance(int sampleNum = 7)
    {
      if (sampleNum < 3) sampleNum = 3;
      if (sampleNum > 15) sampleNum = 15;

      float samples[15];

      for (int i = 0; i < sampleNum; i++)
        samples[i] = getProcessedDistance();

      // insertion sort
      for (int i = 1; i < sampleNum; i++)
      {
        float key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key)
        {
          samples[j + 1] = samples[j];
          j--;
        }
        samples[j + 1] = key;
      }

      // remove smallest and largest
      float sum = 0;
      for (int i = 1; i < sampleNum - 1; i++)
        sum += samples[i];

      return sum / (sampleNum - 2);
    }
};

/* ================= DISTANCE SENSORS ================= */
DistanceSensor distanceSensorFront(4, 7);
DistanceSensor distanceSensorLeft(8, 13);
int distanceFront;
int distanceLeft;

/* ================= MOTORS ================= */
#define LEFT_FORWARD_PIN 5
#define LEFT_BACKWARD_PIN 10
#define RIGHT_FORWARD_PIN 6
#define RIGHT_BACKWARD_PIN 9
#define MOTOR_STALL_TIMEOUT 1700
#define MAX_MOTOR_SPEED 255

/* ================= ENCODERS ================= */
#define ROTATION_LEFT_PIN 2
#define RIGHT_ENCODER_PIN 3
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

/* ================= LED ================= */
#define NEOPIXEL_PIN 12
Adafruit_NeoPixel statusLEDs(4, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);

const uint32_t LED_COLOR_RED = statusLEDs.Color(255, 0, 0);
const uint32_t LED_COLOR_YELLOW = statusLEDs.Color(255, 150, 0);
const uint32_t LED_COLOR_BLUE = statusLEDs.Color(0, 0, 255);
const uint32_t LED_COLOR_WHITE = statusLEDs.Color(255, 255, 255);

/* ================= HELPER FUNCTIONS ================= */

void updateFrontDistance() {
  distanceFront = distanceSensorFront.getCurrentDistance();
}

void updateLeftDistance() {
  distanceLeft = distanceSensorLeft.getCurrentDistance();
}

void setupMotorPins() {
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);
}

void stopMotors() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}

void driveForward() {
  analogWrite(LEFT_FORWARD_PIN, MAX_MOTOR_SPEED * 0.9);
  analogWrite(RIGHT_FORWARD_PIN, MAX_MOTOR_SPEED);
}

void driveBackwards() {
  analogWrite(LEFT_BACKWARD_PIN, MAX_MOTOR_SPEED);
  analogWrite(RIGHT_BACKWARD_PIN, MAX_MOTOR_SPEED);
}

void turnLeft() {
  analogWrite(RIGHT_FORWARD_PIN, MAX_MOTOR_SPEED * 0.8);
}

void rotate() {
  analogWrite(LEFT_FORWARD_PIN, MAX_MOTOR_SPEED * 0.8);
  analogWrite(RIGHT_BACKWARD_PIN, MAX_MOTOR_SPEED * 0.8);
}

/* ================= SERVO ================= */

void setServo(int pulse) {
  for (int i = 0; i < SERVO_CYCLE_REPEAT; i++) {
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(SERVO_PIN, LOW);
    delay(20);
  }
}

void openGripper() {
  setServo(GRIPPER_OPEN_US);
}

void closeGripper() {
  setServo(GRIPPER_CLOSE_US);
}

/* ================= ENCODER ISR ================= */

void CountEncoder1() { leftEncoderCount++; }
void CountEncoder2() { rightEncoderCount++; }

/* ================= MOVEMENT WITH ENCODERS ================= */

void driveForwardOnPulses(int target) {
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  driveForward();

  while (leftEncoderCount < target && rightEncoderCount < target) {
    updateFrontDistance();
    if (distanceFront < 15) break;
  }

  stopMotors();
}

void turnLeftOnPulses(int target) {
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  turnLeft();

  while (rightEncoderCount < target) {
    updateLeftDistance();
    if (distanceLeft < 10) break;
  }

  stopMotors();
}

/* ================= BLACK ZONE ================= */

boolean reachedBlackZone() {
  int count = 0;
  for (int i = 0; i < 8; i++)
    if (analogRead(LINE_SENSOR_PINS[i]) > BLACK_LINE_THRESHOLD)
      count++;
  return count >= 6;
}

/* ================= SETUP ================= */

void setup() {
  setupMotorPins();
  pinMode(SERVO_PIN, OUTPUT);

  pinMode(ROTATION_LEFT_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ROTATION_LEFT_PIN), CountEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), CountEncoder2, CHANGE);

  statusLEDs.begin();
  statusLEDs.fill(LED_COLOR_BLUE, 0, 4);
  statusLEDs.show();
}

/* ================= LOOP ================= */

void loop() {

  if (isMazeNavigationActive) {

    updateFrontDistance();
    updateLeftDistance();

    if (reachedBlackZone()) {
      stopMotors();
      openGripper();
      delay(300);
      isMazeNavigationActive = false;
      isStartSequenceActive = false;
      return;
    }

    // improved thresholds for 30cm maze
    if (distanceLeft > 20 && distanceFront > 18) {
      turnLeftOnPulses(36);
      delay(200);
      driveForwardOnPulses(20);
      turnLeftOnPulses(36);
    }
    else if (distanceFront > 18) {
      driveForwardOnPulses(20);
    }
    else {
      driveBackwards();
      delay(100);
      stopMotors();

      unsigned long rotateStart = millis();
      while (distanceFront < 18 && millis() - rotateStart < 3000) {
        rotate();
        updateFrontDistance();
      }
      stopMotors();
    }
  }

  else {

    updateFrontDistance();

    if (!isStartSequenceActive && distanceFront < 30) {
      openGripper();
      delay(1500);
      isStartSequenceActive = true;
    }

    if (isStartSequenceActive) {
      driveForward();
      delay(800);
      closeGripper();
      turnLeftOnPulses(36);
      driveForwardOnPulses(70);
      isMazeNavigationActive = true;
    }
  }
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
