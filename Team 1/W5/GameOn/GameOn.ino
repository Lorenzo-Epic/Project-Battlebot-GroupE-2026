//-------------------NEOPIXELS-------------------
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 12 //NI to D12, this is for the NeoPixels
#define NUM_LEDS 4 //number of LEDs used

#define NEO_BACK_RIGHT 0 //pixel 0 -> back right
#define NEO_BACK_LEFT 1 //pixel 1 -> back left
#define NEO_FRONT_LEFT 2 //pixel 2 -> forwards left
#define NEO_FRONT_RIGHT 3 //pixel 3 -> forwards right

Adafruit_NeoPixel pixels(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

#define sensorThreshold 850  //instead of hysteresis, we use a threshold

#define SERVO 11 //servo pin, GR to D11
#define gripperOpenPulse 1600 //pulse width needed to open gripper
#define gripperClosePulse 1100 //pulse width needed to close gripper

const int sensorWeights[8]{ -200, -170, -100, -90, 90, 100, 170, 200};
const int sensorPins[8]{ A0, A1, A2, A3, A4, A5, A6, A7 };
bool sensorRead[8]{ 0, 0, 0, 0, 0, 0, 0, 0 };

int lastDestination = 0;
int sensorVal = 0;

int newLeftSpeed;
int newRightSpeed;

float ancoefficient = 2.6;

#define MOTOR_A_FW 10
#define MOTOR_A_BW 9
#define MOTOR_B_BW 6
#define MOTOR_B_FW 5

#define TRIG_SENS 7
#define ECHO_SENS 4

#define SPEED_A_FW 255
#define SPEED_B_FW 253
#define SPEED_A_BW 253
#define SPEED_B_BW 255

#define SPEED_A_R 255
#define SPEED_B_R 0
#define SPEED_A_L 0
#define SPEED_B_L 255

const float SOUND_SPEED = 0.0343;

#define LEFT_TURN_TIME 740
#define RIGHT_TURN_TIME 800

#define CLEAR_DIST 30
#define OBJECT_DIST 12

unsigned long sensorLastRead = 0;
#define SENSOR_READING_INTERVAL 50

unsigned long releaseLastRead = 0;
#define RELEASE_READING_INTERVAL 500

enum RobotState {
  WAIT_FOR_FLAG,
  DRIVE_TO_FIRST_SQUARE,
  TURN_TO_LINE,
  FOLLOW_LINE,
  FINISHED
};

RobotState currentState = WAIT_FOR_FLAG;

unsigned long blackStartTime = 0;
bool timingBlack = false;
bool wasOnBlackSquare = false;
int blackSquareCount = 0;

void setup() {
  Serial.begin(9600);

  pinMode(MOTOR_A_FW, OUTPUT);
  pinMode(MOTOR_A_BW, OUTPUT);
  pinMode(MOTOR_B_BW, OUTPUT);
  pinMode(MOTOR_B_FW, OUTPUT);

  digitalWrite(MOTOR_A_FW, LOW);
  digitalWrite(MOTOR_A_BW, LOW);
  digitalWrite(MOTOR_B_BW, LOW);
  digitalWrite(MOTOR_B_FW, LOW);

  pinMode(TRIG_SENS, OUTPUT);
  pinMode(ECHO_SENS, INPUT);

  digitalWrite(TRIG_SENS, LOW);

  pinMode(SERVO, OUTPUT);

  digitalWrite(SERVO, LOW);

  pixels.begin();
  pixels.setBrightness(50);
  pixels.show();
}

void loop() {
  static long distance = 999;

  if (millis() - sensorLastRead >= SENSOR_READING_INTERVAL) {
    sensorLastRead = millis();
    distance = getDistanceInCm();
  }

  switch (currentState) {

    case WAIT_FOR_FLAG:
      stop();
      if (distance > CLEAR_DIST) {
        if (getDistanceInCm() > OBJECT_DIST) {
          gripper(gripperOpenPulse);
          currentState = DRIVE_TO_FIRST_SQUARE;
          timingBlack = false;
        }
      }
      break;

    case DRIVE_TO_FIRST_SQUARE:
      driveToFirstSquare();
      break;

    case TURN_TO_LINE:
      stop();
      delay(100);
      turnLeft90Deg();
      moveForwardsFor(120);   // reduced from 180 so it doesn't overshoot into the first turn
      blackSquareCount = 1;
      currentState = FOLLOW_LINE;
      break;

    case FOLLOW_LINE:
      gripper(gripperClosePulse);
      followLineWorkingCode();
      checkFinishSquare();
      break;

    case FINISHED:
      stop();
      for (int i = 0; i < 10; i++) {
        gripper(gripperOpenPulse);
        delay(20);
      }
      moveBackwardsFor(500);

      while (true) {
        stop();
      }
      break;
  }
}

void driveToFirstSquare() {
  setPixelsForwardsMovement();

  int activeSensors = countActiveSensors();

  analogWrite(MOTOR_A_BW, 0);
  analogWrite(MOTOR_B_BW, 0);
  analogWrite(MOTOR_A_FW, SPEED_A_FW * 0.75);
  analogWrite(MOTOR_B_FW, SPEED_B_FW * 0.8);

  if (activeSensors >= 5 && !timingBlack) {
    gripper(gripperOpenPulse);
    timingBlack = true;
    blackStartTime = millis();
  }

  if (activeSensors >= 5 && timingBlack) {
    if (millis() - blackStartTime > 250) {
      gripper(gripperClosePulse);
      stop();
      currentState = TURN_TO_LINE;
      timingBlack = false;
    }
  }

  if (activeSensors < 5) {
    gripper(gripperOpenPulse);
    timingBlack = false;
  }
}

void checkFinishSquare() {
  bool onBlack = isBlackSquare();

  if (onBlack && !timingBlack) {
    timingBlack = true;
    blackStartTime = millis();
  }

  if (onBlack && timingBlack) {
    if (millis() - blackStartTime > 350) {
      if (!wasOnBlackSquare) {
        blackSquareCount++;

        if (blackSquareCount >= 2) {
          stop();
          currentState = FINISHED;
        }

        wasOnBlackSquare = true;
      }
    }
  }

  if (!onBlack) {
    timingBlack = false;
    wasOnBlackSquare = false;
  }
}

int countActiveSensors() {
  int activeSensors = 0;

  for (int i = 0; i < 8; i++) {
    if (analogRead(sensorPins[i]) > sensorThreshold) {
      activeSensors++;
    }
  }

  return activeSensors;
}

bool isBlackSquare() {
  return countActiveSensors() >= 6;
}

void followLineWorkingCode() {
  sensorVal = sensorReading();

  if (sensorVal != 0) {
    lastDestination = sensorVal;
  }

  if (!motorCalc(lastDestination, sensorVal)) {
    return;
  }

  moveForwardsNoStop();
}

int sensorReading() {
  sensorVal = 0;

  for (int i = 0; i < 8; i++) {
    if (analogRead(sensorPins[i]) > sensorThreshold) {
      sensorVal += sensorWeights[i];
    }
  }

  return sensorVal;
}

bool motorCalc(int lastDestination, int sensorVal) {
  if (sensorVal == 0) {
    if (lastDestination > 150) {
      analogWrite(MOTOR_A_FW, 200);
      analogWrite(MOTOR_B_FW, 0);

      analogWrite(MOTOR_A_BW, 0);
      analogWrite(MOTOR_B_BW, 200);
      delay(25);
      return false;
    }

    else if (lastDestination < -150) {
      analogWrite(MOTOR_A_FW, 0);
      analogWrite(MOTOR_B_FW, 200);

      analogWrite(MOTOR_A_BW, 200);
      analogWrite(MOTOR_B_BW, 0);
      delay(25);
      return false;
    }
  }

  newLeftSpeed = (SPEED_A_FW - sensorVal) * ancoefficient;
  newRightSpeed = (SPEED_B_FW + sensorVal) * ancoefficient;

  newLeftSpeed = constrain(newLeftSpeed, 0, SPEED_A_FW);
  newRightSpeed = constrain(newRightSpeed, 0, SPEED_B_FW);

  if (sensorVal >= -190 && sensorVal <= 190) { //FORWARDS
    setPixelsForwardsMovement();
  }

  else if (sensorVal < -190) { //LEFT
    setPixelsLeftSteer();
  }

  else if (sensorVal > 190) { //RIGHT
    setPixelsRightSteer();
  }

  return true;
}

void moveForwardsNoStop() {
  analogWrite(MOTOR_A_FW, newLeftSpeed);
  analogWrite(MOTOR_B_FW, newRightSpeed);

  analogWrite(MOTOR_A_BW, LOW);
  analogWrite(MOTOR_B_BW, LOW);
}

void moveForwardsFor(int moveTime) {
  analogWrite(MOTOR_A_BW, 0);
  analogWrite(MOTOR_B_BW, 0);

  analogWrite(MOTOR_A_FW, SPEED_A_FW);
  analogWrite(MOTOR_B_FW, SPEED_B_FW);

  delay(moveTime);
  stop();
}

void moveBackwardsFor(int moveTime) {  
  analogWrite(MOTOR_A_FW, 0);
  analogWrite(MOTOR_B_FW, 0);

  analogWrite(MOTOR_A_BW, SPEED_A_BW);
  analogWrite(MOTOR_B_BW, SPEED_B_BW);

  delay(moveTime);
  stop();
}

void turnRight90Deg() {
  analogWrite(MOTOR_A_BW, 0);
  analogWrite(MOTOR_B_BW, 0);

  analogWrite(MOTOR_A_FW, SPEED_A_R);
  analogWrite(MOTOR_B_FW, SPEED_B_R);

  delay(RIGHT_TURN_TIME);
  stop();
}

void turnLeft90Deg() {
  analogWrite(MOTOR_A_BW, 0);
  analogWrite(MOTOR_B_BW, 0);

  analogWrite(MOTOR_A_FW, SPEED_A_L);
  analogWrite(MOTOR_B_FW, SPEED_B_L);

  delay(LEFT_TURN_TIME);
  stop();
}

long getDistanceInCm() {
  digitalWrite(TRIG_SENS, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_SENS, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_SENS, LOW);

  long duration = pulseIn(ECHO_SENS, HIGH);

  if (duration == 0) {
    return 999;
  }

  long distance = duration * SOUND_SPEED / 2;

  return distance;
}

void gripper(int newPulse) {
  //static -> the value is not changing
  //unsigned -> the value stored is unsigned, therefore can only be positive, doubling the range of available values
  //long -> large value possible
  static unsigned long timer;
  static int oldPulse;
  
  if (millis() >= timer) { //has enough time passed for next pulse to be sent?
    if (newPulse > 0) { //always executing, because both possible newPulses are positive
      oldPulse = newPulse;
    }

    //set SERVO to HIGH -> wait oldPulse milliseconds -> set SERVO to LOW
    digitalWrite(SERVO, HIGH);
    delayMicroseconds(oldPulse);
    digitalWrite(SERVO, LOW);
    timer = millis() + 20; //wait 20 milliseconds before next pulse
  }
}

//-------------------- PIXELS --------------------
void setPixelsForwardsMovement() { //(255,199,191)
  pixels.clear();
  pixels.setPixelColor(NEO_FRONT_RIGHT, pixels.Color(0,255,0));
  pixels.setPixelColor(NEO_FRONT_LEFT, pixels.Color(0,255,0));
  pixels.show();
}

void setPixelsRightSteer() { //(255,127,80)
  pixels.clear();
  pixels.setPixelColor(NEO_BACK_RIGHT, pixels.Color(0,0,255));
  pixels.setPixelColor(NEO_FRONT_RIGHT, pixels.Color(0,0,255));
  pixels.show();
}

void setPixelsLeftSteer() { //(255,127,80)
  pixels.clear();
  pixels.setPixelColor(NEO_BACK_LEFT, pixels.Color(0,0,255));
  pixels.setPixelColor(NEO_FRONT_LEFT, pixels.Color(0,0,255));
  pixels.show();
}

void stop() {
  analogWrite(MOTOR_A_FW, 0);
  analogWrite(MOTOR_A_BW, 0);
  analogWrite(MOTOR_B_BW, 0);
  analogWrite(MOTOR_B_FW, 0);
}