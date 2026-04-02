#include <Adafruit_NeoPixel.h>

#define LED_PIN 7
#define NUM_LEDS 4

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ================= MOTOR PINS =================
const int LEFT_FWD  = 11;
const int LEFT_BWD  = 9;
const int RIGHT_FWD = 10;
const int RIGHT_BWD = 3;

// ================= ULTRASONIC SENSOR =================
const int TRIG_PIN = 13;
const int ECHO_PIN = 12;

// ================= GRIPPER =================
#define SERVO 8
#define GRIPPER_OPEN  1650
#define GRIPPER_CLOSE 1100

// ================= LINE SENSORS =================
int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// ================= SPEED SETTINGS =================
const int BASE_SPEED = 230;
const int TURN_SPEED = 230;
const int SEARCH_SPEED = 210;
const int APPROACH_SPEED = 210;

const int LINE_TH_RIGHT  = 700;
const int LINE_TH_CENTER = 800;
const int LINE_TH_LEFT   = 800;

// ================= FINISH SETTINGS =================
const int FINISH_SLOW_SPEED = 200;
const unsigned long FINISH_CHECK_TIME = 300;

// ================= DISTANCE/TIME SETTINGS =================
const int OBJECT_DETECT_DISTANCE = 30;
const unsigned long APPROACH_TIME = 2000;
const unsigned long BACK_TIME = 4000;
const unsigned long LEFT_TURN_90_TIME = 1100;
const unsigned long FORWARD_AFTER_GRAB_TIME = 400;

// ================= OBSTACLE SETTINGS =================
const int OBSTACLE_DISTANCE = 10;

// ================= ULTRASONIC CACHE =================
unsigned long lastUltrasonicTime = 0;
long cachedDistance = 999;

// ================= STATE =================
int lastTurn = 0;   // -1 left, 0 straight, 1 right
bool turningRight = false;
bool turningLeft  = false;
bool rightTurnLostCenter = false;
bool leftTurnLostCenter  = false;
bool finished = false;
bool startBoostNeeded = true;

unsigned long moveStartTime = 0;
unsigned long actionStartTime = 0;

bool finishCandidate = false;
unsigned long finishCandidateStart = 0;

// ================= MODES =================
enum RobotMode {
  WAIT_OBJECT,
  GO_TO_OBJECT,
  GRAB_OBJECT,
  TURN_LEFT_90_AFTER_GRAB,
  GO_FORWARD_AFTER_GRAB,
  MAZE_MODE,
  FINISHED_MODE
};

RobotMode mode = WAIT_OBJECT;

// ================= LED CONTROL =================
// Распиновка:
// 3 = передний левый
// 2 = передний правый
// 0 = задний правый
// 1 = задний левый

void ledsBaseGreen() {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 40, 0));
  }
}

void ledsForward() {
  ledsBaseGreen();
  strip.setPixelColor(3, strip.Color(200, 0, 0)); // передний левый
  strip.setPixelColor(2, strip.Color(200, 0, 0)); // передний правый
  strip.show();
}

void ledsBack() {
  ledsBaseGreen();
  strip.setPixelColor(0, strip.Color(200, 0, 0)); // задний правый
  strip.setPixelColor(1, strip.Color(200, 0, 0)); // задний левый
  strip.show();
}

void ledsLeft() {
  ledsBaseGreen();
  strip.setPixelColor(3, strip.Color(200, 0, 0)); // передний левый
  strip.setPixelColor(0, strip.Color(200, 0, 0)); // задний левый
  strip.show();
}

void ledsRight() {
  ledsBaseGreen();
  strip.setPixelColor(2, strip.Color(200, 0, 0)); // передний правый
  strip.setPixelColor(1, strip.Color(200, 0, 0)); // задний правый
  strip.show();
}

void ledsIdle() {
  ledsBaseGreen();
  strip.show();
}

// ================= MOTOR CONTROL =================
void stopMotors() {
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, 0);

  ledsIdle();
  startBoostNeeded = true;
}

void forward(int spd) {
  ledsForward();

  if (startBoostNeeded) {
    analogWrite(LEFT_FWD, 255);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_FWD, 255);
    analogWrite(RIGHT_BWD, 0);
    delay(60);
    startBoostNeeded = false;
  }

  analogWrite(LEFT_FWD, max(0, spd - 10));
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, constrain(spd, 0, 255));
  analogWrite(RIGHT_BWD, 0);
}

void backward(int spd) {
  ledsBack();

  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, constrain(spd, 0, 255));
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, constrain(spd + 30, 0, 255));
}

void turnLeft() {
  ledsLeft();

  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, constrain(TURN_SPEED + 40, 0, 255));
  analogWrite(RIGHT_BWD, 0);
}

void turnRight() {
  ledsRight();

  analogWrite(LEFT_FWD, TURN_SPEED);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, TURN_SPEED);
}

void pivotLeft() {
  ledsLeft();

  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, constrain(TURN_SPEED, 0, 255));
  analogWrite(RIGHT_FWD, constrain(TURN_SPEED, 0, 255));
  analogWrite(RIGHT_BWD, 0);
}

void slightLeft() {
  ledsLeft();

  analogWrite(LEFT_FWD, constrain(BASE_SPEED - 50, 0, 255));
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, constrain(BASE_SPEED, 0, 255));
  analogWrite(RIGHT_BWD, 0);
}

void slightRight() {
  ledsRight();

  analogWrite(LEFT_FWD, constrain(BASE_SPEED, 0, 255));
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, constrain(BASE_SPEED - 50, 0, 255));
  analogWrite(RIGHT_BWD, 0);
}

void forwardToObject(int spd) {
  ledsForward();

  if (startBoostNeeded) {
    analogWrite(LEFT_FWD, 255);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_FWD, 255);
    analogWrite(RIGHT_BWD, 0);
    delay(60);
    startBoostNeeded = false;
  }

  analogWrite(LEFT_FWD, max(0, spd - 20));
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, constrain(spd + 40, 0, 255));
  analogWrite(RIGHT_BWD, 0);
}

// ================= GRIPPER CONTROL =================
void gripper(int newPulse) {
  static unsigned long timer = 0;
  static int pulse = GRIPPER_OPEN;

  if (millis() > timer) {
    if (newPulse > 0) {
      pulse = newPulse;
    }

    digitalWrite(SERVO, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(SERVO, LOW);

    timer = millis() + 20;
  }
}

void holdGripper(int pulse, int timeMs) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)timeMs) {
    gripper(pulse);
  }
}

void gripperOpen() {
  holdGripper(GRIPPER_OPEN, 1500);
}

void gripperClose() {
  holdGripper(GRIPPER_CLOSE, 1500);
}

void serviceGripper() {
  gripper(0);
}

void delayWithGripper(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    serviceGripper();
  }
}

// ================= ULTRASONIC =================
long getDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;

  return duration * 0.034 / 2.0;
}

bool isObstacleAhead() {
  if (millis() - lastUltrasonicTime > 500) {
    lastUltrasonicTime = millis();
    cachedDistance = getDistanceCm();
  }

  return (cachedDistance > 0 && cachedDistance < OBSTACLE_DISTANCE);
}

// ================= SENSOR READ =================
void readSensors(bool &farRight, bool &right, bool &center, bool &left, bool &farLeft) {
  farRight = false;
  right = false;
  center = false;
  left = false;
  farLeft = false;

  for (int i = 0; i < 8; i++) {
    int v = analogRead(sensorPins[i]);
    int threshold = LINE_TH_CENTER;

    if (i == 0 || i == 1 || i == 2) threshold = LINE_TH_RIGHT;
    if (i == 5 || i == 6 || i == 7) threshold = LINE_TH_LEFT;

    if (v > threshold) {
      if (i == 0 || i == 1) farRight = true;
      if (i == 2) right = true;
      if (i == 3 || i == 4) center = true;
      if (i == 5) left = true;
      if (i == 6 || i == 7) farLeft = true;
    }
  }
}

bool isFinishSquare() {
  int blackCount = 0;
  bool centerSeen = false;

  for (int i = 0; i < 8; i++) {
    int v = analogRead(sensorPins[i]);

    int threshold = LINE_TH_CENTER;
    if (i == 0 || i == 1 || i == 2) threshold = LINE_TH_RIGHT;
    if (i == 5 || i == 6 || i == 7) threshold = LINE_TH_LEFT;

    if (v > threshold) {
      blackCount++;
      if (i == 3 || i == 4) centerSeen = true;
    }
  }

  return (blackCount >= 6 && centerSeen);
}

// ================= SPECIAL ACTIONS =================
void turnAround() {
  ledsRight();

  analogWrite(LEFT_FWD, 255);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, 255);
  delay(80);

  analogWrite(LEFT_FWD, 255);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, 255);
  delayWithGripper(150);

  stopMotors();
  delayWithGripper(50);
}

void finishStop() {
  Serial.println("FINISH reached");

  stopMotors();
  delayWithGripper(300);

  gripperOpen();
  delayWithGripper(400);

  stopMotors();
  delayWithGripper(400);

  unsigned long start = millis();
  while (millis() - start < BACK_TIME) {
    backward(BASE_SPEED);
    serviceGripper();
  }

  stopMotors();

  finished = true;
  mode = FINISHED_MODE;
}

// ================= LINE FOLLOW WITH RIGHT HAND =================
void lineFollow() {
  serviceGripper();

  if (mode == MAZE_MODE && isObstacleAhead()) {
    stopMotors();
    delayWithGripper(100);
    turningRight = false;
    turningLeft = false;
    finishCandidate = false;
    turnAround();
    lastTurn = 1;
    return;
  }

  if (mode == MAZE_MODE) {
    bool finishNow = isFinishSquare();

    if (finishNow && !finishCandidate) {
      finishCandidate = true;
      finishCandidateStart = millis();
    }

    if (finishCandidate) {
      forward(FINISH_SLOW_SPEED);
      if (finishNow) {
        if (millis() - finishCandidateStart >= FINISH_CHECK_TIME) {
          finishStop();
          return;
        }
      } else {
        finishCandidate = false;
      }
      return;
    }
  }

  bool farRight, right, center, left, farLeft;
  readSensors(farRight, right, center, left, farLeft);

  
  if (farRight || right) {
    turningRight = true;
    turnRight();
    return;
  }

  if (turningRight) {
    turnRight();
    if (center) {
      turningRight = false;
      lastTurn = 1;
    }
    return;
  }

  if (turningLeft) {
    turnLeft();
    if (center) {
      turningLeft = false;
      lastTurn = -1;
    }
    return;
  }


  if (center) {
    lastTurn = 0;

    if (left && !right) {
      slightLeft();
    } else if (right && !left) {
      slightRight();
    } else {
      forward(BASE_SPEED);
    }
    return;
  }

  if (left || farLeft) {
    turningLeft = true;
    turnLeft();
    return;
  }

  if (!farRight && !right && !center && !left && !farLeft) {
    turnAround();
    lastTurn = 1;
    return;
  }

  forward(SEARCH_SPEED);
}

// ================= SETUP =================
void setup() {
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SERVO, OUTPUT);

  strip.begin();
  ledsIdle();

  Serial.begin(9600);
  Serial.println("System Online");

  gripperOpen();
}

// ================= MAIN LOOP =================
void loop() {
  serviceGripper();

  if (finished) {
    stopMotors();
    return;
  }

  switch (mode) {
    case WAIT_OBJECT: {
      stopMotors();

      long distance = getDistanceCm();

      Serial.print("WAIT_OBJECT distance = ");
      Serial.println(distance);

      if (distance > 0 && distance <= OBJECT_DETECT_DISTANCE) {
        Serial.println("OBJECT DETECTED - WAIT 3s");
        unsigned long waitStart = millis();
        while (millis() - waitStart < 3000) {
          serviceGripper();
          stopMotors();
        }
        delayWithGripper(200);
        moveStartTime = millis();
        mode = GO_TO_OBJECT;
      }
      break;
    }

    case GO_TO_OBJECT:
      Serial.print("GO_TO_OBJECT time left = ");
      Serial.println(APPROACH_TIME - (millis() - moveStartTime));

      if (millis() - moveStartTime < APPROACH_TIME) {
        forwardToObject(APPROACH_SPEED);
      } else {
        stopMotors();
        delayWithGripper(200);
        mode = GRAB_OBJECT;
      }
      break;

    case GRAB_OBJECT:
      Serial.println("GRAB_OBJECT");
      stopMotors();
      delayWithGripper(200);

      gripperClose();
      delayWithGripper(400);

      actionStartTime = millis();
      mode = TURN_LEFT_90_AFTER_GRAB;
      break;

    case TURN_LEFT_90_AFTER_GRAB:
      Serial.println("TURN_LEFT_90_AFTER_GRAB");

      if (millis() - actionStartTime < LEFT_TURN_90_TIME) {
        pivotLeft();
        serviceGripper();
      } else {
        stopMotors();
        delayWithGripper(150);
        actionStartTime = millis();
        mode = GO_FORWARD_AFTER_GRAB;
      }
      break;

    case GO_FORWARD_AFTER_GRAB:
      Serial.println("GO_FORWARD_AFTER_GRAB");

      if (millis() - actionStartTime < FORWARD_AFTER_GRAB_TIME) {
        forward(APPROACH_SPEED);
        serviceGripper();
      } else {
        stopMotors();
        delayWithGripper(100);

        finishCandidate = false;
        mode = MAZE_MODE;
      }
      break;

    case MAZE_MODE:
      lineFollow();
      break;

    case FINISHED_MODE:
      stopMotors();
      break;
  }
}
