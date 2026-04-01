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
#define GRIPPER_OPEN  1620
#define GRIPPER_CLOSE 1100

// ================= LINE SENSORS =================
int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// ================= SPEED SETTINGS =================
const int BASE_SPEED = 200;
const int TURN_SPEED = 190;
const int SEARCH_SPEED = 180;
const int APPROACH_SPEED = 180;
const int LINE_TH = 800;

// ================= FINISH SETTINGS =================
const int FINISH_SLOW_SPEED = 160;
const unsigned long FINISH_CHECK_TIME = 500;

// ================= DISTANCE/TIME SETTINGS =================
const int OBJECT_DETECT_DISTANCE = 30;
const unsigned long APPROACH_TIME = 2000;
const unsigned long BACK_TIME = 3000;
const unsigned long LEFT_TURN_90_TIME = 800;
const unsigned long FORWARD_AFTER_GRAB_TIME = 700;

// ================= OBSTACLE SETTINGS =================
const int OBSTACLE_DISTANCE = 10;

// ================= STATE =================
int lastTurn = 0;   // -1 left, 0 straight, 1 right
bool turningRight = false;
bool turningLeft  = false;
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
void ledsBaseGreen() {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 40, 0));
  }
}

void ledsForward() {
  ledsBaseGreen();
  strip.setPixelColor(3, strip.Color(255, 0, 0)); // перед лев
  strip.setPixelColor(2, strip.Color(255, 0, 0)); // перед прав
  strip.show();
}

void ledsBack() {
  ledsBaseGreen();
  strip.setPixelColor(0, strip.Color(255, 0, 0)); // зад прав
  strip.setPixelColor(1, strip.Color(255, 0, 0)); // зад лев
  strip.show();
}

void ledsLeft() {
  ledsBaseGreen();
  strip.setPixelColor(3, strip.Color(255, 0, 0)); // перед лев
  strip.setPixelColor(0, strip.Color(255, 0, 0)); // зад лев
  strip.show();
}

void ledsRight() {
  ledsBaseGreen();
  strip.setPixelColor(2, strip.Color(255, 0, 0)); // перед прав
  strip.setPixelColor(1, strip.Color(255, 0, 0)); // зад прав
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

  analogWrite(LEFT_FWD, spd);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, constrain(spd + 40, 0, 255));
  analogWrite(RIGHT_BWD, 0);
}

void backward(int spd) {
  ledsBack();

  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, spd);
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
  analogWrite(RIGHT_BWD, 0);
}

void pivotLeft() {
  ledsLeft();

  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, constrain(BASE_SPEED + 20, 0, 255));
  analogWrite(RIGHT_FWD, BASE_SPEED);
  analogWrite(RIGHT_BWD, 0);
}

void slightLeft() {
  ledsLeft();

  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, BASE_SPEED);
  analogWrite(RIGHT_BWD, 0);
}

void slightRight() {
  ledsRight();

  analogWrite(LEFT_FWD, constrain(BASE_SPEED + 20, 0, 255));
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
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

  analogWrite(LEFT_FWD, spd - 10);
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
  holdGripper(GRIPPER_OPEN, 1000);
}

void gripperClose() {
  holdGripper(GRIPPER_CLOSE, 1200);
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
  long d = getDistanceCm();
  return (d > 0 && d < OBSTACLE_DISTANCE);
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
    if (v > LINE_TH) {
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
  for (int i = 0; i < 8; i++) {
    int v = analogRead(sensorPins[i]);
    if (v > LINE_TH) {
      blackCount++;
    }
  }
  return (blackCount >= 6);
}

// ================= SPECIAL ACTIONS =================
void turnAround() {
  ledsRight();
  analogWrite(LEFT_FWD, TURN_SPEED);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, TURN_SPEED);
  delayWithGripper(400);

  stopMotors();
  delayWithGripper(50);
}

void finishStop() {
  Serial.println("FINISH reached");

  stopMotors();
  delayWithGripper(300);

  unsigned long preBack = millis();
  while (millis() - preBack < 500) {
    backward(BASE_SPEED);
    serviceGripper();
  }
  stopMotors();
  delayWithGripper(200);

  gripperOpen();
  delayWithGripper(700);

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

  if (farRight || right) {
    turningRight = true;
    turnRight();
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
