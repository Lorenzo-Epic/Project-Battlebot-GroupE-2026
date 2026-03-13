// ================= MOTOR PINS =================
const int LEFT_FWD  = 11;
const int LEFT_BWD  = 9;
const int RIGHT_FWD = 10;
const int RIGHT_BWD = 3;

// ================= LINE SENSORS =================
int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// ================= SPEED SETTINGS =================
const int BASE_SPEED = 255;
const int TURN_SPEED = 255;
const int LINE_TH    = 800; // Threshold: Adjust based on your diagnostic test

// ================= STATE =================
int lastTurn = 0; // -1 for Left, 1 for Right, 0 for Straight

// ================= MOTOR CONTROL HELPERS =================
void stopMotors() {
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, 0);
}

void forward(int spd) {
  analogWrite(LEFT_FWD, spd-10);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, spd-10);
  analogWrite(RIGHT_BWD, 0);
}

void turnLeft() {
  // Pivot left: Left motor stops, Right motor moves forward
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 200);
  analogWrite(RIGHT_BWD, 0);
}

void turnRight() {
  // Pivot right: Right motor stops, Left motor moves forward
  analogWrite(LEFT_FWD, 200);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, 0);
}

// ================= LINE FOLLOWING LOGIC =================
void lineFollow() {
  bool leftSeen = false;
  bool rightSeen = false;
  int THRESHOLD = 400; // Lowered threshold for better sensitivity

  for (int i = 0; i < 8; i++) {
    int v = analogRead(sensorPins[i]);
    // Checking if value is LESS than threshold (Common for many IR arrays)
    if (i == 3 && v > 800) rightSeen = true;
    if (i == 4 && v > 800) leftSeen = true;
  }

  // Debugging: uncomment to see detection in Serial Monitor
  // Serial.print("L: "); Serial.print(leftSeen); Serial.print(" R: "); Serial.println(rightSeen);

  if (leftSeen && rightSeen) {
    lastTurn = 0;
    forward(BASE_SPEED);
  } else if (rightSeen) {
    lastTurn = 1;
    turnRight();
  } else if (leftSeen) {
    lastTurn = -1;
    turnLeft();
  } else {
    if (lastTurn == -1) turnLeft();
    else if (lastTurn == 1) turnRight();
    else forward(120);
  }
}

void setup() {
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.begin(9600);
  Serial.println("System Online: Racing Mode");
}

void loop() {
  lineFollow();
}
