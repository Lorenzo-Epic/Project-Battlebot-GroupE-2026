// ================= TIMING SETTINGS (ms) =================
const unsigned long T_STOP1       = 200;
const unsigned long T_TURN_RIGHT  = 200;  // Time for ~90 degree right turn
const unsigned long T_GO_RIGHT    = 1200;  // Shortened: Side-step distance
const unsigned long T_STOP2       = 200;
const unsigned long T_TURN_LEFT   = 400;  // Capped at under 1s to face forward/parallel
const unsigned long T_GO_STRAIGHT = 1100; // Time to drive past the obstacle
const unsigned long T_TURN_RIGHT2 = 240;  // Re-align to original lane

// ================= MOTOR PINS =================
const int LEFT_FWD  = 11;
const int LEFT_BWD  = 10;
const int RIGHT_FWD = 6;
const int RIGHT_BWD = 5;

// ================= ULTRASONIC =================
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;

// ================= SPEED SETTINGS =================
const int BASE_SPEED = 255;
const int TURN_SPEED = 220;

// ================= OBSTACLE SETTINGS =================
const int OBSTACLE_DIST  = 18;
const int MIN_VALID_DIST = 3;

// ================= GLOBAL STATE =================
bool avoiding = false;
int avoidStage = 0;
unsigned long stageStartTime = 0;

// ================= MOTOR FUNCTIONS =================
void stopMotors() {
  analogWrite(LEFT_FWD, 0); analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_BWD, 0);
}

void forward(int spd) {
  analogWrite(LEFT_FWD, spd); analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, spd); analogWrite(RIGHT_BWD, 0);
}

void turnRight() {
  analogWrite(LEFT_FWD, TURN_SPEED); analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_BWD, TURN_SPEED);
}

void turnLeft() {
  analogWrite(LEFT_FWD, 0); analogWrite(LEFT_BWD, 250);
  analogWrite(RIGHT_FWD, 250); analogWrite(RIGHT_BWD, 0);
}

// ================= SENSOR FUNCTION =================
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

// ================= AVOIDANCE LOGIC =================
void startAvoid() {
  avoiding = true;
  avoidStage = 0;
  stageStartTime = millis();
}

void updateAvoid() {
  unsigned long now = millis();

  switch (avoidStage) {
    case 0: // Stop
      stopMotors();
      if (now - stageStartTime >= T_STOP1) {
        avoidStage = 1; stageStartTime = now;
      }
      break;

    case 1: // Stage 1: Turn Right 90deg
      turnRight();
      if (now - stageStartTime >= T_TURN_RIGHT) {
        avoidStage = 2; stageStartTime = now;
      }
      break;

    case 2: // Stage 2: Move to the side
      forward(250);
      if (now - stageStartTime >= T_GO_RIGHT) {
        avoidStage = 3; stageStartTime = now;
      }
      break;

    case 3: // Stop
      stopMotors();
      if (now - stageStartTime >= T_STOP2) {
        avoidStage = 4; stageStartTime = now;
      }
      break;

    case 4: // Stage 3: Turn Left (Max 1s)
      turnLeft();
      if (now - stageStartTime >= T_TURN_LEFT) {
        avoidStage = 5; stageStartTime = now;
      }
      break;

    case 5: // Stage 4: Drive past the object
      forward(250);
      if (now - stageStartTime >= T_GO_STRAIGHT) {
        avoidStage = 6; stageStartTime = now;
      }
      break;

    case 6: // Stage 5: Re-align
      turnRight();
      if (now - stageStartTime >= T_TURN_RIGHT2) {
        avoidStage = 7; stageStartTime = now;
      }
      break;

    case 7: // Finish
      avoiding = false;
      Serial.println("Maneuver Complete.");
      break;
  }
}

// ================= MAIN SETUP =================
void setup() {
  pinMode(LEFT_FWD, OUTPUT); pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT); pinMode(RIGHT_BWD, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
 
  Serial.begin(9600);
  Serial.println("Robot Ready.");
}

// ================= MAIN LOOP =================
void loop() {
  if (avoiding) {
    updateAvoid();
  }
  else {
    long d = getDistance();
   
    // Debugging info
    Serial.print("Distance: "); Serial.println(d);

    if (d >= MIN_VALID_DIST && d < OBSTACLE_DIST) {
      Serial.println("!!! OBSTACLE DETECTED !!!");
      startAvoid();
    }
    else {
      forward(BASE_SPEED);
    }
  }
  delay(30);
}
