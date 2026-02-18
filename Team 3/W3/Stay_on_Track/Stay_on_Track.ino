// pin mapping
const int LEFT_BACKWARD_PIN  = 10;
const int LEFT_FORWARD_PIN   = 5;
const int RIGHT_FORWARD_PIN  = 6;
const int RIGHT_BACKWARD_PIN = 9;

const int ROTATION_LEFT_PIN  = 2;
const int ROTATION_RIGHT_PIN = 3;

const int ULTRASOUND_TRIG_PIN = 11;
const int ULTRASOUND_ECHO_PIN = 12;

const int NUM_SENSORS = 8;
const int LIGHT_SENSOR_PINS[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

// PID
float Kp = 0.12;
float Ki = 0.001;
float Kd = 0.02; //make lower if robot still twitchy (oroginal value was 0.02)

int integral = 0;
int previousError = 0;
int baseSpeed = 255;   // reduced for stability
const int SPEED_CENTER = 255;   // A3/A4
const int SPEED_OUTER  = 200;   // A2/A5
const int SPEED_EDGE   = 150;   // A0/A1/A6/A7

int sensorWeights[NUM_SENSORS] = {-4000,-2500,-500,-500,500,500,2500,4000};
/// original value: {-3500,-2500,-1500,-500,500,1500,2500,3500}

// sensor calibration
int weights[NUM_SENSORS] = {-237, -233, -224, -257, -250, -266, -287, -303};

// thresholds
const int LIGHT_SENSOR_WHITE_THRESHOLD = 400;
const int LIGHT_SENSOR_BLACK_THRESHOLD = 600;

// apply calibration
int applyLightSensorCalibration(int raw, int sensorIndex) {
  int v = raw + weights[sensorIndex];
  return constrain(v, 0, 1023);
}

// read line position (A3/A4 using others for recovery)
int readLinePosition() {
  long weightedSum = 0;
  int activeCount = 0;

  // main sensors
  for (int i = 3; i <= 4; i++) {  // A3 and A4
    int raw = analogRead(LIGHT_SENSOR_PINS[i]);
    int calibrated = applyLightSensorCalibration(raw, i);

    if (calibrated > LIGHT_SENSOR_BLACK_THRESHOLD) {  // black is higher
      weightedSum += sensorWeights[i];
      activeCount++;
    }
  }

  // recovery sensors
  if (activeCount == 0) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (i >= 3 && i <= 4) continue;  // skip center again

      int raw = analogRead(LIGHT_SENSOR_PINS[i]);
      int calibrated = applyLightSensorCalibration(raw, i);

      if (calibrated > LIGHT_SENSOR_BLACK_THRESHOLD) {
        weightedSum += sensorWeights[i];
        activeCount++;
      }
    }
  }

  // if loses line
  if (activeCount == 0) {
    return previousError / 2;  // gently continue in last direction
  }

  return weightedSum / activeCount;
}

// motor control
void driveMotors(int left, int right) {
  left  = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (left >= 0) {
    analogWrite(LEFT_FORWARD_PIN, left);
    analogWrite(LEFT_BACKWARD_PIN, 0);
  } else {
    analogWrite(LEFT_FORWARD_PIN, 0);
    analogWrite(LEFT_BACKWARD_PIN, -left);
  }

  if (right >= 0) {
    analogWrite(RIGHT_FORWARD_PIN, right);
    analogWrite(RIGHT_BACKWARD_PIN, 0);
  } else {
    analogWrite(RIGHT_FORWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, -right);
  }
}

// PID controller
void PID_LineFollow(int error) {
  int P = error;
  integral += error;
  integral = constrain(integral, -800, 800);   // anti-windup
  int D = error - previousError;

  int PIDvalue = (Kp * P) + (Ki * integral) + (Kd * D);

  previousError = error;

  int leftSpeed  = baseSpeed - PIDvalue;
  int rightSpeed = baseSpeed + PIDvalue;

  driveMotors(leftSpeed, rightSpeed);
}

int computeBaseSpeed() {
  bool center = false;
  bool outer  = false;
  bool edge   = false;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(LIGHT_SENSOR_PINS[i]);
    int calibrated = applyLightSensorCalibration(raw, i);

    if (calibrated > LIGHT_SENSOR_BLACK_THRESHOLD) {
      if (i == 2 || i == 3 || i == 4 || i == 5) center = true;
      if (i == 1 || i == 6) outer = true;
      if (i == 0 || i == 7) edge = true;
    }
  }

  if (edge == true) {
    return SPEED_EDGE;
  }
  if (outer == true) {
    return SPEED_OUTER;
  }
  if (center == true) {
    return SPEED_CENTER;
  }

  return SPEED_EDGE; // lost line:
}

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  pinMode(ROTATION_LEFT_PIN, INPUT_PULLUP);
  pinMode(ROTATION_RIGHT_PIN, INPUT_PULLUP);

  pinMode(ULTRASOUND_TRIG_PIN, OUTPUT);
  pinMode(ULTRASOUND_ECHO_PIN, INPUT);

  for (int i = 0; i < NUM_SENSORS; i++)
    pinMode(LIGHT_SENSOR_PINS[i], INPUT);

  driveMotors(0, 0); // stop motors
}

void loop() {
  baseSpeed = computeBaseSpeed();
  int position = readLinePosition();
  PID_LineFollow(position);
}
