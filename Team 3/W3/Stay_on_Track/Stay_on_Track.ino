// this is a WIP will work on it more later.

// pin mapping
const int LEFT_BACKWARD_PIN  = 10;
const int LEFT_FORWARD_PIN   = 5;
const int RIGHT_FORWARD_PIN  = 6;
const int RIGHT_BACKWARD_PIN = 9;

const int NUM_SENSORS = 8;
const int LIGHT_SENSOR_PINS[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

// thresholds
const int LIGHT_SENSOR_WHITE_THRESHOLD = 400;
const int LIGHT_SENSOR_BLACK_THRESHOLD = 600;

// global variable
int previousError = 0;  // used in readLinePosition() to continue last direction

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  for (int i = 0; i < NUM_SENSORS; i++)
    pinMode(LIGHT_SENSOR_PINS[i], INPUT);

  driveMotors(0, 0); // stop motors
}

void loop() {

  int s3 = analogRead(LIGHT_SENSOR_PINS[3]);
  int s4 = analogRead(LIGHT_SENSOR_PINS[4]);

  if(s3 > LIGHT_SENSOR_BLACK_THRESHOLD && s4 > LIGHT_SENSOR_BLACK_THRESHOLD){
      driveMotors(255,255);   // straight
  }
  else if(s3 > LIGHT_SENSOR_BLACK_THRESHOLD){
      driveMotors(150,255);   // slight left
  }
  else if(s4 > LIGHT_SENSOR_BLACK_THRESHOLD){
      driveMotors(255,150);   // slight right
  }
  else{
      driveMotors(150,-150);  // search
  }
}

int baseSpeed = 255;   // reduced for stability
const int SPEED_CENTER = 255;   // A3/A4
const int SPEED_OUTER  = 200;   // A2/A5
const int SPEED_EDGE   = 150;   // A0/A1/A6/A7

int sensorWeights[NUM_SENSORS] = {-4000,-2500,-500,-500,500,500,2500,4000};
/// original value: {-3500,-2500,-1500,-500,500,1500,2500,3500}

// sensor calibration
int weights[NUM_SENSORS] = {-237, -233, -224, -257, -250, -266, -287, -303};


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
