// pin mapping
const int LEFT_FORWARD_PIN  = 7;
const int LEFT_BACKWARD_PIN = 10;
const int RIGHT_FORWARD_PIN = 11;
const int RIGHT_BACKWARD_PIN = 9;

// D1 = A0, D2 = A1, D3 = A2, D4 = A3, D5 = A4, D6 = A5, D7 = A6, D8 = A7
const int NUMBER_OF_SENSORS = 8;
const int LINE_SENSOR[NUMBER_OF_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
int LINE_VALUES[NUMBER_OF_SENSORS]; // this keeps the sensor readings of the moment

// sensor calibration
int weights[NUMBER_OF_SENSORS] = {-237, -233, -224, -257, -250, -266, -287, -303};
int sensorWeights[NUMBER_OF_SENSORS] = {-4000,-2500,-500,-500,500,500,2500,4000};
/// original value: {-3500,-2500,-1500,-500,500,1500,2500,3500}

// thresholds
const int LIGHT_SENSOR_WHITE_THRESHOLD = 400;
const int LIGHT_SENSOR_BLACK_THRESHOLD = 600;

// speed settings
int baseSpeed = 120;
int correctionSpeed = 30;
int recovoerySpeed = 150; //only used of line is completely lost

//last black line detection
bool lastBlackDetected = false;

// setup
void setup() {

  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  for (int i = 0; i < NUMBER_OF_SENSORS; i++) pinMode(LINE_SENSOR[i], INPUT);

  stopMotors();
}

// line sensor loop
void loop() {
  defaultLineSensor();
  delay(10); // delay for some sensor stability
}

// calibration function (corrects raw sensor values)
int applyCalibration(int raw, int sensorIndex) {
  int v = raw + weights[sensorIndex];
  return constrain(v,0,1023);
}

// line sensor logic void

void defaultLineSensor() {
  int maxSensorValue = 0;

  // Read reflection sensor values
  for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
    int raw = analogRead(LINE_SENSOR[i]);
    LINE_VALUES[i] = applyCalibration(raw,i); // apply calibration before using the value

    if(LINE_VALUES[i] > maxSensorValue) maxSensorValue = LINE_VALUES[i];
  }

  // this part decides movement
  if(maxSensorValue >= LIGHT_SENSOR_BLACK_THRESHOLD) {
    lastBlackDetected = true; // remembers last time black line was seen

    // so center sensors see line then move forward
    if(LINE_VALUES[2] >= LIGHT_SENSOR_BLACK_THRESHOLD || LINE_VALUES[5] >= LIGHT_SENSOR_BLACK_THRESHOLD) {
      driveForward(baseSpeed + correctionSpeed, baseSpeed + correctionSpeed);
    }

    // if right sensors see line then turn right
    else if(LINE_VALUES[0] >= LIGHT_SENSOR_BLACK_THRESHOLD || LINE_VALUES[1] >= LIGHT_SENSOR_BLACK_THRESHOLD) {
      driveRight(baseSpeed + 40, baseSpeed + 50);
    }

    // of left sensors see line then turn left
    else if(LINE_VALUES[6] >= LIGHT_SENSOR_BLACK_THRESHOLD || LINE_VALUES[7] >= LIGHT_SENSOR_BLACK_THRESHOLD) {
      driveLeft(baseSpeed + 50, baseSpeed + 40);
    }

    else {
      stopMotors();
    } 
  }

  // recovery logic when all sensors see white
  else if(maxSensorValue <= LIGHT_SENSOR_WHITE_THRESHOLD && lastBlackDetected)  {
    driveBackward(recovoerySpeed, recovoerySpeed); // robot reverses until it finds the line again
  }

  else  {
    stopMotors();
  }
}

// motor control
void driveForward(int left, int right) {
  analogWrite(LEFT_FORWARD_PIN, left);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, right);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}

void driveBackward(int left, int right) {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, left);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, right);
}

void driveRight(int left, int right) {
  analogWrite(LEFT_FORWARD_PIN, left);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, right);
}

void driveLeft(int left, int right) {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, left);
  analogWrite(RIGHT_FORWARD_PIN, right);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}

void stopMotors() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}
