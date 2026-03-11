// pin mapping
const int LEFT_FORWARD_PIN  = 3;
const int LEFT_BACKWARD_PIN = 10;
const int RIGHT_FORWARD_PIN = 11;
const int RIGHT_BACKWARD_PIN = 9;

// D1 = A0, D2 = A1, D3 = A2, D4 = A3, D5 = A4, D6 = A5, D7 = A6, D8 = A7
const int NUM_SENSORS = 7;
const int LINE_SENSOR_PINS[NUM_SENSORS] = {A0, A1, A2, A4, A5, A6, A7};
int lineValues[NUM_SENSORS]; // this keeps the sensor readings of the moment
/// pin A3 is used for trig sensor right ultrasound sensor
// sensor calibration
int weights[NUM_SENSORS] = {-273, -264, -253, -250, -276, -309, -322};

// thresholds
const int LIGHT_SENSOR_WHITE_THRESHOLD = 400;
const int LIGHT_SENSOR_BLACK_THRESHOLD = 600;

// for hysterysis, remembers what the sensor's last value was
bool sensorBlack[NUM_SENSORS] = {false};

// sensors declared here for flexibility from array
const int RIGHT_OUTER_SENSOR = 0;
const int RIGHT_MIDDLE_SENSOR = 1;
const int RIGHT_INNER_SENSOR = 2;
const int MIDDLE_SENSOR = 3;
const int LEFT_OUTER_SENSOR = 4;
const int LEFT_MIDDLE_SENSOR = 5;
const int LEFT_INNER_SENSOR = 6;


// speed settings
const int SPEED_NONE = 230; //Speed for all sensors detecting white (means black line is perfectly in the center)
const int SPEED_INNER = 200; //speed for when one of the inner sensors are on black
const int SPEED_MIDDLE = 180; //speed for when one of the middle sensors (second to edge) are on black
const int SPEED_EDGE = 0; //speed for when outer sensors (at the edge)are on black
const int RECOVERY_SPEED = 200; //only used if line is completely lost

const int CORRECTION_EDGE = 160;
const int CORRECTION_MIDDLE = 105;
const int CORRECTION_INNER = 75;

const int RIGHT_MOTOR_CALIBRATION = 0;
const int LEFT_MOTOR_CALIBRATION = 25;

void setup() {
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(LINE_SENSOR_PINS[i], INPUT);
  }
  
  stopMotors();
}

void loop() {
  followTheLine();
}

// calibration function (corrects raw sensor values)
int applyCalibration(int raw, int sensorIndex) {
  int v = raw + weights[sensorIndex];
  return constrain(v, 0, 1023);
}

void followTheLine() {

  // Read reflection sensor values
  for (int i = 0; i < NUM_SENSORS; i++) 
  {
    int raw = analogRead(LINE_SENSOR_PINS[i]);
    lineValues[i] = applyCalibration(raw,i); // apply calibration before using the value

/// Hysterisis logic, only update the sensor value as black if it goes past hysterisis
    if (lineValues[i] >= LIGHT_SENSOR_BLACK_THRESHOLD) {
      sensorBlack[i] = true;
    }
    else if (lineValues[i] <= LIGHT_SENSOR_WHITE_THRESHOLD) {
      sensorBlack[i] = false;
    }
  }

  // Convenience booleans, writes if the sensor is reading black or not (accounting for hysterisis too) to a boolean valye
  bool leftOuterBlack = sensorBlack[LEFT_OUTER_SENSOR];
  bool leftMiddleBlack = sensorBlack[LEFT_MIDDLE_SENSOR];
  bool leftInnerBlack = sensorBlack[LEFT_INNER_SENSOR];
  bool middleBlack = sensorBlack[MIDDLE_SENSOR];
  bool rightInnerBlack = sensorBlack[RIGHT_INNER_SENSOR];
  bool rightMiddleBlack = sensorBlack[RIGHT_MIDDLE_SENSOR];
  bool rightOuterBlack = sensorBlack[RIGHT_OUTER_SENSOR];

// Convenience booleans for if any of the left or right sensors read as black
  bool anyLeftBlack  = leftOuterBlack || leftMiddleBlack || leftInnerBlack;
  bool anyRightBlack = rightOuterBlack || rightMiddleBlack || rightInnerBlack;

///count how many sensors counted black
  int blackCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorBlack[i]) {
      blackCount++;
    }
  }

/// If all sensors are black
  if (blackCount == NUM_SENSORS) {
    driveForward(SPEED_NONE + LEFT_MOTOR_CALIBRATION, SPEED_NONE + RIGHT_MOTOR_CALIBRATION);
    return;
  }

///if any right sensor and no left sensors
    if (anyRightBlack && !anyLeftBlack) {
      // EDGE FIRST
      if(rightOuterBlack) {
        driveForward(SPEED_EDGE + CORRECTION_EDGE + LEFT_MOTOR_CALIBRATION, SPEED_EDGE + RIGHT_MOTOR_CALIBRATION);
        return;
      }

      // THEN MIDDLE
      else if(rightMiddleBlack) {
        driveForward(SPEED_MIDDLE + CORRECTION_MIDDLE + LEFT_MOTOR_CALIBRATION, SPEED_MIDDLE + RIGHT_MOTOR_CALIBRATION);
        return;
      }

      // THEN INNER
      else { //rightInnerBlack
        driveForward(SPEED_INNER + CORRECTION_INNER + LEFT_MOTOR_CALIBRATION, SPEED_INNER + RIGHT_MOTOR_CALIBRATION);
        return;
      }
      
    }
  // if any left sensors and no right sensors
    else if (!anyRightBlack && anyLeftBlack) {
      
      if (leftOuterBlack) {
        driveForward(SPEED_EDGE + LEFT_MOTOR_CALIBRATION, SPEED_EDGE + CORRECTION_EDGE + RIGHT_MOTOR_CALIBRATION);
        return;
      }

      else if(leftMiddleBlack) {
        driveForward(SPEED_MIDDLE + LEFT_MOTOR_CALIBRATION, SPEED_MIDDLE + CORRECTION_MIDDLE + RIGHT_MOTOR_CALIBRATION);
        return;
      }

      else { //leftInnerBlack
        driveForward(SPEED_INNER + LEFT_MOTOR_CALIBRATION, SPEED_INNER + CORRECTION_INNER + RIGHT_MOTOR_CALIBRATION);
        return;
      }
      
    }

    else if (rightInnerBlack) {
      driveForward(SPEED_INNER + CORRECTION_INNER + LEFT_MOTOR_CALIBRATION, SPEED_INNER + RIGHT_MOTOR_CALIBRATION);
      return;
    }

    else if (leftInnerBlack) {
      driveForward(SPEED_INNER + LEFT_MOTOR_CALIBRATION, SPEED_INNER + CORRECTION_INNER + RIGHT_MOTOR_CALIBRATION);
      return;
    }

  driveForward(SPEED_INNER + LEFT_MOTOR_CALIBRATION, SPEED_INNER + RIGHT_MOTOR_CALIBRATION);

}
 
// motor control
void driveForward(int left, int right) {
// constrain because sometimes value + correction goes over or under 0 and 255 respectively
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
  analogWrite(LEFT_FORWARD_PIN, left);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, right);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}

void driveBackward(int left, int right) {
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, left);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, right);
}

void stopMotors() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}
