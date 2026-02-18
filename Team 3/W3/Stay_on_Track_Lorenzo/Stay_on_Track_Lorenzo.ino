// Pin mapping.
const int LEFT_BACKWARD_PIN = 10;
const int LEFT_FORWARD_PIN = 5;
const int RIGHT_FORWARD_PIN = 6;
const int RIGHT_BACKWARD_PIN = 9;

const int ROTATION_LEFT_PIN = 2;
const int ROTATION_RIGHT_PIN = 3;

const int ULTRASOUND_TRIG_PIN = 11;
const int ULTRASOUND_ECHO_PIN = 12;

const int NUM_SENSORS = 8;
const int LIGHT_SENSOR_PINS[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
/// light sensor calibration
int weights[NUM_SENSORS] = {310, 319, 338, 285, 296, 275, 245, 229};
const int LIGHT_SENSOR_WHITE_THRESHOLD = 450; //adjust these two later, calibrate with the black lines
const int LIGHT_SENSOR_BLACK_THRESHOLD = 550;
///high values are black
///low values are white

// Motor PWM calibration.
const int CALIBRATION_FORWARD_LEFT = 255;
const int CALIBRATION_BACKWARD_LEFT = 255;
const int CALIBRATION_FORWARD_RIGHT = 243;
const int CALIBRATION_BACKWARD_RIGHT = 210;

// Lower PWM at turn to reduce overshoot. (NOT VERIFIED YET)
const int TURN_SLOW_LEFT_FORWARD = 150;
const int TURN_SLOW_LEFT_BACKWARD = 150;
const int TURN_SLOW_RIGHT_FORWARD = 145;
const int TURN_SLOW_RIGHT_BACKWARD = 135;

/// 0 = stop
/// 1 = drive forward
/// 2 = drive backward
/// 3 = turn left in place
/// 4 = turn right in place
void drive(byte option) {
  if (option == 0) {
    analogWrite(LEFT_FORWARD_PIN, 0);
    analogWrite(LEFT_BACKWARD_PIN, 0);
    analogWrite(RIGHT_FORWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, 0);
  
    digitalWrite(LEFT_FORWARD_PIN, LOW);
    digitalWrite(LEFT_BACKWARD_PIN, LOW);
    digitalWrite(RIGHT_FORWARD_PIN, LOW);
    digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  }
  if (option == 1) {
    analogWrite(LEFT_BACKWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, 0);
  
    analogWrite(LEFT_FORWARD_PIN, CALIBRATION_FORWARD_LEFT);
    analogWrite(RIGHT_FORWARD_PIN, CALIBRATION_FORWARD_RIGHT);
  }
  if (option == 2) {
    analogWrite(LEFT_FORWARD_PIN, 0);
    analogWrite(RIGHT_FORWARD_PIN, 0);

   analogWrite(LEFT_BACKWARD_PIN, CALIBRATION_BACKWARD_LEFT);
   analogWrite(RIGHT_BACKWARD_PIN, CALIBRATION_BACKWARD_RIGHT);
  }
  if (option == 3) {
    analogWrite(LEFT_FORWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, 0);

    analogWrite(LEFT_BACKWARD_PIN, CALIBRATION_BACKWARD_LEFT);
    analogWrite(RIGHT_FORWARD_PIN, CALIBRATION_FORWARD_RIGHT);
  }
  if (option == 4) {
    analogWrite(LEFT_BACKWARD_PIN, 0);
    analogWrite(RIGHT_FORWARD_PIN, 0);

    analogWrite(LEFT_FORWARD_PIN, CALIBRATION_FORWARD_LEFT);
    analogWrite(RIGHT_BACKWARD_PIN, CALIBRATION_BACKWARD_RIGHT);
  }
}

int applyLightSensorCalibration(int raw, int sensorIndex) {
  int v = raw + weights[sensorIndex];
  return constrain(v, 0, 1023);
}

///Only reading light sensors
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(LIGHT_SENSOR_PINS[i]);
    int calibrated = applyLightSensorCalibration(raw, i);

    Serial.print("Sensor (Calibrated)");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(calibrated);
    Serial.print(" ");
  }
  Serial.print("\n");
}

void readLine() {

//  0 is ignore, 1 is white, 2 is black
  int sensorsBlackAndWhiteReadout[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(LIGHT_SENSOR_PINS[i]);
    int calibrated = applyLightSensorCalibration(raw, i);

    if (calibrated < LIGHT_SENSOR_WHITE_THRESHOLD) {
      sensorsBlackAndWhiteReadout[i] = 1; //if lower than white threshold (means its white), mark as 1
      
    } else if (calibrated > LIGHT_SENSOR_BLACK_THRESHOLD) {
      sensorsBlackAndWhiteReadout[i] = 2; //if higher than black threshold (means its black), mark as 2
    }
    
    }
  
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
  
  for (int i = 0; i < 8; i++) pinMode(LIGHT_SENSOR_PINS[i], INPUT);

  drive(0);
}

void loop() {
  delay(1000);
  readSensors();
  
}
