// Pin mapping
const int LEFT_BACKWARD_PIN  = 10;
const int LEFT_FORWARD_PIN   = 5;
const int RIGHT_FORWARD_PIN  = 6;
const int RIGHT_BACKWARD_PIN = 9;

const int ROTATION_LEFT_PIN  = 2;
const int ROTATION_RIGHT_PIN = 3;

const int ULTRASOUND_TRIG_PIN = 11;
const int ULTRASOUND_ECHO_PIN = 12;

// Light sensors
const int NUM_SENSORS = 8;
const int LIGHT_SENSOR_PINS[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

// PID variables
float Kp = 0.12;
float Ki = 0.001;
float Kd = 0.02;

int previousError = 0;
int integral = 0;
int baseSpeed = 220;

// Sensor weights for line position (-left to +right)
int sensorWeights[NUM_SENSORS] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

// Sensor calibration offsets
int weights[NUM_SENSORS] = {310, 319, 338, 285, 296, 275, 245, 229};

// Sensor thresholds
const int LIGHT_SENSOR_WHITE_THRESHOLD = 550;
const int LIGHT_SENSOR_BLACK_THRESHOLD = 450;

// Motor calibration
const int CALIBRATION_FORWARD_LEFT  = 255;
const int CALIBRATION_BACKWARD_LEFT = 255;
const int CALIBRATION_FORWARD_RIGHT = 243;
const int CALIBRATION_BACKWARD_RIGHT = 210;

// Drive motors helper function
void driveMotors(int left, int right) {
  left  = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // Left motor
  if (left >= 0) {
    analogWrite(LEFT_FORWARD_PIN, left);
    analogWrite(LEFT_BACKWARD_PIN, 0);
  } else {
    analogWrite(LEFT_FORWARD_PIN, 0);
    analogWrite(LEFT_BACKWARD_PIN, -left);
  }

  // Right motor
  if (right >= 0) {
    analogWrite(RIGHT_FORWARD_PIN, right);
    analogWrite(RIGHT_BACKWARD_PIN, 0);
  } else {
    analogWrite(RIGHT_FORWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, -right);
  }
}

// Motor drive option helper (optional, can use driveMotors directly)
void drive(byte option) {
  if (option == 0) driveMotors(0, 0);
  if (option == 1) driveMotors(CALIBRATION_FORWARD_LEFT, CALIBRATION_FORWARD_RIGHT);
  if (option == 2) driveMotors(-CALIBRATION_BACKWARD_LEFT, -CALIBRATION_BACKWARD_RIGHT);
}

// Apply calibration to raw sensor readings
int applyLightSensorCalibration(int raw, int sensorIndex) {
  int v = raw + weights[sensorIndex];
  return constrain(v, 0, 1023);
}

// Read line position (-ve = left, +ve = right)
int readLinePosition() {
  long weightedSum = 0;
  int activeCount = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(LIGHT_SENSOR_PINS[i]);
    int calibrated = applyLightSensorCalibration(raw, i);

    // Black line detection: lower values = black
    if (calibrated < LIGHT_SENSOR_BLACK_THRESHOLD) {
      weightedSum += sensorWeights[i];
      activeCount++;
    }
  }

  if (activeCount == 0) {
    // Line lost: keep turning same direction
    return (previousError > 0) ? 3000 : -3000;
  }

  return weightedSum / activeCount;
}

// PID line-following
void PID_LineFollow(int error) {
  int P = error;
  integral += error;
  integral = constrain(integral, -1000, 1000);
  int D = error - previousError;

  int PIDvalue = (Kp * P) + (Ki * integral) + (Kd * D);
  previousError = error;

  int leftSpeed  = baseSpeed - PIDvalue;
  int rightSpeed = baseSpeed + PIDvalue;

  driveMotors(leftSpeed, rightSpeed);
}

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  // Rotation sensors
  pinMode(ROTATION_LEFT_PIN, INPUT_PULLUP);
  pinMode(ROTATION_RIGHT_PIN, INPUT_PULLUP);

  // Ultrasound sensor
  pinMode(ULTRASOUND_TRIG_PIN, OUTPUT);
  pinMode(ULTRASOUND_ECHO_PIN, INPUT);

  // Light sensors
  for (int i = 0; i < NUM_SENSORS; i++) pinMode(LIGHT_SENSOR_PINS[i], INPUT);

  // Stop motors initially
  drive(0);
}

void loop() {
  int position = readLinePosition();
  PID_LineFollow(position);

  // Optional: debug
  // Serial.println(position);
  delay(5); // small delay for stability
}
