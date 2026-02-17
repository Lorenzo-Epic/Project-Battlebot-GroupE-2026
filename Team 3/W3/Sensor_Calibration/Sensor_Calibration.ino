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

///Light sensors
///number of samples for the log
const int SENSOR_SAMPLES_AMOUNT = 75;
///sensor calibration
int weights[NUM_SENSORS] = {0, 0, 0, 0, 0, 0, 0, 0};

///2D log array and index
int sensorLog[NUM_SENSORS][SENSOR_SAMPLES_AMOUNT];
int logIndex = 0;
bool logFull = false;

// Motor PWM calibration.
const int CALIBRATION_FORWARD_LEFT = 255;
const int CALIBRATION_BACKWARD_LEFT = 255;
const int CALIBRATION_FORWARD_RIGHT = 243;
const int CALIBRATION_BACKWARD_RIGHT = 210;


// Lower PWM near turn target to reduce overshoot. (NOT VERIFIED YET)
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

///Only reading light sensors
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(LIGHT_SENSOR_PINS[i]);

    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(raw);
    Serial.print(" ");
  }
  Serial.print("\n");
}

///Reading light sensors and outputting to console
void readLightSensorsandLog() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(LIGHT_SENSOR_PINS[i]);

    if (!logFull) {
      sensorLog[i][logIndex] = raw;  // store into 2D array
    }
    
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(raw);
    Serial.print(" ");
  }

  if (!logFull) {
    logIndex++;
    if (logIndex >= SENSOR_SAMPLES_AMOUNT) {  
      logFull = true;
      logIndex = SENSOR_SAMPLES_AMOUNT - 1;   
  }
  
  Serial.print("\n");
  }
}

///Printing the average values of the light sensors from the logs for calibration
void printLightSensorsLog() {

  if (!logFull) {
    return;
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    long sum = 0;
    for (int g = 0; g < SENSOR_SAMPLES_AMOUNT; g++) {
      sum += sensorLog[i][g];
    }
    double avg = (double)sum / (double)SENSOR_SAMPLES_AMOUNT;
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" average: ");
    Serial.print(avg);
    Serial.print("\n");
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
  delay(100);
  readLightSensorsandLog();
  delay(100);
  printLightSensorsLog();
  
}
