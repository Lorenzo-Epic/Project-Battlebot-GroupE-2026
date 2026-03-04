///IMPORTANT NOTE FOR THIS CODE:
///gripperUpdate() needs to be called in every long loop to keep servos powered

// Pin mapping.
const int NUM_SENSORS = 4;
const int LIGHT_SENSOR_PINS[NUM_SENSORS] = {A1, A2, A3, A4};
///purple = D3=A1, blue = D4=A2, yellow = D5=A3, green = D6=A4
///Light sensors
///number of samples for the log
const int SENSOR_SAMPLES_AMOUNT = 10;
// sensor calibration
int weights[NUM_SENSORS] = {-224, -257, -250, -266};
float whiteAvg[NUM_SENSORS] = {0};
float blackAvg[NUM_SENSORS] = {0};
long whiteAvgTotalAverage = 0;
long blackAvgTotalAverage = 0;
///the average targeted by the script, so greater than targetAvg is black, lower is white
int targetAvg = 500;

// 4 sensors mapped from original 8-sensor array: old 3,4,5,6
// index: 0=left outer, 1=left center, 2=right center, 3=right outer
const int S_LEFT_OUTER   = 0;
const int S_LEFT_CENTER  = 1;
const int S_RIGHT_CENTER = 2;
const int S_RIGHT_OUTER  = 3;

const int CALIB_RUNS = 5;            // number of averaged passes per color
float whiteSum[NUM_SENSORS] = {0};
float blackSum[NUM_SENSORS] = {0};
int whiteRuns = 0;
int blackRuns = 0;

int initialApproximateCalibration[NUM_SENSORS] = {0, 0, 0, 0};

///2D log array and index
int sensorLog[NUM_SENSORS][SENSOR_SAMPLES_AMOUNT];
int logIndex = 0;
bool logFull = false;

bool lineSensorsCalibrated = false;

///LIGHT SENSORS

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

int sensorWeights[NUM_SENSORS] = {-1500,-500,500,1500};
/// original value: {-3500,-2500,-1500,-500,500,1500,2500,3500}

// thresholds
const int LIGHT_SENSOR_WHITE_THRESHOLD = 400;
const int LIGHT_SENSOR_BLACK_THRESHOLD = 600;


// ---------------- Servo / Gripper ----------------
#define SERVO_PIN 11 
#define GRIPPER_OPEN_US  1820
#define GRIPPER_CLOSE_US 1000

// ---------------- Motors ----------------
#define LEFT_FORWARD_PIN 5
#define LEFT_BACKWARD_PIN 10 
#define RIGHT_FORWARD_PIN 6
#define RIGHT_BACKWARD_PIN 9

// Motor PWM calibration
#define PWM_LEFT_FWD 255
#define PWM_LEFT_BWD 255
#define PWM_RIGHT_FWD 238
#define PWM_RIGHT_BWD 210

#define PWM_LEFT_FWD_SLOW 240
#define PWM_LEFT_BWD_SLOW 230
#define PWM_RIGHT_FWD_SLOW 210
#define PWM_RIGHT_BWD_SLOW 210

// ---------------- Encoders ----------------
#define ROTATION_LEFT_PIN 2
#define ROTATION_RIGHT_PIN 3

// Wheel geometry
#define WHEEL_DIAMETER_CM 6.5f
#define SLOTS_PER_REV 20
#define EDGES_PER_SLOT 2
#define TICKS_PER_REV (SLOTS_PER_REV * EDGES_PER_SLOT)
#define WHEEL_CIRC_CM (PI * WHEEL_DIAMETER_CM)
#define TICKS_PER_CM ((float)TICKS_PER_REV / WHEEL_CIRC_CM)

#define NUMBER_OF_BLACK_LINES_INITIAL_CALIBRATION 3


// Shared with interrupts
volatile unsigned long leftTicks = 0;
volatile unsigned long rightTicks = 0;

///Sensor calibration thingy:
// apply calibration
int applyLightSensorCalibration(int raw, int sensorIndex) {
  int v = raw + weights[sensorIndex];
  return constrain(v, 0, 1023);
}

// read line position (A3/A4 using others for recovery)
int readLinePosition() {
  long weightedSum = 0;
  int activeCount = 0;

  // MAIN sensors: the two center sensors (old 4 and 5)
  for (int i = S_LEFT_CENTER; i <= S_RIGHT_CENTER; i++) { // 1..2
    int raw = analogRead(LIGHT_SENSOR_PINS[i]);
    int calibrated = applyLightSensorCalibration(raw, i);

    if (calibrated > LIGHT_SENSOR_BLACK_THRESHOLD) {
      weightedSum += sensorWeights[i];
      activeCount++;
    }
  }

  // RECOVERY sensors: the two outer-adjacent sensors (old 3 and 6)
  if (activeCount == 0) {
    int i = S_LEFT_OUTER;
    {
      int raw = analogRead(LIGHT_SENSOR_PINS[i]);
      int calibrated = applyLightSensorCalibration(raw, i);
      if (calibrated > LIGHT_SENSOR_BLACK_THRESHOLD) {
        weightedSum += sensorWeights[i];
        activeCount++;
      }
    }

    i = S_RIGHT_OUTER;
    {
      int raw = analogRead(LIGHT_SENSOR_PINS[i]);
      int calibrated = applyLightSensorCalibration(raw, i);
      if (calibrated > LIGHT_SENSOR_BLACK_THRESHOLD) {
        weightedSum += sensorWeights[i];
        activeCount++;
      }
    }
  }

  // LOST line: gently keep turning same way
  if (activeCount == 0) {
    return previousError / 2;
  }

  return (int)(weightedSum / activeCount);
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

  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(LIGHT_SENSOR_PINS[i]);
    int calibrated = applyLightSensorCalibration(raw, i);

    if (calibrated > LIGHT_SENSOR_BLACK_THRESHOLD) {
      if (i == S_LEFT_CENTER || i == S_RIGHT_CENTER) center = true;
      if (i == S_LEFT_OUTER  || i == S_RIGHT_OUTER)  outer  = true;
    }
  }

  // If outer sees black, you are further off-center -> slow down
  if (outer)  return SPEED_OUTER;
  if (center) return SPEED_CENTER;

  // lost line
  return SPEED_OUTER;
}
///methods etc

// Count every edge of the wheel encoders (40x per rotation instead of 20x)
void isrLeft() { 
  leftTicks++;
}
void isrRight() {
  rightTicks++; 
}

///reset sensor calibration log
void resetLog() {
  logIndex = 0;
  logFull = false;
}

// Add these helper functions somewhere above loop() (e.g. after computeBaseSpeed())

bool isBlack(int sensorIndex) {
  int raw = analogRead(LIGHT_SENSOR_PINS[sensorIndex]);
  int calibrated = applyLightSensorCalibration(raw, sensorIndex);
  return (calibrated > LIGHT_SENSOR_BLACK_THRESHOLD);
}

bool isWhite(int sensorIndex) {
  int raw = analogRead(LIGHT_SENSOR_PINS[sensorIndex]);
  int calibrated = applyLightSensorCalibration(raw, sensorIndex);
  return (calibrated < LIGHT_SENSOR_WHITE_THRESHOLD);
}

bool allSensorsBlack() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!isBlack(i)) return false;
  }
  return true;
}

bool allSensorsWhite() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!isWhite(i)) return false;
  }
  return true;
}

// True when BOTH centers are white AND BOTH outers are white
bool outerAndCenterAllWhite() {
  return isWhite(S_LEFT_OUTER) && isWhite(S_LEFT_CENTER) &&
         isWhite(S_RIGHT_CENTER) && isWhite(S_RIGHT_OUTER);
}

// Drive forward slowly until condition met (keeps servo powered)
void driveForwardUntilAllBlack() {
  driveForwardCalibrated(2); // uses your PWM calibration
  while (!allSensorsBlack()) {
    gripperUpdate();
  }
  stopMotors();
}

void driveForwardUntilAllWhite() {
  driveForwardCalibrated(2); // uses your PWM calibration
  while (allSensorsBlack()) {
    gripperUpdate();
  }
  stopMotors();
}

///Reading light sensors and logging SENSOR_SAMPLES_AMOUNT x into array
void readLightSensorsandLog() {
  if (logFull) {
    return;
  }
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(LIGHT_SENSOR_PINS[i]);
    sensorLog[i][logIndex] = raw;  // store into 2D array
  }

  logIndex++;
  if (logIndex >= SENSOR_SAMPLES_AMOUNT) {
    logFull = true;
    logIndex = SENSOR_SAMPLES_AMOUNT - 1;
  }
}

///Printing the average values of the light sensors from the logs for calibration and writing to the white/black avg arrays
void printLightSensorsLog(int blackOrWhite) {
//  /1 is white, 2 is black

  if (!logFull) {
    return;
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    long sum = 0;
    for (int g = 0; g < SENSOR_SAMPLES_AMOUNT; g++) {
      sum += sensorLog[i][g];
    }
    float avg = (float)sum / (float)SENSOR_SAMPLES_AMOUNT;
    
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" average: ");
    Serial.print(avg);
    Serial.print("\n");

    if (blackOrWhite == 1) {
      whiteAvg[i] = avg;   
    } else {
      blackAvg[i] = avg;
    }
    
  }
}

void getAvgBlackOrWhite(int blackOrWhite) {
  // 1 is white, 2 is black
  if (blackOrWhite == 1) {
    Serial.println("Put robot on white!");
  }
  else {
    Serial.println("Put robot on black!");
  }

  // give user time to place robot
  waitMs(5000);

  // take multiple runs and accumulate
  for (int run = 0; run < CALIB_RUNS; run++) {
    resetLog();
    while (!logFull) {
      readLightSensorsandLog();
      gripperUpdate(); // keep servo powered during sampling
    }

    // compute this run's averages and add to sums
    for (int i = 0; i < NUM_SENSORS; i++) {
      long sum = 0;
      for (int g = 0; g < SENSOR_SAMPLES_AMOUNT; g++) {
        sum += sensorLog[i][g];
      }
      float avg = (float)sum / (float)SENSOR_SAMPLES_AMOUNT;

      if (blackOrWhite == 1) whiteSum[i] += avg;
      else                   blackSum[i] += avg;
    }

    waitMs(30); // small pause between runs
  }

  // finalize the averages from accumulated sums
  long totalAverage = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (blackOrWhite == 1) {
      whiteAvg[i] = whiteSum[i] / (float)(whiteRuns + CALIB_RUNS);
      totalAverage += (long)whiteAvg[i];
    } else {
      blackAvg[i] = blackSum[i] / (float)(blackRuns + CALIB_RUNS);
      totalAverage += (long)blackAvg[i];
    }
  }

  if (blackOrWhite == 1) {
    whiteRuns += CALIB_RUNS;
    whiteAvgTotalAverage = totalAverage / NUM_SENSORS;
    Serial.print("White total average: ");
    Serial.println(whiteAvgTotalAverage);
  } else {
    blackRuns += CALIB_RUNS;
    blackAvgTotalAverage = totalAverage / NUM_SENSORS;
    Serial.print("Black total average: ");
    Serial.println(blackAvgTotalAverage);
  }

  // optional per-sensor print (kept similar to your original style)
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" average: ");
    if (blackOrWhite == 1) {
      Serial.println(whiteAvg[i]);
    } else {
      Serial.println(blackAvg[i]);
    }
  }
}

void calculateLightSensorsCalibration() {

//   getAvgBlackOrWhite(1);/
//   getAvgBlackOrWhite(2);/
///THESE NEED TO BE RUN BEFORE NOW

   long midPoint = (whiteAvgTotalAverage + blackAvgTotalAverage) / 2;

   Serial.print("White total avg: ");
   Serial.print(whiteAvgTotalAverage);
   Serial.print("\n");

   Serial.print("Black total avg: ");
   Serial.print(blackAvgTotalAverage);
   Serial.print("\n");

   Serial.print("Midpoint: ");
   Serial.print(midPoint);
   Serial.print("\n");

   // Per-sensor calibration (each sensor gets its own weight)
   for (int i = 0; i < NUM_SENSORS; i++) {
     long sensorMidPoint = (long)((whiteAvg[i] + blackAvg[i]) / 2.0f);
     weights[i] = (int)(targetAvg - sensorMidPoint);

     Serial.print("Sensor ");
     Serial.print(i);
     Serial.print(" midpoint: ");
     Serial.print(sensorMidPoint);
     Serial.print(" weight: ");
     Serial.print(weights[i]);
     Serial.print("\n");
    
   }

   Serial.print("{");
   for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(weights[i]);
    if (i < NUM_SENSORS - 1) {
      Serial.print(", ");
    }
   }
  Serial.print("}\n");
}

// ---------------- helper functions ----------------
void stopMotors() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);

  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
}
//1 = fast, 2 = slow
void driveForwardCalibrated(int speed) {
  if (speed == 1) {
    analogWrite(LEFT_BACKWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, 0);

    analogWrite(LEFT_FORWARD_PIN, PWM_LEFT_FWD);
    analogWrite(RIGHT_FORWARD_PIN, PWM_RIGHT_FWD);
  } else {
    analogWrite(LEFT_BACKWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, 0);

    analogWrite(LEFT_FORWARD_PIN, PWM_LEFT_FWD_SLOW);
    analogWrite(RIGHT_FORWARD_PIN, PWM_RIGHT_FWD_SLOW);
  }

}

void resetEncoders() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

unsigned long averageTicks() {
  unsigned long l;
  unsigned long r;
  noInterrupts();
  l = leftTicks;
  r = rightTicks;
  interrupts();
  return (l + r) / 2;
}

// Drive forward x cm, calculated with encoder ticks
// int gripper 0 = gripper close
// int gripper 1 = gripper open
void driveForwardCmWithGripper(int cm, int gripper) {

  long targetTicks = (long)((float)cm * TICKS_PER_CM + 0.5f);
  if (targetTicks <= 0) return;

  if (gripper == 0) closeGripper();
  else openGripper();

  resetEncoders();
  driveForwardCalibrated(1);

  while ((long)averageTicks() < targetTicks) {
    gripperUpdate(); // keep servo powered
  }

  stopMotors();
}

// ---------------- Gripper (powered indefinitely, non-blocking) ----------------
volatile int gripperPulseUs = GRIPPER_OPEN_US;   // target pulse width
unsigned long lastServoMs = 0;

void gripperUpdate() {
  // Send one servo pulse every ~20ms to keep it powered
  unsigned long now = millis();
  if (now - lastServoMs >= 20) {
    lastServoMs = now;
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(gripperPulseUs);
    digitalWrite(SERVO_PIN, LOW);
  }
}

// Non-blocking commands: just set target
void openGripper()  { gripperPulseUs = GRIPPER_OPEN_US; }
void closeGripper() { gripperPulseUs = GRIPPER_CLOSE_US; }

// Replace ALL delay(ms) with this (keeps servo powered during waits)
void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    gripperUpdate();
  }
}

void setup() {
  Serial.begin(9600);
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(LIGHT_SENSOR_PINS[i], INPUT);
  }

  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  pinMode(ROTATION_LEFT_PIN, INPUT_PULLUP);
  pinMode(ROTATION_RIGHT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ROTATION_LEFT_PIN), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTATION_RIGHT_PIN), isrRight, CHANGE);

  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  lastServoMs = millis(); // start timing for gripper pulses

  stopMotors();

}

void loop() {

  // ----------- AUTO CALIBRATION PHASE -----------
  if (!lineSensorsCalibrated) {

    for (int i = 0; i < NUMBER_OF_BLACK_LINES_INITIAL_CALIBRATION; i++) {

      // Wait until robot is clearly on white with all 4 sensors
      // Then calibrate WHITE once.
      while (!outerAndCenterAllWhite()) {
        gripperUpdate();
        stopMotors(); // don't move while waiting for placement
      }
      getAvgBlackOrWhite(1); // calibrate WHITE
  
      // Now move forward until ALL sensors see black, then calibrate BLACK.
      driveForwardUntilAllBlack();
      getAvgBlackOrWhite(2); // calibrate BLACK
  
      // Compute weights from the two captured averages
      calculateLightSensorsCalibration();      

      //drive forward to next white
      driveForwardUntilAllWhite();
      
      // small settle
      waitMs(200);
    }
    lineSensorsCalibrated = true;
  }

  // ----------- NORMAL LINE FOLLOWING PHASE -----------
  baseSpeed = computeBaseSpeed();
  int position = readLinePosition();
  PID_LineFollow(position);

  gripperUpdate(); // keep servo powered
}
