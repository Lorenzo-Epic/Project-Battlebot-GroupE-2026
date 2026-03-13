// 1. SETTINGS 

const bool INVERT_STEERING = true; // Keep this TRUE as per your fix 

const int BASE_SPEED = 250;        // Fast Speed 

const int SEARCH_SPEED = 230;      // Fast Search 

 

// ================================================================ 

 

const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7}; 

#define LEFT_FWD 5 

#define LEFT_BWD 6 

#define RIGHT_FWD 9 

#define RIGHT_BWD 10 

 

int minValues[8]; 

int maxValues[8]; 

int lastKnownDirection = 1; 

 

// ================= MOTOR CONTROL ================= 

void setMotors(int left, int right) { 

  left = constrain(left, -255, 255); 

  right = constrain(right, -255, 255); 

 

  if (left >= 0) { 

    analogWrite(LEFT_FWD, left); 

    analogWrite(LEFT_BWD, 0); 

  } else { 

    analogWrite(LEFT_FWD, 0); 

    analogWrite(LEFT_BWD, abs(left)); 

  } 

 

  if (right >= 0) { 

    analogWrite(RIGHT_FWD, right); 

    analogWrite(RIGHT_BWD, 0); 

  } else { 

    analogWrite(RIGHT_FWD, 0); 

    analogWrite(RIGHT_BWD, abs(right)); 

  } 

} 

 

// ================= SNAPSHOT CALIBRATION (NO SPIN) ================= 

void setup() { 

  Serial.begin(9600); 

  pinMode(LEFT_FWD, OUTPUT); 

  pinMode(LEFT_BWD, OUTPUT); 

  pinMode(RIGHT_FWD, OUTPUT); 

  pinMode(RIGHT_BWD, OUTPUT); 

 

  // --- INSTANT SNAPSHOT --- 

  // We assume the robot is CENTERED on the line. 

  // Sensors 3 and 4 are on Black. Sensors 0 and 7 are on White. 

 

  delay(500); // 0.5s pause to let sensors stabilize 

 

  int blackSample = (analogRead(A3) + analogRead(A4)) / 2; 

  int whiteSample = (analogRead(A0) + analogRead(A7)) / 2; 

 

  // Safety Check: If contrast is too low (e.g., in air), use defaults 

  if ((blackSample - whiteSample) < 100) { 

     blackSample = 800; 

     whiteSample = 100; 

  } 

 

  // Set the values for all sensors based on this snapshot 

  for(int i=0; i<8; i++) { 

    minValues[i] = whiteSample - 50; // Add buffer 

    if (minValues[i] < 0) minValues[i] = 0; 

 

    maxValues[i] = blackSample + 50; // Add buffer 

    if (maxValues[i] > 1023) maxValues[i] = 1023; 

  } 

} 

 

// ================= LOOP ================= 

void loop() { 

  long weightedSum = 0; 

  long sum = 0; 

  int activeSensors = 0; 

 

  for(int i=0; i<8; i++) { 

    int val = analogRead(sensorPins[i]); 

 

    // Map using the Snapshot values 

    int calibratedVal = map(val, minValues[i], maxValues[i], 0, 1000); 

    calibratedVal = constrain(calibratedVal, 0, 1000); 

 

    // Assuming Standard Sensors (Black = High) 

    // If your sensors are inverted (Black=Low), uncomment the line below: 

    // calibratedVal = 1000 - calibratedVal; 

 

    if(calibratedVal > 300) activeSensors++; 

 

    weightedSum += (long)calibratedVal * (i * 1000); 

    sum += calibratedVal; 

  } 

 

  // LOST LINE LOGIC 

  if (activeSensors == 0) { 

    if (lastKnownDirection == -1) setMotors(-SEARCH_SPEED, SEARCH_SPEED); 

    else setMotors(SEARCH_SPEED, -SEARCH_SPEED); 

    return; 

  } 

 

  // PID STEERING 

  int position = weightedSum / sum; 

  int error = position - 3500; 

 

  if (error < -500) lastKnownDirection = -1; 

  else if (error > 500) lastKnownDirection = 1; 

 

  int turnAdjustment = error * 0.12; 

 

  if (INVERT_STEERING) turnAdjustment = -turnAdjustment; 

 

  int leftSpeed = BASE_SPEED + turnAdjustment; 

  int rightSpeed = BASE_SPEED - turnAdjustment; 

 

  setMotors(leftSpeed, rightSpeed); 

} 
