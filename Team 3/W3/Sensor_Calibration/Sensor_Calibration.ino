// Pin mapping.
const int NUM_SENSORS = 8;
const int LIGHT_SENSOR_PINS[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

///Light sensors
///number of samples for the log
const int SENSOR_SAMPLES_AMOUNT = 67;
///sensor calibration
int weights[NUM_SENSORS] = {0, 0, 0, 0, 0, 0, 0, 0};
float whiteAvg[NUM_SENSORS] = {0};
float blackAvg[NUM_SENSORS] = {0};
long whiteAvgTotalAverage = 0;
long blackAvgTotalAverage = 0;
///the average targeted by the script, so greater than targetAvg is black, lower is white
int targetAvg = 500;


///2D log array and index
int sensorLog[NUM_SENSORS][SENSOR_SAMPLES_AMOUNT];
int logIndex = 0;
bool logFull = false;

///reset sensor calibration log
void resetLog() {
  logIndex = 0;
  logFull = false;
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
//  1 is white, 2 is black
  if (blackOrWhite == 1) {
    Serial.println("Put robot on white!");
  } else {
    Serial.println("Put robot on black!");
  }

//  /give user time to place robot
  delay (5000);

  resetLog();
  while (!logFull) {
    readLightSensorsandLog();
  }
  printLightSensorsLog(blackOrWhite);


  long totalAverage = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (blackOrWhite == 1) {
      totalAverage += (long)whiteAvg[i];
    } else {
      totalAverage += (long)blackAvg[i];
    }
  }
  
  if (blackOrWhite == 1) {
      whiteAvgTotalAverage = totalAverage / NUM_SENSORS;
  } else {
      blackAvgTotalAverage = totalAverage / NUM_SENSORS;
  }
  

  Serial.print("Total average: ");
  if (blackOrWhite == 1) {
    Serial.print(whiteAvgTotalAverage);
  } else {
    Serial.print(blackAvgTotalAverage);
  }
  Serial.print("\n");

}

void calculateLightSensorsCalibration() {

   getAvgBlackOrWhite(1);
   getAvgBlackOrWhite(2);

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
  
}

void setup() {
  Serial.begin(9600);
  
  for (int i = 0; i < 8; i++) pinMode(LIGHT_SENSOR_PINS[i], INPUT);

}

void loop() {
  calculateLightSensorsCalibration();
  delay (60000);
}
