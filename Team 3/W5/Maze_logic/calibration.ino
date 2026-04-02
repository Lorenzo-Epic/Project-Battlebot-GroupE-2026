// ~~~~~~~~~~~~~~~~~~~~~~ AUTO-CALIBRATION FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
// Apply calibration to one sensor based on it's weight, and constrain the result 
int applyLightSensorCalibration(int raw, int sensorIndex) {
    int v = raw + weights[sensorIndex];
    return constrain(v, LIGHT_SENSORS_MIN_READING, LIGHT_SENSORS_MAX_READING);
}

bool allSensorsBlack() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (!isBlack(i)) {
            return false;
        }
    }

    return true;
}

bool allSensorsWhite() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (!isWhite(i)) {
            return false;
        }
    }

    return true;
}

// Drive forward until condition met (keeps servo powered)
void driveForwardUntilAllBlack() {
    resetEncoderCounts();
    startForwardMotion();

    while (!allSensorsBlack()) {
        gripperUpdate();
        correctMotorSpeeds(
            LEFT_FORWARD_SPEED,
            RIGHT_FORWARD_SPEED,
            DIRECTION_FORWARD
        );
    }

    stopMotors();
}

void driveForwardUntilAllWhite() {
    resetEncoderCounts();
    startForwardMotion();

    while (!allSensorsWhite()) {
        gripperUpdate();
        correctMotorSpeeds(
            LEFT_FORWARD_SPEED,
            RIGHT_FORWARD_SPEED,
            DIRECTION_FORWARD
        );
    }

    stopMotors();
}

// Read all sensors once and store this set of readings in the log. at sensorLog[sensorIndex][sampleIndex]
void readLightSensorsAndLog() {
    if (logFull) {
        return;
    }

// gets one sensor reading per sensor and writes into the current logIndex
    for (int i = 0; i < NUM_SENSORS; i++) {
        int raw = analogRead(LINE_SENSOR_PINS[i]);
        sensorLog[i][logIndex] = raw;
    }

    logIndex++;

// Check if log is full, if so, set the boolean logFull to true
    if (logIndex >= SENSOR_SAMPLES_AMOUNT) {
        logFull = true;
        logIndex = SENSOR_SAMPLES_AMOUNT - 1;
    }
}


// Collect several calibration runs for either white or black, then update the saved average values for each sensor.
void getAvgBlackOrWhite(int blackOrWhite) {
    // Take multiple runs and accumulate
    for (int run = 0; run < SENSOR_SAMPLES_AMOUNT; run++) {
        collectAndStoreOneCalibrationRun(blackOrWhite);
    }
    // calculate the accumulated runs into average valyes
    updateCalibrationAverages(blackOrWhite);

    // increment to track how many runs have been used to calculate the averages
    if (blackOrWhite == WHITE) {
        whiteRuns += SENSOR_SAMPLES_AMOUNT;
    } else {
        blackRuns += SENSOR_SAMPLES_AMOUNT;
    }
}

void collectAndStoreOneCalibrationRun(int blackOrWhite) {
    resetLog();

    // fill up log with sensor readings
    while (!logFull) {
        readLightSensorsAndLog();
        gripperUpdate();
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
        long sum = 0;
        // sum up all sensor readings per sensor
        for (int g = 0; g < SENSOR_SAMPLES_AMOUNT; g++) {
            sum += sensorLog[i][g];
        }
        // Average of this sensor's samples during this one run.
        float avg = (float)sum / (float)SENSOR_SAMPLES_AMOUNT;

        // add this run to the sum array
        if (blackOrWhite == WHITE) {
            whiteSum[i] += avg;
        } else {
            blackSum[i] += avg;
        }
    }
}

// Convert the accumulated sums into average values for each sensor.
void updateCalibrationAverages(int blackOrWhite) {
    // float totalAverage = 0;
    // Accumulates all the per-run average sums into one final average across all runs collected so far per sensor
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (blackOrWhite == WHITE) {
            // Average white reading of sensor i across all white runs collected so far.
            whiteAvg[i] = whiteSum[i] / (float)(whiteRuns + SENSOR_SAMPLES_AMOUNT);
        } else {
            // Average black reading of sensor i across all black runs collected so far.
            blackAvg[i] = blackSum[i] / (float)(blackRuns + SENSOR_SAMPLES_AMOUNT);
        }
    }
}

// Reset sensor calibration log
void resetLog() {
    logIndex = 0;
    logFull = false;
}

void calculateLightSensorsCalibration() {
    // Per-sensor calibration (each sensor gets its own weight)
    for (int i = 0; i < NUM_SENSORS; i++) {
        // For each sensior, find the midpoint between it's average white reading and black reading
        long sensorMidPoint = (long)((whiteAvg[i] + blackAvg[i]) / 2.0f);
        // see how far sensor midpoint is from TARGET_AVG, that value becomes the final calibration weight
        weights[i] = (int)(TARGET_AVG - sensorMidPoint);
    }
}
