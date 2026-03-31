// ~~~~~~~~~~~~~~~~~~~~~~ AUTO-CALIBRATION FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
// Apply calibration
int applyLightSensorCalibration(int raw, int sensorIndex) {
    int v = raw + weights[sensorIndex];
    return constrain(v, LIGHT_SENSORS_MIN_READING, LIGHT_SENSORS_MAX_READING);
}

// Hysteresis logic
bool isBlack(int sensorIndex) {
    int raw = analogRead(LINE_SENSOR_PINS[sensorIndex]);
    int calibrated = applyLightSensorCalibration(raw, sensorIndex);
    return calibrated > LIGHT_SENSOR_BLACK_THRESHOLD;
}

bool isWhite(int sensorIndex) {
    int raw = analogRead(LINE_SENSOR_PINS[sensorIndex]);
    int calibrated = applyLightSensorCalibration(raw, sensorIndex);
    return calibrated < LIGHT_SENSOR_WHITE_THRESHOLD;
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

// Reading light sensors and logging SENSOR_SAMPLES_AMOUNT times into array
void readLightSensorsAndLog() {
    if (logFull) {
        return;
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
        int raw = analogRead(LINE_SENSOR_PINS[i]);
        sensorLog[i][logIndex] = raw;
    }

    logIndex++;

    if (logIndex >= SENSOR_SAMPLES_AMOUNT) {
        logFull = true;
        logIndex = SENSOR_SAMPLES_AMOUNT - 1;
    }
}

void getAvgBlackOrWhite(int blackOrWhite) {
    // 1 is white, 2 is black

    // Take multiple runs and accumulate
    for (int run = 0; run < CALIB_RUNS; run++) {
        resetLog();

        while (!logFull) {
            readLightSensorsAndLog();
            gripperUpdate();
        }

        // Compute this run's averages and add to sums
        for (int i = 0; i < NUM_SENSORS; i++) {
            long sum = 0;

            for (int g = 0; g < SENSOR_SAMPLES_AMOUNT; g++) {
                sum += sensorLog[i][g];
            }

            float avg = (float)sum / (float)SENSOR_SAMPLES_AMOUNT;

            // Adds average to white or black sum respectively
            if (blackOrWhite == WHITE) {
                whiteSum[i] += avg;
            } else {
                blackSum[i] += avg;
            }
        }
    }

    // Finalize the averages from accumulated sums
    long totalAverage = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (blackOrWhite == WHITE) {
            whiteAvg[i] = whiteSum[i] / (float)(whiteRuns + CALIB_RUNS);
            totalAverage += (long)whiteAvg[i];
        } else {
            blackAvg[i] = blackSum[i] / (float)(blackRuns + CALIB_RUNS);
            totalAverage += (long)blackAvg[i];
        }
    }

    if (blackOrWhite == WHITE) {
        whiteRuns += CALIB_RUNS;
        whiteAvgTotalAverage = totalAverage / NUM_SENSORS;
    } else {
        blackRuns += CALIB_RUNS;
        blackAvgTotalAverage = totalAverage / NUM_SENSORS;
    }
}

// Reset sensor calibration log
void resetLog() {
    logIndex = 0;
    logFull = false;
}

void calculateLightSensorsCalibration() {
    long midPoint = (whiteAvgTotalAverage + blackAvgTotalAverage) / 2;

    // Per-sensor calibration (each sensor gets its own weight)
    for (int i = 0; i < NUM_SENSORS; i++) {
        long sensorMidPoint = (long)((whiteAvg[i] + blackAvg[i]) / 2.0f);
        weights[i] = (int)(TARGET_AVG - sensorMidPoint);
    }

    Serial.print(F("Calibration values: {"));

    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(weights[i]);

        if (i < NUM_SENSORS - 1) {
            Serial.print(F(", "));
        }
    }

    Serial.print(F("}\n"));
}
