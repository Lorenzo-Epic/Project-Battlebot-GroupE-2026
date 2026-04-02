// ~~~~~~~~~~~~~~~~~~~~~~ LINE SENSORS / FOLLOWING FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
// updates the sensorBlack array on the line sensors to track which sensors are reading black or white, uses hysterisis
void updateLineSensors() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        int calibrated = readCalibratedLineSensor(i);

        if (calibrated >= LIGHT_SENSOR_BLACK_THRESHOLD) {
            sensorBlack[i] = true;
        } else if (calibrated <= LIGHT_SENSOR_WHITE_THRESHOLD) {
            sensorBlack[i] = false;
        }
    }
}

bool isBlack(int sensorIndex) {
    return readCalibratedLineSensor(sensorIndex) > LIGHT_SENSOR_BLACK_THRESHOLD;
}

bool isWhite(int sensorIndex) {
    return readCalibratedLineSensor(sensorIndex) < LIGHT_SENSOR_WHITE_THRESHOLD;
}

// read line sensor for an index and returns the calibrated version
int readCalibratedLineSensor(int sensorIndex) {
    int raw = analogRead(LINE_SENSOR_PINS[sensorIndex]);
    return applyLightSensorCalibration(raw, sensorIndex);
}

// Simple function to return if a line is detected on any of the active line sensors
bool isLineDetected() {
    updateLineSensors();
    return getBlackCountSensors() > 0;
}

// Get number of black sensors detected on the line sensors
int getBlackCountSensors() {
    int blackCount = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorBlack[i]) {
            blackCount++;
        }
    }

    return blackCount;
}

// Run continiously during movement to interrupt if sensors see a line.
bool handleLineInterrupt() {
    if (isLineDetected()) {
        Serial.print(F("LINE INTERRUPT: black line detected during movement"));
        stopMotors();
        return true;
    }

    return false;
}

void followTheLine() {
    // Keep updating line sensors and gripper, stop if there is no line,
    // also stop (victory condition) if all sensors are black
    while (true) {
        updateLineSensors();
        gripperUpdate();

        if (getBlackCountSensors() == 0) {
            stopMotors();
            return;
        } else if (allSensorsBlack()) {
            Serial.println(F("RACE FINISHED"));
            stopMotors();
            openGripper();

            while (true) {
                gripperUpdate();
                // Victory dance/sequence can go here.
            }
        }

        followTheLineMovement();
    }
}

void updateConvenienceBooleans() {
    // Convenience booleans, writes if the sensor is reading black or not to a boolean value
    leftOuterBlack = sensorBlack[LEFT_OUTER_SENSOR];
    leftMiddleBlack = sensorBlack[LEFT_MIDDLE_SENSOR];
    leftInnerBlack = sensorBlack[LEFT_INNER_SENSOR];
    rightInnerBlack = sensorBlack[RIGHT_INNER_SENSOR];
    rightMiddleBlack = sensorBlack[RIGHT_MIDDLE_SENSOR];
    rightOuterBlack = sensorBlack[RIGHT_OUTER_SENSOR];

    // Convenience booleans for if any of the left or right sensors read as black
    anyLeftBlack = leftOuterBlack || leftMiddleBlack || leftInnerBlack;
    anyRightBlack = rightOuterBlack || rightMiddleBlack || rightInnerBlack;

    // Count how many sensors counted black
    blackCount = getBlackCountSensors();
}

// helper functions for following line logic
void lineFollowingHandleAllBlack(){
    driveForward(
        SPEED_NONE + LEFT_MOTOR_CALIBRATION,
        SPEED_NONE + RIGHT_MOTOR_CALIBRATION
    );
    return;
}

void lineFollowingHandleRightAndNoLeft() {
    // Edge first
    if (rightOuterBlack) {
        driveForward(
            SPEED_EDGE + CORRECTION_EDGE + LEFT_MOTOR_CALIBRATION,
            SPEED_EDGE + RIGHT_MOTOR_CALIBRATION
        );
        return;
    }
    // Then middle
    else if (rightMiddleBlack) {
        driveForward(
            SPEED_MIDDLE + CORRECTION_MIDDLE + LEFT_MOTOR_CALIBRATION,
            SPEED_MIDDLE + RIGHT_MOTOR_CALIBRATION
        );
        return;
    }
    // Then inner
    else {
        driveForward(
            SPEED_INNER + CORRECTION_INNER + LEFT_MOTOR_CALIBRATION,
            SPEED_INNER + RIGHT_MOTOR_CALIBRATION
        );
        return;
    }
}

void LineFollowingHandleLeftAndNoRight() {
    if (leftOuterBlack) {
        driveForward(
            SPEED_EDGE + LEFT_MOTOR_CALIBRATION,
            SPEED_EDGE + CORRECTION_EDGE + RIGHT_MOTOR_CALIBRATION
        );
        return;
    } else if (leftMiddleBlack) {
        driveForward(
            SPEED_MIDDLE + LEFT_MOTOR_CALIBRATION,
            SPEED_MIDDLE + CORRECTION_MIDDLE + RIGHT_MOTOR_CALIBRATION
        );
        return;
    } else {
        driveForward(
            SPEED_INNER + LEFT_MOTOR_CALIBRATION,
            SPEED_INNER + CORRECTION_INNER + RIGHT_MOTOR_CALIBRATION
        );
        return;
    }
}

void LineFollowingHandleRightInnerBlack() {
    driveForward(
        SPEED_INNER + CORRECTION_INNER + LEFT_MOTOR_CALIBRATION,
        SPEED_INNER + RIGHT_MOTOR_CALIBRATION
    );
    return;
}

void LineFollowingHandleLeftInnerBlack() {
    driveForward(
        SPEED_INNER + LEFT_MOTOR_CALIBRATION,
        SPEED_INNER + CORRECTION_INNER + RIGHT_MOTOR_CALIBRATION
    );
    return;
}

// Follow the line logic
void followTheLineMovement() {
    updateConvenienceBooleans();

    // If all sensors are black
    if (blackCount == NUM_SENSORS) {
        lineFollowingHandleAllBlack();
        return;
    }

    // If any right sensor and no left sensors
    if (anyRightBlack && !anyLeftBlack) {
        lineFollowingHandleRightAndNoLeft();
        return;
    }
    // If any left sensors and no right sensors
    else if (!anyRightBlack && anyLeftBlack) {
        LineFollowingHandleLeftAndNoRight();
        return;
    }
    else if (rightInnerBlack) {
        LineFollowingHandleRightInnerBlack();
        return;
    }
    else if (leftInnerBlack) {
        LineFollowingHandleLeftInnerBlack();
        return;
    }

    driveForward(
        SPEED_INNER + LEFT_MOTOR_CALIBRATION,
        SPEED_INNER + RIGHT_MOTOR_CALIBRATION
    );
}

// Motor control for line follower movement
void driveForward(int left, int right) {
    showForwardLights();

    // Constrain because sometimes value + correction goes over or under 0 and 255
    // respectively
    left = constrain(left, MOTORS_PWM_MIN, MOTORS_PWM_MAX);
    right = constrain(right, MOTORS_PWM_MIN, MOTORS_PWM_MAX);

    analogWrite(LEFT_FORWARD_PIN, left);
    analogWrite(LEFT_BACKWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(RIGHT_FORWARD_PIN, right);
    analogWrite(RIGHT_BACKWARD_PIN, MOTORS_PWM_MIN);
}
