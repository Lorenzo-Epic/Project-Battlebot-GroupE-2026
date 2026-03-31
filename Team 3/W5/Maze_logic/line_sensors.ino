// ~~~~~~~~~~~~~~~~~~~~~~ LINE SENSORS / FOLLOWING FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
void updateLineSensors() {
    // Read line sensor values
    for (int i = 0; i < NUM_SENSORS; i++) {
        int raw = analogRead(LINE_SENSOR_PINS[i]);
        lineValues[i] = applyLightSensorCalibration(raw, i);

        // Hysteresis logic, only update the sensor value as black
        // if it goes past hysteresis
        if (lineValues[i] >= LIGHT_SENSOR_BLACK_THRESHOLD) {
            sensorBlack[i] = true;
        } else if (lineValues[i] <= LIGHT_SENSOR_WHITE_THRESHOLD) {
            sensorBlack[i] = false;
        }
    }
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
                // PLACEHOLDER FOR VICTORY DANCE OR SOMETHING
            }
        }

        followTheLineMovement();
    }
}

void followTheLineMovement() {
    // Convenience booleans, writes if the sensor is reading black or not
    // (accounting for hysteresis too) to a boolean value
    bool leftOuterBlack = sensorBlack[LEFT_OUTER_SENSOR];
    bool leftMiddleBlack = sensorBlack[LEFT_MIDDLE_SENSOR];
    bool leftInnerBlack = sensorBlack[LEFT_INNER_SENSOR];
    bool rightInnerBlack = sensorBlack[RIGHT_INNER_SENSOR];
    bool rightMiddleBlack = sensorBlack[RIGHT_MIDDLE_SENSOR];
    bool rightOuterBlack = sensorBlack[RIGHT_OUTER_SENSOR];

    // Convenience booleans for if any of the left or right sensors read as black
    bool anyLeftBlack = leftOuterBlack || leftMiddleBlack || leftInnerBlack;
    bool anyRightBlack = rightOuterBlack || rightMiddleBlack || rightInnerBlack;

    // Count how many sensors counted black
    int blackCount = getBlackCountSensors();

    // If all sensors are black
    if (blackCount == NUM_SENSORS) {
        driveForward(
            SPEED_NONE + LEFT_MOTOR_CALIBRATION,
            SPEED_NONE + RIGHT_MOTOR_CALIBRATION
        );
        return;
    }

    // If any right sensor and no left sensors
    if (anyRightBlack && !anyLeftBlack) {
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
    // If any left sensors and no right sensors
    else if (!anyRightBlack && anyLeftBlack) {
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
    } else if (rightInnerBlack) {
        driveForward(
            SPEED_INNER + CORRECTION_INNER + LEFT_MOTOR_CALIBRATION,
            SPEED_INNER + RIGHT_MOTOR_CALIBRATION
        );
        return;
    } else if (leftInnerBlack) {
        driveForward(
            SPEED_INNER + LEFT_MOTOR_CALIBRATION,
            SPEED_INNER + CORRECTION_INNER + RIGHT_MOTOR_CALIBRATION
        );
        return;
    }

    driveForward(
        SPEED_INNER + LEFT_MOTOR_CALIBRATION,
        SPEED_INNER + RIGHT_MOTOR_CALIBRATION
    );
}

// Motor control
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

void driveBackward(int left, int right) {
    showBackwardLights();
    left = constrain(left, MOTORS_PWM_MIN, MOTORS_PWM_MAX);
    right = constrain(right, MOTORS_PWM_MIN, MOTORS_PWM_MAX);

    analogWrite(LEFT_FORWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(LEFT_BACKWARD_PIN, left);
    analogWrite(RIGHT_FORWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(RIGHT_BACKWARD_PIN, right);
}
