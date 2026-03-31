// ~~~~~~~~~~~~~~~~~~~~~~ MOTORS FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
// Round a float to the nearest int by adding or removing 0.5 from the float
// because int to float does not round normally
int roundFloatToInt(float value) {
    if (value >= 0) {
        return (int)(value + 0.5);
    } else {
        return (int)(value - 0.5);
    }
}

// This function runs automatically every time the left encoder signal changes
void leftEncoderInterrupt() {
    unsigned long currentTime = micros();

    if (currentTime - lastLeftEdgeTime >= MINIMUM_EDGE_TIME_US) {
        leftTickCount = leftTickCount + 1;
        lastLeftEdgeTime = currentTime;
    }
}

// This function runs automatically every time the right encoder signal changes
void rightEncoderInterrupt() {
    unsigned long currentTime = micros();

    if (currentTime - lastRightEdgeTime >= MINIMUM_EDGE_TIME_US) {
        rightTickCount = rightTickCount + 1;
        lastRightEdgeTime = currentTime;
    }
}

// Reset both encoder counters to zero, turns interrupts off for writing to
// values then back on when function ends
void resetEncoderCounts() {
    noInterrupts();
    leftTickCount = 0;
    rightTickCount = 0;
    lastLeftEdgeTime = 0;
    lastRightEdgeTime = 0;
    interrupts();
}

// Get current left encoder count, put it into value, and return value,
// because you cannot resume interrupts after a return
// noInterrupts because writing while they are enabled might mess up the value
unsigned long getLeftTicks() {
    unsigned long value;

    noInterrupts();
    value = leftTickCount;
    interrupts();

    return value;
}

// Get current right encoder count
unsigned long getRightTicks() {
    unsigned long value;

    noInterrupts();
    value = rightTickCount;
    interrupts();

    return value;
}

// Stop all motors
void stopMotors() {
    clearLights();
    analogWrite(LEFT_FORWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(LEFT_BACKWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(RIGHT_FORWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(RIGHT_BACKWARD_PIN, MOTORS_PWM_MIN);
}

// Start moving forward
void startForwardMotion() {
    showForwardLights();
    analogWrite(LEFT_BACKWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(RIGHT_BACKWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(LEFT_FORWARD_PIN, LEFT_FORWARD_SPEED);
    analogWrite(RIGHT_FORWARD_PIN, RIGHT_FORWARD_SPEED);
}

// Start moving backward
void startBackwardMotion() {
    showBackwardLights();
    analogWrite(LEFT_FORWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(RIGHT_FORWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(LEFT_BACKWARD_PIN, LEFT_BACKWARD_SPEED);
    analogWrite(RIGHT_BACKWARD_PIN, RIGHT_BACKWARD_SPEED);
}

// Start turning left
void startLeftForwardTurn() {
    showLeftLights();
    analogWrite(LEFT_FORWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(LEFT_BACKWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(RIGHT_BACKWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(RIGHT_FORWARD_PIN, RIGHT_TURN_SPEED);
}

// Start turning left backwards
void startLeftBackwardTurn() {
    showLeftLights();
    analogWrite(LEFT_FORWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(LEFT_BACKWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(RIGHT_FORWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(RIGHT_BACKWARD_PIN, RIGHT_TURN_SPEED);
}

// Start turning right
void startRightForwardTurn() {
    showRightLights();
    analogWrite(RIGHT_FORWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(RIGHT_BACKWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(LEFT_BACKWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(LEFT_FORWARD_PIN, LEFT_TURN_SPEED);
}

// Start turning right backwards
void startRightBackwardTurn() {
    showRightLights();
    analogWrite(RIGHT_FORWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(RIGHT_BACKWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(LEFT_FORWARD_PIN, MOTORS_PWM_MIN);
    analogWrite(LEFT_BACKWARD_PIN, LEFT_TURN_SPEED);
}

// direction = 1 = forward, else = backwards
void correctMotorSpeeds(int leftBaseSpeed, int rightBaseSpeed, int direction) {
    // See how much the wheel rotations have differed
    int difference = getRightTicks() - getLeftTicks();

    // If left is going faster, this returns a negative number

    // Multiplied difference (difference between ticks * X) is the difference
    // that will be written to PWM
    difference = difference * MOTOR_ENCODER_CORRECTION_GAIN;

    // Slow down right and speed up left (because right is the stronger motor)
    int leftSpeed = leftBaseSpeed + difference;
    int rightSpeed = rightBaseSpeed - difference;

    // Constrain corrected speed between 0 and 255
    rightSpeed = constrain(rightSpeed, MOTORS_PWM_MIN, MOTORS_PWM_MAX);
    leftSpeed = constrain(leftSpeed, MOTORS_PWM_MIN, MOTORS_PWM_MAX);

    // Write corrected speeds to motors
    if (direction == DIRECTION_FORWARD) {
        analogWrite(LEFT_BACKWARD_PIN, MOTORS_PWM_MIN);
        analogWrite(RIGHT_BACKWARD_PIN, MOTORS_PWM_MIN);

        analogWrite(LEFT_FORWARD_PIN, leftSpeed);
        analogWrite(RIGHT_FORWARD_PIN, rightSpeed);
    } else {
        analogWrite(LEFT_FORWARD_PIN, MOTORS_PWM_MIN);
        analogWrite(RIGHT_FORWARD_PIN, MOTORS_PWM_MIN);

        analogWrite(LEFT_BACKWARD_PIN, leftSpeed);
        analogWrite(RIGHT_BACKWARD_PIN, rightSpeed);
    }
}

// Move forward a given distance in centimeters
void moveForwardCm(float distanceCm) {
    unsigned long targetTicks = roundFloatToInt(distanceCm * TICKS_PER_CM);

    if (targetTicks <= 0) {
        return;
    }

    resetEncoderCounts();
    startForwardMotion();

    unsigned long lastTicks = 0;
    unsigned long lastStallTime = millis();

    while (true) {
        gripperUpdate();

        if (lineSensorsCalibrated && handleLineInterrupt()) {
            followTheLine();
            return;
        }

        unsigned long leftValue = getLeftTicks();
        unsigned long rightValue = getRightTicks();
        unsigned long averageTicks = (leftValue + rightValue) / 2;
        correctMotorSpeeds(
            LEFT_FORWARD_SPEED,
            RIGHT_FORWARD_SPEED,
            DIRECTION_FORWARD
        );

        if (averageTicks >= targetTicks) {
            break;
        }

        // If robot moved forward, update last ticks and last stall time
        // (because it has not stalled)
        if (averageTicks != lastTicks) {
            lastTicks = averageTicks;
            lastStallTime = millis();
        }

        // If robot has not been moving for too long, do recovery,
        // then break movement
        if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
            stopMotors();
            moveBackwardCm(STALL_RECOVERY_BACKWARD_CM);
            break;
        }
    }

    stopMotors();
}

// Move backward a given distance in centimeters
void moveBackwardCm(float distanceCm) {
    unsigned long targetTicks = roundFloatToInt(distanceCm * TICKS_PER_CM);

    if (targetTicks <= 0) {
        return;
    }

    unsigned long lastTicks = 0;
    unsigned long lastStallTime = millis();

    resetEncoderCounts();
    startBackwardMotion();

    while (true) {
        gripperUpdate();

        if (lineSensorsCalibrated && handleLineInterrupt()) {
            followTheLine();
            return;
        }

        unsigned long leftValue = getLeftTicks();
        unsigned long rightValue = getRightTicks();
        unsigned long averageTicks = (leftValue + rightValue) / 2;
        correctMotorSpeeds(
            LEFT_FORWARD_SPEED,
            RIGHT_FORWARD_SPEED,
            DIRECTION_BACKWARD
        );

        if (averageTicks >= targetTicks) {
            break;
        }

        if (averageTicks != lastTicks) {
            lastTicks = averageTicks;
            lastStallTime = millis();
        }

        if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
            stopMotors();
            moveForwardCm(STALL_RECOVERY_FORWARD_CM);
            break;
        }
    }

    stopMotors();
}

void turnLeftForwardTicksWithDeadEnd(unsigned long targetTicks, boolean isDeadEnd) {
    Serial.print(F("Running turnLeftForward..., targetTicks: "));
    Serial.println(targetTicks);
    Serial.print(F("isDeadEnd: "));
    Serial.println(isDeadEnd);

    if (targetTicks <= 0) {
        return;
    }

    unsigned long lastTicks = 0;
    unsigned long lastStallTime = millis();

    resetEncoderCounts();
    startLeftForwardTurn();

    while (true) {
        gripperUpdate();

        if (lineSensorsCalibrated && handleLineInterrupt()) {
            followTheLine();
            return;
        }

        unsigned long rightValue = getRightTicks();

        // During a left turn, the right wheel is the moving wheel
        if (rightValue >= targetTicks) {
            break;
        }

        if (rightValue != lastTicks) {
            lastTicks = rightValue;
            lastStallTime = millis();
        }

        if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
            if (isDeadEnd) {
                unsigned long remainingTicks = targetTicks - rightValue;
                moveBackwardCm(MOVE_RECOVERY_IS_DEAD_END_CM);
                turnLeftForwardTicksWithDeadEnd(remainingTicks, true);
                break;
            } else {
                stopMotors();
                moveBackwardCm(TURN_STALL_RECOVERY_BACKUP_CM);
                turnRightBackwardTicksWithDeadEnd(
                    TURN_STALL_RECOVERY_TICKS,
                    false
                );
                break;
            }
        }
    }

    stopMotors();
}

void turnRightForwardTicksWithDeadEnd(unsigned long targetTicks, boolean isDeadEnd) {
    Serial.print(F("Running turnRightForward..., targetTicks: "));
    Serial.println(targetTicks);
    Serial.print(F("isDeadEnd: "));
    Serial.println(isDeadEnd);

    if (targetTicks <= 0) {
        return;
    }

    unsigned long lastTicks = 0;
    unsigned long lastStallTime = millis();

    resetEncoderCounts();
    startRightForwardTurn();

    while (true) {
        gripperUpdate();

        if (lineSensorsCalibrated && handleLineInterrupt()) {
            followTheLine();
            return;
        }

        unsigned long leftValue = getLeftTicks();

        // During a right turn, the left wheel is the moving wheel
        if (leftValue >= targetTicks) {
            break;
        }

        if (leftValue != lastTicks) {
            lastTicks = leftValue;
            lastStallTime = millis();
        }

        if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
            if (isDeadEnd) {
                unsigned long remainingTicks = targetTicks - leftValue;
                moveBackwardCm(MOVE_RECOVERY_IS_DEAD_END_CM);
                turnRightForwardTicksWithDeadEnd(remainingTicks, true);
                break;
            } else {
                stopMotors();
                moveBackwardCm(TURN_STALL_RECOVERY_BACKUP_CM);
                turnLeftBackwardTicksWithDeadEnd(TURN_STALL_RECOVERY_TICKS, false);
                break;
            }
        }
    }

    stopMotors();
}

void turnLeftBackwardTicksWithDeadEnd(unsigned long targetTicks, boolean isDeadEnd) {
    Serial.print(F("Running turnLeftBackward..., targetTicks: "));
    Serial.println(targetTicks);
    Serial.print(F("isDeadEnd: "));
    Serial.println(isDeadEnd);

    if (targetTicks <= 0) {
        return;
    }

    unsigned long lastTicks = 0;
    unsigned long remainingTicks = 0;
    unsigned long lastStallTime = millis();

    resetEncoderCounts();
    startRightBackwardTurn();

    while (true) {
        gripperUpdate();

        if (lineSensorsCalibrated && handleLineInterrupt()) {
            followTheLine();
            return;
        }

        unsigned long leftValue = getLeftTicks();

        // During a right turn, the left wheel is the moving wheel
        if (leftValue >= targetTicks) {
            break;
        }

        if (leftValue != lastTicks) {
            lastTicks = leftValue;
            lastStallTime = millis();
        }

        if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
            if (isDeadEnd) {
                remainingTicks = targetTicks - leftValue;
                moveForwardCm(MOVE_RECOVERY_IS_DEAD_END_CM);
                turnLeftBackwardTicksWithDeadEnd(remainingTicks, true);
                break;
            } else {
                stopMotors();
                moveForwardCm(TURN_STALL_RECOVERY_FORWARD_CM);
                turnRightForwardTicksWithDeadEnd(TURN_STALL_RECOVERY_TICKS, false);
                break;
            }
        }
    }

    stopMotors();
}

void turnRightBackwardTicksWithDeadEnd(unsigned long targetTicks, boolean isDeadEnd) {
    Serial.print(F("Running turnRightBackward..., targetTicks: "));
    Serial.println(targetTicks);
    Serial.print(F("isDeadEnd: "));
    Serial.println(isDeadEnd);

    if (targetTicks <= 0) {
        return;
    }

    unsigned long lastTicks = 0;
    unsigned long lastStallTime = millis();

    resetEncoderCounts();
    startLeftBackwardTurn();

    while (true) {
        gripperUpdate();

        if (lineSensorsCalibrated && handleLineInterrupt()) {
            followTheLine();
            return;
        }

        unsigned long rightValue = getRightTicks();

        // During a left turn, the right wheel is the moving wheel
        if (rightValue >= targetTicks) {
            break;
        }

        if (rightValue != lastTicks) {
            lastTicks = rightValue;
            lastStallTime = millis();
        }

        if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
            if (isDeadEnd) {
                unsigned long remainingTicks = targetTicks - rightValue;
                moveForwardCm(MOVE_RECOVERY_IS_DEAD_END_CM);
                turnRightBackwardTicksWithDeadEnd(remainingTicks, true);
                break;
            } else {
                stopMotors();
                moveForwardCm(TURN_STALL_RECOVERY_FORWARD_CM);
                turnRightForwardTicksWithDeadEnd(TURN_STALL_RECOVERY_TICKS, false);
                break;
            }
        }
    }

    stopMotors();
}