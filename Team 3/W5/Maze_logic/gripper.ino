// ~~~~~~~~~~~~~~~~~~~~~~ GRIPPER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
void gripperUpdate() {
    // Send one servo pulse every 20 ms to keep it powered
    unsigned long now = millis();

    // If 20 ms have passed, send another pulse to servo
    if (now - lastServoMs >= SERVO_REFRESH_INTERVAL_MS) {
        lastServoMs = now;
        digitalWrite(SERVO_PIN, HIGH);

        // PulseUs will be either the value to close or open the gripper,
        // delay to keep the pulse HIGH for required amount
        delayMicroseconds(gripperPulseUs);
        digitalWrite(SERVO_PIN, LOW);
    }
}

// Functions to update gripperPulseUs depending on if the gripper should be
// opened or closed, this is used in gripperUpdate()
void openGripper() {
    gripperPulseUs = GRIPPER_OPEN_US;
}

void closeGripper() {
    gripperPulseUs = GRIPPER_CLOSE_US;
}

// Replace ALL delay(ms) with this (keeps servo powered during waits)
void waitMs(unsigned long ms) {
    unsigned long start = millis();

    // While millis time - start time is less than desired wait time,
    // do nothing and keep updating the gripper
    while (millis() - start < ms) {
        gripperUpdate();
    }
}
