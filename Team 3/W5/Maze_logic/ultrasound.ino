// ~~~~~~~~~~~~~~~~~~~~~~ ULTRASOUND SENSORS FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
float getDistance(int trigPin, int echoPin) {
    // send a high pulse and set trigger back to low after the pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(ULTRASOUND_TRIGGER_SETTLE_US);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(ULTRASOUND_TRIGGER_PULSE_US);

    digitalWrite(trigPin, LOW);

    // If signal takes too long, return -1
    unsigned long duration = pulseIn(echoPin, HIGH, ULTRASOUND_PULSE_TIMEOUT_US);

    if (duration == 0) {
        return NO_DISTANCE_READING;
    }

    // If valid reading, return the distance in centimeters
    float distance = duration * ULTRASOUND_US_TO_CM;

    return distance;
}

// Gets the average of the getDistance from the sensor "amount" amount of times
float getAverageDistance(int trigPin, int echoPin, int amount) {
    if (amount <= 0) {
        return NO_DISTANCE_READING;
    }

    float sum = 0.0f;
    int validCount = 0;
    int attempts = 0;
    int maxAttempts = amount * ULTRASOUND_MAX_ATTEMPTS_MULTIPLIER;

    while (validCount < amount && attempts < maxAttempts) {
        attempts++;

        float distance = getDistance(trigPin, echoPin);

        // used to be < ULTRASOUND_MAX_VALID_DISTANCE, but this can cause weird behaviour and make the robot think 
        // sensors are failing when in reality they're seeing the open space at the end of the maze, so now the max value is a valid reading
        if (distance > 0 && distance <= ULTRASOUND_MAX_VALID_DISTANCE_CM) {
            sum += distance;
            validCount++;
        }

        waitMs(ULTRASOUND_SENSOR_MEASUREMENT_DELAY_TIME_MS);
    }

    if (validCount < amount) {
        return NO_DISTANCE_READING;
    }

    // Wait at the end of the loop because sometimes this function gets called
    // many times in a row
    waitMs(ULTRASOUND_SENSOR_MEASUREMENT_DELAY_TIME_MS);

    return sum / validCount;
}

float getAverageDistanceWithRetry(int trigPin, int echoPin) {
    int failedTimesInARow = 0;

    while (failedTimesInARow < ULTRASOUND_SENSOR_MAX_FAILS_IN_A_ROW) {
        float distance = getAverageDistance(
            trigPin,
            echoPin,
            NUM_SAMPLES_ULTRASOUND
        );

        if (distance != NO_DISTANCE_READING) {
            return distance;
        }

        failedTimesInARow++;
    }

    return NO_DISTANCE_READING;
}

float getAverageDistanceFront() {
    float distance = getAverageDistanceWithRetry(TRIG_FRONT, ECHO_FRONT);

    if (distance == NO_DISTANCE_READING) {
      // used to be return NO_DISTANCE_READING for all of these getAverageDistance functions, but now, 
      // if there's no distance reading, assume it's the max distance
        return ULTRASOUND_MAX_VALID_DISTANCE_CM;
    }

    return constrain(
        distance,
        FRONT_DISTANCE_MIN_VALID_CM,
        ULTRASOUND_MAX_VALID_DISTANCE_CM
    );
}

float getAverageDistanceLeft() {
    float distance = getAverageDistanceWithRetry(TRIG_LEFT, ECHO_LEFT);

    if (distance == NO_DISTANCE_READING) {
        return ULTRASOUND_MAX_VALID_DISTANCE_CM;
    }

    return constrain(
        distance,
        SIDE_DISTANCE_MIN_VALID_CM,
        ULTRASOUND_MAX_VALID_DISTANCE_CM
    );
}

float getAverageDistanceRight() {
    float distance = getAverageDistanceWithRetry(TRIG_RIGHT, ECHO_RIGHT);

    if (distance == NO_DISTANCE_READING) {
        return ULTRASOUND_MAX_VALID_DISTANCE_CM;
    }

    return constrain(
        distance,
        SIDE_DISTANCE_MIN_VALID_CM,
        ULTRASOUND_MAX_VALID_DISTANCE_CM
    );
}

bool getStartCondition() {
    float distance = getAverageDistanceFront();
    Serial.println(F("getStartCondition sensor reading: "));
    Serial.print(distance);

    return (
        distance < START_CONDITION_MAX_DISTANCE &&
        distance > START_CONDITION_MIN_DISTANCE
    );
}

bool isTooCloseToFrontWallForTurnLeftOrRight() {
    float frontDistance = getAverageDistanceFront();

    return (
        frontDistance <= MAX_DISTANCE_FRONT_LIMIT_TO_TURN_LEFT_OR_RIGHT &&
        frontDistance >= MIN_DISTANCE_FRONT_LIMIT_TO_TURN_LEFT_OR_RIGHT
    );
}

bool cannotBeCloserToWallForTurnLeftOrRight() {
    float frontDistance = getAverageDistanceFront();

    return frontDistance <= MIN_DISTANCE_FRONT_LIMIT_TO_TURN_LEFT_OR_RIGHT;
}


