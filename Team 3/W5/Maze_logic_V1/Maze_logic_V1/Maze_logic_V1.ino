// Motor pins
const int LEFT_FORWARD_PIN  = 6;
const int LEFT_BACKWARD_PIN = 10;
const int RIGHT_FORWARD_PIN = 11;
const int RIGHT_BACKWARD_PIN = 9;

// Rotation pins
const int ROTATION_LEFT_PIN  = 2;
const int ROTATION_RIGHT_PIN = 3;

// Ultrasonic pins
const int TRIG_FRONT = 12;
const int ECHO_FRONT = 13;

const int TRIG_LEFT  = 7;
const int ECHO_LEFT  = 4;

const int TRIG_RIGHT = A3;
const int ECHO_RIGHT = 8;

// Light sensors
const int NUM_SENSORS = 4;

const int LIGHT_SENSOR_PINS[NUM_SENSORS] = {A0, A1, A2, A4};

const int BLACK_THRESHOLD = 600;

// Gripper
const int SERVO_PIN = 5;

const int GRIPPER_OPEN_US  = 1820;
const int GRIPPER_CLOSE_US = 1000;

volatile int gripperPulseUs = GRIPPER_CLOSE_US;

unsigned long lastServoMs = 0;

// Maze settings
const int WALL_DISTANCE = 20;

// Encoder variables

volatile unsigned long leftTickCount  = 0;
volatile unsigned long rightTickCount = 0;

// Wheel constants

const float WHEEL_DIAMETER_CM      = 6.5;
const float WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;

const float TICKS_PER_REVOLUTION = 40.0;
const float TICKS_PER_CM         = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_CM;


// Ultrasound constants
// speed of sound / 2

const float SOUND_HALF = 0.017;

// Setup

void setup()
{
    Serial.begin(115200);

    pinMode(LEFT_FORWARD_PIN, OUTPUT);
    pinMode(LEFT_BACKWARD_PIN, OUTPUT);
    pinMode(RIGHT_FORWARD_PIN, OUTPUT);
    pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

    pinMode(TRIG_FRONT, OUTPUT);
    pinMode(ECHO_FRONT, INPUT);

    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);

    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        pinMode(LIGHT_SENSOR_PINS[i], INPUT);
    }

    pinMode(SERVO_PIN, OUTPUT);

    pinMode(ROTATION_LEFT_PIN, INPUT_PULLUP);
    pinMode(ROTATION_RIGHT_PIN, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(ROTATION_LEFT_PIN),
        leftEncoderInterrupt,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(ROTATION_RIGHT_PIN),
        rightEncoderInterrupt,
        CHANGE
    );

    stopMotors();
}

// Main loop

void loop() {
  
    runCalibration();

    followBlackLineUntilMaze();

    solveMaze();

    followBlackLineUntilEnd();

    openGripper();

    stopMotors();

    while (true) {
        // Stop forever
    }
}

// Line following

void followBlackLineUntilMaze() {
    while (true) {
        int left  = analogRead(LIGHT_SENSOR_PINS[0]);
        int midL  = analogRead(LIGHT_SENSOR_PINS[1]);
        int midR  = analogRead(LIGHT_SENSOR_PINS[2]);
        int right = analogRead(LIGHT_SENSOR_PINS[3]);

        float front = getDistance(TRIG_FRONT, ECHO_FRONT);

        if (front > 30) {
            stopMotors();
            return;
        }

        if (midL > BLACK_THRESHOLD || midR > BLACK_THRESHOLD) {
            moveForwardCm(4);
        }
        else if (left > BLACK_THRESHOLD) {
            turnLeftSmall();
        }
        else if (right > BLACK_THRESHOLD) {
            turnRightSmall();
        }
        else {
            moveForwardCm(2);
        }
    }
}


void followBlackLineUntilEnd() {
    while (true) {
        if (allSensorsBlack()) {
            stopMotors();
            return;
        }

        int left  = analogRead(LIGHT_SENSOR_PINS[0]);
        int midL  = analogRead(LIGHT_SENSOR_PINS[1]);
        int midR  = analogRead(LIGHT_SENSOR_PINS[2]);
        int right = analogRead(LIGHT_SENSOR_PINS[3]);

        if (midL > BLACK_THRESHOLD || midR > BLACK_THRESHOLD) {
            moveForwardCm(4);
        }
        else if (left > BLACK_THRESHOLD) {
            turnLeftSmall();
        }
        else if (right > BLACK_THRESHOLD) {
            turnRightSmall();
        }
        else {
            moveForwardCm(2);
        }
    }
}

// MAZE SOLVER

void solveMaze() {
    while (true) {
        if (checkBlackLine()) {
            stopMotors();
            return;
        }

        float front = getDistance(TRIG_FRONT, ECHO_FRONT);
        float left  = getDistance(TRIG_LEFT, ECHO_LEFT);
        float right = getDistance(TRIG_RIGHT, ECHO_RIGHT);

        if (left > WALL_DISTANCE) {
            turnLeft90();
            moveForwardCm(25);
        }
        else if (front > WALL_DISTANCE) {
            moveForwardCm(25);
        }
        else if (right > WALL_DISTANCE) {
            turnRight90();
            moveForwardCm(25);
        }
        else {
            turnRight90();
            turnRight90();
        }
    }
}

// Ultrasonic

float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);

    return duration * SOUND_HALF;
}

// Movement

void moveForwardCm(float cm) {
    int target = cm * TICKS_PER_CM;

    resetEncoders();

    analogWrite(LEFT_FORWARD_PIN, 200);
    analogWrite(RIGHT_FORWARD_PIN, 200);

    while ((leftTickCount + rightTickCount) / 2 < target) {
        gripperUpdate();
    }

    stopMotors();
}


void turnLeft90() {
    analogWrite(RIGHT_FORWARD_PIN, 200);
    delay(330);
    stopMotors();
}

void turnRight90() {
    analogWrite(LEFT_FORWARD_PIN, 200);
    delay(330);
    stopMotors();
}

void turnLeftSmall() {
    analogWrite(RIGHT_FORWARD_PIN, 200);
    delay(60);
    stopMotors();
}

void turnRightSmall() {
    analogWrite(LEFT_FORWARD_PIN, 200);
    delay(60);
    stopMotors();
}

void stopMotors() {
    analogWrite(LEFT_FORWARD_PIN, 0);
    analogWrite(LEFT_BACKWARD_PIN, 0);
    analogWrite(RIGHT_FORWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, 0);
}

// Encoders

void leftEncoderInterrupt() {
    leftTickCount++;
}

void rightEncoderInterrupt() {
    rightTickCount++;
}

void resetEncoders() {
    leftTickCount  = 0;
    rightTickCount = 0;
}

// Light sensor helpers

bool allSensorsBlack() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (analogRead(LIGHT_SENSOR_PINS[i]) < BLACK_THRESHOLD) {
            return false;
        }
    }
    return true;
}


bool checkBlackLine() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (analogRead(LIGHT_SENSOR_PINS[i]) > BLACK_THRESHOLD) {
            return true;
        }
    }
    return false;
}

// Calibration

void runCalibration() {
    delay(2000);
}

// Gripper

void gripperUpdate() {
    unsigned long now = millis();
    if (now - lastServoMs >= 20) {
        lastServoMs = now;

        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(gripperPulseUs);
        digitalWrite(SERVO_PIN, LOW);
    }
}

void openGripper() {
    gripperPulseUs = GRIPPER_OPEN_US;
}
