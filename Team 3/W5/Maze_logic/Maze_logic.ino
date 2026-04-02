#include <Adafruit_NeoPixel.h>

// ~~~~~~~~~~~~~~~~~~~~~~ MOVEMENT / LOGIC CONSTANTS ~~~~~~~~~~~~~~~~~~~~~~
#define INITIAL_BACKUP_AFTER_GRAB_CM 3
#define INITIAL_MOVE_INTO_MAZE_CM 5

#define LEFT_OPEN_PRE_TURN_BACKUP_CM 1 //was 3
#define LEFT_OPEN_PRE_TURN_FORWARD_CM 3
#define LEFT_OPEN_POST_TURN_FORWARD_CM 10

#define RIGHT_OPEN_PRE_TURN_BACKUP_CM 1 //was 3
#define RIGHT_OPEN_PRE_TURN_FORWARD_CM 3
#define RIGHT_OPEN_POST_TURN_FORWARD_CM 10

#define FORWARD_OPEN_DEFAULT_MOVE_CM 10
#define FORWARD_OPEN_SHORT_MOVE_CM 5 
#define FORWARD_OPEN_SIDE_CLEARANCE_CM 3

#define STALL_RECOVERY_BACKWARD_CM 5
#define STALL_RECOVERY_FORWARD_CM 5

#define TURN_STALL_RECOVERY_BACKUP_CM 3
#define TURN_STALL_RECOVERY_FORWARD_CM 3
#define TURN_STALL_RECOVERY_TICKS 5

#define LEFT_WALL_ADJUST_SMALL_TICKS 1 //was 2
#define LEFT_WALL_ADJUST_MEDIUM_TICKS 2 //was 3
#define LEFT_WALL_ADJUST_LARGE_TICKS 4 //was 5
#define FOLLOW_LEFT_WALL_IDEAL_TOLERANCE 1

#define MOTOR_ENCODER_CORRECTION_GAIN 35

#define DIRECTION_BACKWARD 0
#define DIRECTION_FORWARD 1

// ~~~~~~~~~~~~~~~~~~~~~~ MOTORS VALUES ~~~~~~~~~~~~~~~~~~~~~~
#define MOTORS_PWM_MIN 0
#define MOTORS_PWM_MAX 255

#define LEFT_FORWARD_PIN 6
#define LEFT_BACKWARD_PIN 10
#define RIGHT_FORWARD_PIN 11
#define RIGHT_BACKWARD_PIN 9

#define ROTATION_LEFT_PIN 2
#define ROTATION_RIGHT_PIN 3

// Motor speeds for straight driving
#define LEFT_FORWARD_SPEED 230
#define LEFT_BACKWARD_SPEED 230
#define RIGHT_FORWARD_SPEED 223
#define RIGHT_BACKWARD_SPEED 223

// Motor speeds for turning
#define LEFT_TURN_SPEED 230
#define RIGHT_TURN_SPEED 220

// Wheel and encoder values
const float WHEEL_DIAMETER_CM = 6.5f;
const float WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const float TICKS_PER_REVOLUTION = 40.0f;
const float TICKS_PER_CM = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_CM;

// Ignore encoder pulses that happen too quickly
const unsigned long MINIMUM_EDGE_TIME_US = 150;

// 90 degree turn calibration
#define TURN_90_TARGET_TICKS 35

// Timeout for recovery
const unsigned long STALL_TIMEOUT_MS = 1000;

// Encoder counters
volatile unsigned long leftTickCount = 0;
volatile unsigned long rightTickCount = 0;

// Time of previous encoder pulse
volatile unsigned long lastLeftEdgeTime = 0;
volatile unsigned long lastRightEdgeTime = 0;

// Constants for recovery
#define MOVE_RECOVERY_IS_DEAD_END_CM 2

// ~~~~~~~~~~~~~~~~~~~~~~ ULTRASOUND SENSORS VALUES ~~~~~~~~~~~~~~~~~~~~~~
// front
#define TRIG_FRONT 12
#define ECHO_FRONT 13

// left
#define TRIG_LEFT 7
#define ECHO_LEFT 4

// right
#define TRIG_RIGHT A3
#define ECHO_RIGHT 8

// constrain values
#define LIGHT_SENSORS_MIN_READING 0
#define LIGHT_SENSORS_MAX_READING 1023

// ultrasound timing / conversion
const unsigned long ULTRASOUND_TRIGGER_SETTLE_US = 2;
const unsigned long ULTRASOUND_TRIGGER_PULSE_US = 10;

// pulse timeout is if signal takes too long
const unsigned long ULTRASOUND_PULSE_TIMEOUT_US = 30000UL;

// this is speed of sound by microseconds so: distance (cm) = duration (µs) * 0.017
// 0.017 is speed of sound already divided by 2, so instead of 0.034 it is 0.017
const float ULTRASOUND_US_TO_CM = 0.017f;

// value to return if ultrasound measurement has errors
const float NO_DISTANCE_READING = -1.0f;

// ultrasound reading limits
#define ULTRASOUND_MAX_VALID_DISTANCE_CM 1000
#define FRONT_DISTANCE_MIN_VALID_CM 4
#define SIDE_DISTANCE_MIN_VALID_CM 0
#define ULTRASOUND_MAX_ATTEMPTS_MULTIPLIER 3

// distance to wall
// values for deciding if to go left, straight or right
#define WALL_DISTANCE_SIDE 18
#define WALL_DISTANCE_FRONT 15

// values for sticking close to the left wall in cm
#define FOLLOW_LEFT_WALL_VERY_VERY_CLOSE 1
#define FOLLOW_LEFT_WALL_VERY_CLOSE 3
#define FOLLOW_LEFT_WALL_CLOSE 5
#define FOLLOW_LEFT_WALL_IDEAL 7
#define FOLLOW_LEFT_WALL_FAR 9
#define FOLLOW_LEFT_WALL_VERY_FAR 12
#define FOLLOW_LEFT_WALL_VERY_VERY_FAR 15

// distance to front wall for deciding if to move forward before turning or not
#define MAX_DISTANCE_FRONT_LIMIT_TO_TURN_LEFT_OR_RIGHT 14
#define MIN_DISTANCE_FRONT_LIMIT_TO_TURN_LEFT_OR_RIGHT 10
// safety margin for calculating if robot should move full distance or not 
#define DISTANCE_SAFETY_MARGIN_CM 1

// amount of samples of ultrasound sensors to average out
#define NUM_SAMPLES_ULTRASOUND 3

// distance that token has to be from ultrasound sensor to start the robot
#define START_CONDITION_MIN_DISTANCE 20
#define START_CONDITION_MAX_DISTANCE 25

// ultrasound sensors wait a certain ms between measurements to avoid bad readings
#define ULTRASOUND_SENSOR_MEASUREMENT_DELAY_TIME_MS 30

// Time to wait in ms after robot first sees the cone, to allow the robot
// that drops the cone to move out of the way
#define MS_WAIT_AFTER_SEEING_CONE 6000

// recovery constants for ultrasound average reading functions
#define ULTRASOUND_SENSOR_MAX_FAILS_IN_A_ROW 3

// ~~~~~~~~~~~~~~~~~~~~~~ LINE SENSORS ~~~~~~~~~~~~~~~~~~~~~~
// D1 = A0, D2 = A1, D3 = A2, D4 = A3, D5 = A4, D6 = A5, D7 = A6, D8 = A7
#define NUM_SENSORS 6
const int LINE_SENSOR_PINS[NUM_SENSORS] = {A0, A1, A2, A5, A6, A7};
// int lineValues[NUM_SENSORS];  // this keeps the sensor readings of the moment

// pin A3 is used for trig sensor right ultrasound sensor
// sensor calibration initial weightings (gets calibrated properly later)
int weights[NUM_SENSORS] = {-273, -264, -253, -276, -309, -322};

// thresholds
const int LIGHT_SENSOR_WHITE_THRESHOLD = 400;
const int LIGHT_SENSOR_BLACK_THRESHOLD = 600;

// for hysteresis, remembers what the sensor's last value was
bool sensorBlack[NUM_SENSORS] = {false};

// sensors declared here for flexibility from array
const int RIGHT_OUTER_SENSOR = 0;
const int RIGHT_MIDDLE_SENSOR = 1;
const int RIGHT_INNER_SENSOR = 2;
const int LEFT_OUTER_SENSOR = 3;
const int LEFT_MIDDLE_SENSOR = 4;
const int LEFT_INNER_SENSOR = 5;

// speed settings for line following
const int SPEED_NONE = 230;    // speed for all sensors detecting white
const int SPEED_INNER = 200;   // speed for when one of the inner sensors is on black
const int SPEED_MIDDLE = 180;  // speed for when one of the middle sensors is on black
const int SPEED_EDGE = 0;      // speed for when outer sensors are on black

// correction PWM to apply to motor when line following based on where the lines are
const int CORRECTION_EDGE = 180;
const int CORRECTION_MIDDLE = 105;
const int CORRECTION_INNER = 75;

// motor calibration for line following
const int RIGHT_MOTOR_CALIBRATION = 0;
const int LEFT_MOTOR_CALIBRATION = 25;

// for getAvgBlackOrWhite, WHITE = 1 and BLACK = 2
const int WHITE = 1;
const int BLACK = 2;

// Line following helper booleans declaration so that they're global
bool leftOuterBlack;
bool leftMiddleBlack;
bool leftInnerBlack;
bool rightInnerBlack;
bool rightMiddleBlack;
bool rightOuterBlack;
bool anyLeftBlack;
bool anyRightBlack;
int blackCount;

// ~~~~~~~~~~~~~~~~~~~~~~ AUTO-CALIBRATION ~~~~~~~~~~~~~~~~~~~~~~
// number of samples for the log
const int SENSOR_SAMPLES_AMOUNT = 10;

// used to store the average readings of all the white and black sensors
float whiteAvg[NUM_SENSORS] = {0};
float blackAvg[NUM_SENSORS] = {0};
// long whiteAvgTotalAverage = 0;
// long blackAvgTotalAverage = 0;

// the average targeted by the script, so greater than targetAvg is black,
// lower is white
const int TARGET_AVG = 500;

float whiteSum[NUM_SENSORS] = {0};
float blackSum[NUM_SENSORS] = {0};

// keeps track of how many times the robot went over white or black
int whiteRuns = 0;
int blackRuns = 0;

// 2D log array and index
int sensorLog[NUM_SENSORS][SENSOR_SAMPLES_AMOUNT];
int logIndex = 0;
bool logFull = false;

bool lineSensorsCalibrated = false;

// this is the number of black lines the robot faces for calibration.
// it is 3 black lines if the robot starts within the parking space on the white.
const int NUMBER_OF_BLACK_LINES_INITIAL_CALIBRATION = 3;

// ~~~~~~~~~~~~~~~~~~~~~~ GRIPPER VALUES ~~~~~~~~~~~~~~~~~~~~~~
#define SERVO_PIN 5
#define GRIPPER_OPEN_US 1820
#define GRIPPER_CLOSE_US 1050
#define SERVO_REFRESH_INTERVAL_MS 20

// This will be the target pulse width for the gripper (was volatile int)
int gripperPulseUs = GRIPPER_OPEN_US;
unsigned long lastServoMs = 0;

// ~~~~~~~~~~~~~~~~~~~~~~ NEOPIXELS VALUES ~~~~~~~~~~~~~~~~~~~~~~
// NeoPixels
const int PIN_NEO = A4;
const int NUM_PIXELS = 4;

// Pixel position mapping
// 0 = back left, 1 = back right, 2 = front right, 3 = front left
const int FRONT_LEFT = 3;
const int FRONT_RIGHT = 2;
const int BACK_LEFT = 0;
const int BACK_RIGHT = 1;

// light colors
// names GREEN, RED, and BLUE represent the RGB values these constants have to be placed in, not the color that they will produce
#define LED_COLOR_OFF 0
#define LED_FORWARD_GREEN 150
#define LED_BACKWARD_RED 150
#define LED_TURN_RED 150
#define LED_TURN_GREEN 120
#define LED_TURN_BLUE 0

// initializes LEDs to object pixels
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEO, NEO_RGB + NEO_KHZ800);

void setup() {
    Serial.begin(9600);

    // ~~~~~~~~~~~~~~~~~~~~~~ MOTORS SETUP ~~~~~~~~~~~~~~~~~~~~~~
    pinMode(LEFT_FORWARD_PIN, OUTPUT);
    pinMode(LEFT_BACKWARD_PIN, OUTPUT);
    pinMode(RIGHT_FORWARD_PIN, OUTPUT);
    pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

    pinMode(ROTATION_LEFT_PIN, INPUT_PULLUP);
    pinMode(ROTATION_RIGHT_PIN, INPUT_PULLUP);

    // Converts encoder pin numbers into interrupt pins needed by attachInterrupt
    int leftInterruptPin = digitalPinToInterrupt(ROTATION_LEFT_PIN);
    int rightInterruptPin = digitalPinToInterrupt(ROTATION_RIGHT_PIN);

    // When there is a change in the encoder pins, call the function
    // leftEncoderInterrupt or rightEncoderInterrupt respectively
    attachInterrupt(leftInterruptPin, leftEncoderInterrupt, CHANGE);
    attachInterrupt(rightInterruptPin, rightEncoderInterrupt, CHANGE);

    stopMotors();

    // ~~~~~~~~~~~~~~~~~~~~~~ ULTRASOUND SENSORS SETUP ~~~~~~~~~~~~~~~~~~~~~~
    pinMode(TRIG_FRONT, OUTPUT);
    pinMode(ECHO_FRONT, INPUT);

    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);

    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);

    // ~~~~~~~~~~~~~~~~~~~~~~ LINE SENSORS SETUP ~~~~~~~~~~~~~~~~~~~~~~
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(LINE_SENSOR_PINS[i], INPUT);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~ GRIPPER SETUP ~~~~~~~~~~~~~~~~~~~~~~
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);

    // ~~~~~~~~~~~~~~~~~~~~~~ NEOPIXELS SETUP ~~~~~~~~~~~~~~~~~~~~~~
    // Initialize neopixels, make them visible to the rest of the code
    pixels.begin();
    pixels.show();
}

void loop() {
    Serial.println("========NEW LOOP========");
    raceLogic();
}

void raceLogic() {
    gripperUpdate();
    updateLineSensors();

    // raceStartLogic only runs at the start of the race
    raceStartLogic();
    // this function decides between running maze logic or line follower
    lineFollowOrMazeLogicDecision();
}

void lineFollowOrMazeLogicDecision() {
    // See if the sensor sees any black lines first, and skip maze logic
    if (isLineDetected()) {
        Serial.println("Line Detected, running followTheLine()");
        followTheLine();
    } else {
        runMazeLogic();
    }

    stopMotors();
}

void grabConeAndEnterMaze() {
    // After calibration is complete, go into the maze
    // Move forward until all black, then keep moving forward until all
    // white, this is to go just over the black square, and then grab the token
    driveForwardUntilAllBlack();
    driveForwardUntilAllWhite();
    closeGripper();

    // Move back a bit, turn right, and go into the maze
    moveBackwardCm(INITIAL_BACKUP_AFTER_GRAB_CM);
    turnLeftForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, false);
    moveForwardCm(INITIAL_MOVE_INTO_MAZE_CM);
    lineSensorsCalibrated = true;
    Serial.println(F("CALIBRATING SENSORS DONE"));
}

void calibrateSensors() {
    // Wait until robot is on all white sensors, and then start calibrating
        while (!allSensorsWhite()) {
            gripperUpdate();
            stopMotors();
        }

        // Repeat for each black row it has to calibrate
        // (this loop does one black and one white row)
        for (int i = 0; i < NUMBER_OF_BLACK_LINES_INITIAL_CALIBRATION; i++) {
            getAvgBlackOrWhite(WHITE);

            // Move forward until all sensors see black, then calibrate black
            driveForwardUntilAllBlack();
            getAvgBlackOrWhite(BLACK);

            // Compute weights from the two captured averages
            calculateLightSensorsCalibration();

            // Drive forward to next white
            driveForwardUntilAllWhite();
        }
}

void raceStartLogic() {
    // Calibrate line sensors if not already done, start logic if robot sees
    // something from 20-25 cm away from it, wait 3 seconds, then start.
    if (!lineSensorsCalibrated) {
        openGripper();
        bool raceStart = false;

        // Wait for token to be seen, then start auto calibration
        while (raceStart == false) {
            // Turns true if the token is within the correct distance range
            // from the ultrasound sensor
            raceStart = getStartCondition();
        }

        // Wait 3 seconds for other robot to drop token and move out of the way
        Serial.println(F("RACE START = TRUE, WAITING 3S, CALIBRATING SENSORS"));
        waitMs(MS_WAIT_AFTER_SEEING_CONE);

        calibrateSensors();
        grabConeAndEnterMaze();
        
    }
}


void runMazeLogic() {
    // Average of 3 readings, constrained
        float frontDistance = getAverageDistanceFront();
        float leftDistance = getAverageDistanceLeft();
        float rightDistance = getAverageDistanceRight();

        Serial.print(F("Front: "));
        Serial.print(frontDistance);
        Serial.print(F("  Left: "));
        Serial.print(leftDistance);
        Serial.print(F("  Right: "));
        Serial.println(rightDistance);

        bool isWallOnLeft = leftDistance < WALL_DISTANCE_SIDE && leftDistance > 0;
        bool isWallOnRight = rightDistance < WALL_DISTANCE_SIDE && rightDistance > 0;
        bool isWallForward = frontDistance < WALL_DISTANCE_FRONT && frontDistance > 0;

        Serial.print(F("isWallOnLeft = "));
        Serial.println(isWallOnLeft);
        Serial.print(F("isWallOnRight = "));
        Serial.println(isWallOnRight);
        Serial.print(F("isWallForward = "));
        Serial.println(isWallForward);

        // Only keep correct distance from the left wall if there IS a left wall
        if (isWallOnLeft) {
            // Left wall correction
            // If very far from left wall, turn left a lot
            if (leftDistance > FOLLOW_LEFT_WALL_VERY_VERY_FAR) {
                turnLeftForwardTicksWithDeadEnd(
                    LEFT_WALL_ADJUST_LARGE_TICKS,
                    false
                );
                Serial.print(
                    F("ADJUSTMENT: Very very far from left wall, turning left ")
                );
            } else if (leftDistance > FOLLOW_LEFT_WALL_VERY_FAR) {
                Serial.print(F("ADJUSTMENT: Very far from left wall, turning left "));
                turnLeftForwardTicksWithDeadEnd(
                    LEFT_WALL_ADJUST_MEDIUM_TICKS,
                    false
                );
            } else if (leftDistance > FOLLOW_LEFT_WALL_FAR) {
                Serial.print(F("ADJUSTMENT: far from left wall, turning left "));
                turnLeftForwardTicksWithDeadEnd(
                    LEFT_WALL_ADJUST_SMALL_TICKS,
                    false
                );
            } else if (
                leftDistance >= FOLLOW_LEFT_WALL_IDEAL - FOLLOW_LEFT_WALL_IDEAL_TOLERANCE
                && leftDistance <= FOLLOW_LEFT_WALL_IDEAL + FOLLOW_LEFT_WALL_IDEAL_TOLERANCE
            ) {
                Serial.print(
                    F("ADJUSTMENT: ideal distance from left wall, adjustment skipped ")
                );
            } else if (leftDistance > FOLLOW_LEFT_WALL_CLOSE) {
                turnRightForwardTicksWithDeadEnd(
                    LEFT_WALL_ADJUST_SMALL_TICKS,
                    false
                );
                Serial.print(F("ADJUSTMENT: close to left wall, turning right"));
            } else if (leftDistance > FOLLOW_LEFT_WALL_VERY_CLOSE) {
                turnRightForwardTicksWithDeadEnd(
                    LEFT_WALL_ADJUST_MEDIUM_TICKS,
                    false
                );
                Serial.print(F("ADJUSTMENT: Very close to left wall, turning right"));
            } else if (leftDistance > FOLLOW_LEFT_WALL_VERY_VERY_CLOSE) {
                turnRightForwardTicksWithDeadEnd(
                    LEFT_WALL_ADJUST_LARGE_TICKS,
                    false
                );
                Serial.print(
                    F("ADJUSTMENT: Very very close to left wall, turning right")
                );
            }
        }

        // Follows the left wall all the time
        // If it is at a dead end, escape
        if (isWallOnLeft && isWallOnRight && isWallForward) {
            if (leftDistance > rightDistance) {
                // If more space on left, recover turning left
                turnLeftBackwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, true);
                turnLeftForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, true);
                Serial.print(
                    F("Dead end with left wall further away, Turning 180 degrees left ")
                );
            } else {
                // Else, recover turning right
                turnRightBackwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, true);
                turnRightForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, true);
                Serial.print(
                    F("Dead end with right wall further away, Turning 180 degrees right ")
                );
            }
        }
        // Else if left is open, go left
        else if (!isWallOnLeft) {
            // If it is too close to the wall to turn left, go back first
            if (isTooCloseToFrontWallForTurnLeftOrRight()) {
                moveBackwardCm(LEFT_OPEN_PRE_TURN_BACKUP_CM);
            }
            // If it is far enough from the front wall, move forward a little bit
            else if (!cannotBeCloserToWallForTurnLeftOrRight()) {
                moveForwardCm(LEFT_OPEN_PRE_TURN_FORWARD_CM);
            }

            // Will always turn left and go forward 10 cm
            turnLeftForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, false);
            moveForwardCm(LEFT_OPEN_POST_TURN_FORWARD_CM);
            Serial.print(F("No wall on left, Going forward, then left, then forward"));
        }

        // If forward is open, go forward
        else if (!isWallForward) {
            float moveCm = FORWARD_OPEN_DEFAULT_MOVE_CM;

            // Only shorten the forward move if both sides are not extremely close
            if (
                frontDistance > 0
                && frontDistance < (MIN_DISTANCE_FRONT_LIMIT_TO_TURN_LEFT_OR_RIGHT + moveCm + DISTANCE_SAFETY_MARGIN_CM) 
                && leftDistance > FORWARD_OPEN_SIDE_CLEARANCE_CM
                && rightDistance > FORWARD_OPEN_SIDE_CLEARANCE_CM
            ) {
                moveCm = FORWARD_OPEN_SHORT_MOVE_CM;
                Serial.println(F("FORWARD OPEN BUT CLOSE, SHORTENING MOVE TO: "));
                Serial.println(moveCm);
                Serial.println(F("FRONT DISTANCE: "));
                Serial.println(frontDistance);
            }

            if (moveCm > 0) {
                moveForwardCm(moveCm);
            }

            Serial.print(F("Wall left but none forward, Going forward "));
        }
        // If right is open, go right
        else {
            if (isTooCloseToFrontWallForTurnLeftOrRight()) {
                moveBackwardCm(RIGHT_OPEN_PRE_TURN_BACKUP_CM);
            } else if (!cannotBeCloserToWallForTurnLeftOrRight()) {
                moveForwardCm(RIGHT_OPEN_PRE_TURN_FORWARD_CM);
            }

            turnRightForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, false);
            moveForwardCm(RIGHT_OPEN_POST_TURN_FORWARD_CM);
            Serial.print(
                F("Wall left and forward, Going forward, right, and then forward")
            );
        }
}