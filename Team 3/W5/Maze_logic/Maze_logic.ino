#include <Adafruit_NeoPixel.h>

///~~~~~~~~~~NOT DONE
///LOGIC:
///TODO: DECIDE HOW TO HANDLE MEDIAN READINGS THAT RETURN -1;
///TODO: if extra time, make it easy to quickly switch between left and right priority
///TODO: COMMENT RACE FINISH LOGIC BACK IN, WAS REMOVED FOR TESTING PURPOSES

///CODE QUALITY:
///TODO: spread responsibilities between functions
///TODO: remove all logic from main loop and only call function
///TODO: make constants instead of hardcoding the values for anything
///TODO: make sure code follows the C++ coding conventions

///TODO: make a state machine, do lots and lots of abstraction, and put methods in methods, etc.

///~~~~~~~~~~~~~~~~~~~~~~ MOTORS VALUES ~~~~~~~~~~~~~~~~~~~~~~
#define LEFT_FORWARD_PIN 6
#define LEFT_BACKWARD_PIN 10
#define RIGHT_FORWARD_PIN 11
#define RIGHT_BACKWARD_PIN 9

#define ROTATION_LEFT_PIN 2
#define  ROTATION_RIGHT_PIN 3

// Motor speeds for straight driving
#define LEFT_FORWARD_SPEED 230 // was 225
#define LEFT_BACKWARD_SPEED 240
#define RIGHT_FORWARD_SPEED 223 //was 223
#define RIGHT_BACKWARD_SPEED 200

// Motor speeds for turning
#define LEFT_TURN_SPEED 210
#define RIGHT_TURN_SPEED 200

// Wheel and encoder values
const float WHEEL_DIAMETER_CM = 6.5;
const float WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const float TICKS_PER_REVOLUTION = 40.0;
const float TICKS_PER_CM = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_CM;

// Ignore encoder pulses that happen too quickly (unsigned long because it's what micros() uses)
const unsigned long MINIMUM_EDGE_TIME_US = 150;

// 90 degree turn calibration
#define TURN_90_TARGET_TICKS 35 //was 32
#define TURN_5_TARGET_TICKS 2

// Timeout for recovery
const unsigned long STALL_TIMEOUT_MS = 1000;

// Encoder counters (volatile to ensure it's always upated)
volatile unsigned long leftTickCount = 0;
volatile unsigned long rightTickCount = 0;

// Time of previous encoder pulse
volatile unsigned long lastLeftEdgeTime = 0;
volatile unsigned long lastRightEdgeTime = 0;

// Constants for recovery
#define MOVE_RECOVERY_ISDEADEND_CM 2

///~~~~~~~~~~~~~~~~~~~~~~ ULTRASOUND SENSORS VALUES ~~~~~~~~~~~~~~~~~~~~~~ 
// front
#define TRIG_FRONT 12
#define ECHO_FRONT 13
// left
#define TRIG_LEFT 7
#define ECHO_LEFT 4
// right
#define TRIG_RIGHT A3
#define ECHO_RIGHT 8

// distance to wall 
//values for deciding if to go left, straight or right
#define WALL_DISTANCE_SIDE 18 // cm
#define WALL_DISTANCE_FRONT 15 //was 18 but idfk bro 

//values for sticking close to the left wall in cm
#define FOLLOW_LEFT_WALL_VERY_VERY_CLOSE 1
#define FOLLOW_LEFT_WALL_VERY_CLOSE 3
#define FOLLOW_LEFT_WALL_CLOSE 5
#define FOLLOW_LEFT_WALL_IDEAL 7
#define FOLLOW_LEFT_WALL_FAR 9
#define FOLLOW_LEFT_WALL_VERY_FAR 12
#define FOLLOW_LEFT_WALL_VERY_VERY_FAR 15

///distance to front wall for deciding if to move forward before turning or not
#define MAX_DISTANCE_FRONT_LIMIT_TO_TURN_LEFT_OR_RIGHT 14
#define MIN_DISTANCE_FRONT_LIMIT_TO_TURN_LEFT_OR_RIGHT 10

//amount of samples of ultrasound sensors to average out
#define NUM_SAMPLES_ULTRASOUND 3

//distance that token has to be from ultrasound sensor to start the robot
#define START_CONDITION_MIN_DISTANCE 20
#define START_CONDITION_MAX_DISTANCE 25

// ultrasound sensors wait a certain ms between measuremnts to avoid bad readings, this is that value in ms (was 30, shortened to 10 for speed)
#define ULTRASOUND_SENSOR_MEASURMENT_DELAY_TIME_MS 10

// Time to wait in ms after robot first sees the cone, to allow the robot that drops the cone to move out of the way
#define MS_WAIT_AFTER_SEEING_CONE 3000

// recovery constants for ultrasound average reading functions
#define ULTRASOUND_SENSOR_MAX_FAILS_IN_A_ROW 3
#define ULTRASOUND_SENSOR_FAIL_RECOVERY_CM 3


///~~~~~~~~~~~~~~~~~~~~~~ LINE SENSORS ~~~~~~~~~~~~~~~~~~~~~~
// D1 = A0, D2 = A1, D3 = A2, D4 = A3, D5 = A4, D6 = A5, D7 = A6, D8 = A7
#define NUM_SENSORS 6
const int LINE_SENSOR_PINS[NUM_SENSORS] = {A0, A1, A2, A5, A6, A7};
int lineValues[NUM_SENSORS]; // this keeps the sensor readings of the moment
/// pin A3 is used for trig sensor right ultrasound sensor
// sensor calibration initial weightings (gets calibrated properly later)
int weights[NUM_SENSORS] = {-273, -264, -253, -276, -309, -322};

// thresholds
const int LIGHT_SENSOR_WHITE_THRESHOLD = 400;
const int LIGHT_SENSOR_BLACK_THRESHOLD = 600;

// for hysterysis, remembers what the sensor's last value was
bool sensorBlack[NUM_SENSORS] = {false};

// sensors declared here for flexibility from array
const int RIGHT_OUTER_SENSOR = 0;
const int RIGHT_MIDDLE_SENSOR = 1;
const int RIGHT_INNER_SENSOR = 2;
const int LEFT_OUTER_SENSOR = 3;
const int LEFT_MIDDLE_SENSOR = 4;
const int LEFT_INNER_SENSOR = 5;

// speed settings for line following
const int SPEED_NONE = 230; //Speed for all sensors detecting white (means black line is perfectly in the center)
const int SPEED_INNER = 200; //speed for when one of the inner sensors are on black
const int SPEED_MIDDLE = 180; //speed for when one of the middle sensors (second to edge) are on black
const int SPEED_EDGE = 0; //speed for when outer sensors (at the edge)are on black

//correction PWM to apply to motor when line followoing based on where the lines are
const int CORRECTION_EDGE = 180;
const int CORRECTION_MIDDLE = 105;
const int CORRECTION_INNER = 75;

//motor calibration for line following
const int RIGHT_MOTOR_CALIBRATION = 0;
const int LEFT_MOTOR_CALIBRATION = 25;

// for getAvgBlackOrWhite, WHITE = 1 and BLACK = 2, this just makes reading the code easier
const int WHITE = 1;
const int BLACK = 2;

///~~~~~~~~~~~~~~~~~~~~~~ AUTO-CALIBRATION ~~~~~~~~~~~~~~~~~~~~~~
///number of samples for the log
const int SENSOR_SAMPLES_AMOUNT = 10;
//used to store the average readings of all the white and black sensors
float whiteAvg[NUM_SENSORS] = {0};
float blackAvg[NUM_SENSORS] = {0};
long whiteAvgTotalAverage = 0;
long blackAvgTotalAverage = 0;
///the average targeted by the script, so greater than targetAvg is black, lower is white
int targetAvg = 500;
const int CALIB_RUNS = SENSOR_SAMPLES_AMOUNT;            // number of passes per color, two rows in one log of passes
float whiteSum[NUM_SENSORS] = {0};
float blackSum[NUM_SENSORS] = {0};
//keeps track of how many times the robot went over white or black
int whiteRuns = 0;
int blackRuns = 0;
///2D log array and index
int sensorLog[NUM_SENSORS][SENSOR_SAMPLES_AMOUNT];
int logIndex = 0;
bool logFull = false;

bool lineSensorsCalibrated = false;

///this is the number of black lines the robot faces for calibration. it is 3 black lines if the robot starts within the parking space on the white.
const int NUMBER_OF_BLACK_LINES_INITIAL_CALIBRATION = 3;

///~~~~~~~~~~~~~~~~~~~~~~ GRIPPER VALUES ~~~~~~~~~~~~~~~~~~~~~~
#define SERVO_PIN 5 
#define GRIPPER_OPEN_US  1820
#define GRIPPER_CLOSE_US 1050

// This will be the target pulse width for the gripper
volatile int gripperPulseUs = GRIPPER_OPEN_US;
unsigned long lastServoMs = 0;

///~~~~~~~~~~~~~~~~~~~~~~ NEOPIXELS VALUES ~~~~~~~~~~~~~~~~~~~~~~
// NeoPixels
const int PIN_NEO = A4;
const int NUM_PIXELS = 4;
// Pixel position mapping
// 0 = back left, 1 = back right, 2 = front right, 3 = front left
const int FRONT_LEFT  = 3;
const int FRONT_RIGHT = 2;
const int BACK_LEFT   = 0;
const int BACK_RIGHT  = 1;
// initializes LED's to object pixels
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEO, NEO_RGB + NEO_KHZ800);

void setup() {

  Serial.begin(9600);

///~~~~~~~~~~~~~~~~~~~~~~ MOTORS SETUP ~~~~~~~~~~~~~~~~~~~~~~
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  pinMode(ROTATION_LEFT_PIN, INPUT_PULLUP);
  pinMode(ROTATION_RIGHT_PIN, INPUT_PULLUP);

  // Converts encoder pin numbers into interrupt pins which are needed by attach interrupt
  int leftInterruptPin = digitalPinToInterrupt(ROTATION_LEFT_PIN);
  int rightInterruptPin = digitalPinToInterrupt(ROTATION_RIGHT_PIN);

  // when there is a change in the encoder pins, call the function leftEncoderInterrupt or rightEncoderInterrupt respectively
  attachInterrupt(leftInterruptPin, leftEncoderInterrupt, CHANGE);
  attachInterrupt(rightInterruptPin, rightEncoderInterrupt, CHANGE);

  stopMotors();

///~~~~~~~~~~~~~~~~~~~~~~ ULTRASOUND SENSORS SETUP ~~~~~~~~~~~~~~~~~~~~~~
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  ///~~~~~~~~~~~~~~~~~~~~~~ LINE SENSORS SETUP ~~~~~~~~~~~~~~~~~~~~~~
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(LINE_SENSOR_PINS[i], INPUT);
  }

  ///~~~~~~~~~~~~~~~~~~~~~~ GRIPPER SETUP ~~~~~~~~~~~~~~~~~~~~~~
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);

  ///~~~~~~~~~~~~~~~~~~~~~~ NEOPIXELS SETUP ~~~~~~~~~~~~~~~~~~~~~~
  // Initialize neopixels, make them visible to the rest of the code
  pixels.begin();
  pixels.show();
}

void loop() {

  Serial.println("========NEW LOOP========");

  gripperUpdate();
  updateLineSensors();

///Calibrate line sensors if not already done, start logic if robot sees something from 20-25cm away from it, waits 3 seconds, then starts. 
  if (!lineSensorsCalibrated) {
    openGripper();
    bool raceStart = false;
    
//  Wait for token to be seen, then start auto calibration
    while (raceStart == false) {
//  Turns true if the token is within the correct distance range from the ultrasound sensor
      raceStart = getStartCondition();
    }

// wait 3 seconds for other robot to drop token and move out the way
    Serial.println(F("RACE START = TRUE, WAITING 3S, CALIBRATING SENSORS"));
    waitMs(MS_WAIT_AFTER_SEEING_CONE);
    
    // Wait until robot is on all white sensors, and then start calibrating
    while (!allSensorsWhite()) {
      gripperUpdate();
      stopMotors();
    }

// Repeat for each black row it has to calibrate (this loop does one black and one white row
    for (int i = 0; i < NUMBER_OF_BLACK_LINES_INITIAL_CALIBRATION; i++) {
      
      getAvgBlackOrWhite(WHITE);
  
      // Move forward until all sensors see black, then calibrate black.
      driveForwardUntilAllBlack();
      getAvgBlackOrWhite(BLACK);
  
      // Compute weights from the two captured averages
      calculateLightSensorsCalibration();      

      //drive forward to next white
      driveForwardUntilAllWhite();
    }

//    after calibration is complete, go into the maze
    Serial.println(F("Moving forward extra cm"));
//  Move forward untill all black, then keep moving forward until all white, this is to go just over the black square, and then grab the token
    driveForwardUntilAllBlack();
    driveForwardUntilAllWhite();
    closeGripper();
//    move back a bit, turn right, and go into the maze
    moveBackwardCm(3);
    turnLeftForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, false);
    moveForwardCm(5);
    lineSensorsCalibrated = true;
    Serial.println(F("CALIBRATING SENSORS DONE"));
  }

/// See if the sensor sees any black lines first, and skip maze logic
  if (isLineDetected()) {
    Serial.println("Line Detected, running followTheLine()");
    followTheLine();
  } else {
  

  //  Average of 3 readings, constrained
    float frontDistance = getAverageDistanceFront();
    float leftDistance = getAverageDistanceLeft();
    float rightDistance = getAverageDistanceRight();
  
    Serial.print(F("Front: "));
    Serial.print(frontDistance);
    Serial.print(F("  Left: "));
    Serial.print(leftDistance);
    Serial.print(F("  Right: "));
    Serial.println(rightDistance);
    
    bool isWallOnLeft = leftDistance < WALL_DISTANCE_SIDE && leftDistance > 0;;
    bool isWallOnRight = (rightDistance < WALL_DISTANCE_SIDE && rightDistance > 0);
    bool isWallForward = (frontDistance < WALL_DISTANCE_FRONT && frontDistance > 0);
    Serial.print(F("isWallOnLeft = "));
    Serial.println(isWallOnLeft);
    Serial.print(F("isWallOnRight = "));
    Serial.println(isWallOnRight);
    Serial.print(F("isWallForward = "));
    Serial.println(isWallForward);
  
  // Only keep correct distance from the left wall if there IS a left wall
    if (isWallOnLeft) {
      // Left wall correction
      // If very far from left wall, turn right a lot
  //    /tested calibration was 6, 4, 2 but we stepped it down
      if (leftDistance > FOLLOW_LEFT_WALL_VERY_VERY_FAR) {
        turnLeftForwardTicksWithDeadEnd(5, false);
        Serial.print(F("ADJUSTMENT: Very very far from left wall, turning left "));
      }
      else if (leftDistance > FOLLOW_LEFT_WALL_VERY_FAR) {
        Serial.print(F("ADJUSTMENT: Very far from left wall, turning left "));
        turnLeftForwardTicksWithDeadEnd(3, false);
      }
      else if (leftDistance > FOLLOW_LEFT_WALL_FAR) {
        Serial.print(F("ADJUSTMENT: far from left wall, turning left "));
        turnLeftForwardTicksWithDeadEnd(2, false);
      }
      else if (leftDistance >= FOLLOW_LEFT_WALL_IDEAL - 1 && leftDistance <= FOLLOW_LEFT_WALL_IDEAL + 1) {
        Serial.print(F("ADJUSTMENT: ideal distance from left wall, adjustment skipped "));
      }
      else if (leftDistance > FOLLOW_LEFT_WALL_CLOSE) {
        turnRightForwardTicksWithDeadEnd(2, false);
        Serial.print(F("ADJUSTMENT: close to left wall, turning right"));
      }
      else if (leftDistance > FOLLOW_LEFT_WALL_VERY_CLOSE) {
        turnRightForwardTicksWithDeadEnd(3, false);
        Serial.print(F("ADJUSTMENT: Very close to left wall, turning right"));
      }
      else if (leftDistance > FOLLOW_LEFT_WALL_VERY_VERY_CLOSE) {
        turnRightForwardTicksWithDeadEnd(5, false);
        Serial.print(F("ADJUSTMENT: Very very close to left wall, turning right"));
      }
    }
  
    // follows the left wall all the time
  // if it's at a dead end, escape
    if(isWallOnLeft && isWallOnRight && isWallForward) {
      if (leftDistance > rightDistance) {
        //  if more space on left, recover turning left
        turnLeftBackwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, true);
        turnLeftForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, true);
        Serial.print(F("Dead end with left wall further away, Turning 180 degrees left "));
      } else { // else, recover turning right
        turnRightBackwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, true);
        turnRightForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, true);
        Serial.print(F("Dead end with right wall further away, Turning 180 degrees right "));
      }
  
    }
  //  else if left is open, go left
    else if(!isWallOnLeft) {
//      if it's too close to the wall to turn left, go back first
      if (isTooCloseToFrontWallForTurnLeftOrRight()) {
        moveBackwardCm(3); //tune this later if necessary
      }
//      if its far enough from the front wall, move forward a little bit
      else if (!cannotBeCloserToWallForTurnLeftOrRight()) {
        moveForwardCm(3); 
      }

//    Will always turn left and go forward 10cm
      turnLeftForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, false);
      moveForwardCm(10); //was 5, now 3, try 10 later
      Serial.print(F("No wall on left, Going forward, then left, then forward"));
    }
    
  //  if forward is open, go forward
    // else if (!isWallForward) {
    //   moveForwardCm(10);
    //   Serial.print(F("Wall left but none forward, Going forward "));
    // }

    else if (!isWallForward) {
      float moveCm = 10;

    // only shorten the forward move if both sides are not extremely close
    if (frontDistance > 0 && frontDistance < (MIN_DISTANCE_FRONT_LIMIT_TO_TURN_LEFT_OR_RIGHT + moveCm) && leftDistance > 3 && rightDistance > 3) {
      moveCm = 5;
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
  // if right is open, go right (!isWallOnRight)
    else {
      if (isTooCloseToFrontWallForTurnLeftOrRight()) {
        moveBackwardCm(3); //tune this later if necessary
      }
      
      else if (!cannotBeCloserToWallForTurnLeftOrRight()) {
        moveForwardCm(3); 
      }
      
      turnRightForwardTicksWithDeadEnd(TURN_90_TARGET_TICKS, false);
      moveForwardCm(10);
      Serial.print(F("Wall left and forward, Going forward, right, and then forward"));
    }

  }
  
  stopMotors();
}

///~~~~~~~~~~~~~~~~~~~~~~ MOTORS FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
// Round a float to the nearest int by adding or removing 0.5 from the float because int to float doesn't round normally
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

// Reset both encoder counters to zero, turns interrupts off for writing to values then back on when function ends
void resetEncoderCounts() {
  noInterrupts(); 
  leftTickCount = 0;
  rightTickCount = 0;
  lastLeftEdgeTime = 0;
  lastRightEdgeTime = 0;
  interrupts();
}

// Get current left encoder count, put it into value, and return value, because you cannot resume interrupts after a return
//noInterrupts because writing while they're enabled might mess up the value
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
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}

// Start moving forward
void startForwardMotion() {
  showForwardLights();
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(LEFT_FORWARD_PIN, LEFT_FORWARD_SPEED);
  analogWrite(RIGHT_FORWARD_PIN, RIGHT_FORWARD_SPEED);
}

// Start moving backward
void startBackwardMotion() {
  showBackwardLights();
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, LEFT_BACKWARD_SPEED);
  analogWrite(RIGHT_BACKWARD_PIN, RIGHT_BACKWARD_SPEED);
}

// Start turning left
void startLeftForwardTurn() {
  showLeftLights();
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, RIGHT_TURN_SPEED);
}

// Start turning left backwards
void startLeftBackwardTurn() {
  showLeftLights();
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, RIGHT_TURN_SPEED);
}

// Start turning right
void startRightForwardTurn() {
  showRightLights();
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(LEFT_FORWARD_PIN, LEFT_TURN_SPEED);
}

// Start turning right backwards
void startRightBackwardTurn() {
  showRightLights();
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, LEFT_TURN_SPEED);
}

// direction = 1 = forward, else = backwards
void correctMotorSpeeds(int leftBaseSpeed, int rightBaseSpeed, int direction) {

// See how much the wheel rotations have differed
  int difference = getRightTicks() - getLeftTicks();
// If left is going faster, this returns a negative number

// Multiplied difference (difference between ticks * X) is the difference that will be written to PWM
  difference = (difference * 35);

// Slow down right and speed up left (because right is the stronger motor)
  int leftSpeed = leftBaseSpeed + difference;
  int rightSpeed = rightBaseSpeed - difference;

// Constrain corrected speed between 0 and 255
  rightSpeed = constrain(rightSpeed, 0, 255);
  leftSpeed = constrain(leftSpeed, 0, 255);

// Write corrected speeds to motors
  if (direction == 1) {
    analogWrite(LEFT_BACKWARD_PIN, 0);
    analogWrite(RIGHT_BACKWARD_PIN, 0);

    analogWrite(LEFT_FORWARD_PIN, leftSpeed);
    analogWrite(RIGHT_FORWARD_PIN, rightSpeed);
  } else {
    analogWrite(LEFT_FORWARD_PIN, 0);
    analogWrite(RIGHT_FORWARD_PIN, 0);

    analogWrite(LEFT_BACKWARD_PIN, leftSpeed);
    analogWrite(RIGHT_BACKWARD_PIN, rightSpeed);
  }
}

// Move forward a given distance in centimeters
// Boolean 
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
    correctMotorSpeeds(LEFT_FORWARD_SPEED, RIGHT_FORWARD_SPEED, 1);

    if (averageTicks >= targetTicks) {
      break;
    }

//  if robot moved forward, update last ticks and last stall time (because it hasn't stalled)
    if (averageTicks != lastTicks) {
      lastTicks = averageTicks;
      lastStallTime = millis();
    }

// if robot has not been moving for too long, do recovery, then break movement
    if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
      stopMotors();
      moveBackwardCm(5);
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
    correctMotorSpeeds(LEFT_FORWARD_SPEED, RIGHT_FORWARD_SPEED, 0);

    if (averageTicks >= targetTicks) {
      break;
    }
    
    if (averageTicks != lastTicks) {
      lastTicks = averageTicks;
      lastStallTime = millis();
    }

    if (millis() - lastStallTime >= STALL_TIMEOUT_MS) {
      stopMotors();
      moveForwardCm(5);
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
        moveBackwardCm(MOVE_RECOVERY_ISDEADEND_CM);
        turnLeftForwardTicksWithDeadEnd(remainingTicks, true);
//        turnRightBackwardTicksWithDeadEnd(remainingTicks, true);/
        break;
      } else {
        stopMotors();
        moveBackwardCm(3);
        turnRightBackwardTicksWithDeadEnd(5, false);
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
        moveBackwardCm(MOVE_RECOVERY_ISDEADEND_CM);
        turnRightForwardTicksWithDeadEnd(remainingTicks, true);
//        turnLeftBackwardTicksWithDeadEnd(remainingTicks, true);/
        break;
      } else {
        stopMotors();
        moveBackwardCm(3);
        turnLeftBackwardTicksWithDeadEnd(5, false);
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
        moveForwardCm(MOVE_RECOVERY_ISDEADEND_CM);
        turnLeftBackwardTicksWithDeadEnd(remainingTicks, true);
//        turnRightForwardTicksWithDeadEnd(remainingTicks, true);/
        break;
      } else {
        stopMotors();
        moveForwardCm(3);
        turnRightForwardTicksWithDeadEnd(5, false);
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
        moveForwardCm(MOVE_RECOVERY_ISDEADEND_CM);
        turnRightBackwardTicksWithDeadEnd(remainingTicks, true);
//        turnLeftForwardTicksWithDeadEnd(remainingTicks, true);/
        break;
      } else {
        stopMotors();
        moveForwardCm(3);
        turnRightForwardTicksWithDeadEnd(5, false);
        break; 
      }
    }
  }

  stopMotors();
}

///~~~~~~~~~~~~~~~~~~~~~~ ULTRASOUND SENSORS FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
float getDistance(int trigPin, int echoPin) 
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

// If signal takes too long, return -1
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) {
    return -1;
  }

//  long duration = pulseIn(echoPin, HIGH);/ was this
// this is speed of sound by microseconds so: distance (cm) = duration (µs/microseconds)* 0.017
// 0.017 is speed of sound already divided by 2, so instead of 0.034 its 0.017
  float distance = duration * 0.017; 

  return distance;
}

///gets the average of the read distance from the sensor "amount" amount of times
float getAverageDistance(int trigPin, int echoPin, int amount) {
  if (amount <= 0) {
    return -1;
  }

  float sum = 0.0;
  int validCount = 0;
  int attempts = 0;
  int maxAttempts = amount * 3;

  while (validCount < amount && attempts < maxAttempts) {
    attempts++;

    float distance = getDistance(trigPin, echoPin);

    if (distance > 0 && distance < 1000) {
      sum += distance;
      validCount++;
    }

    waitMs(ULTRASOUND_SENSOR_MEASURMENT_DELAY_TIME_MS);
  }

  if (validCount < amount) {
    return -1;
  }

// Wait at the end of the loop because sometimes this function gets called many times in a row
  waitMs(ULTRASOUND_SENSOR_MEASURMENT_DELAY_TIME_MS);
  return sum / validCount;
}

float getAverageDistanceWithRetry(int trigPin, int echoPin) {
  int failedTimesInARow = 0;

  while (failedTimesInARow < ULTRASOUND_SENSOR_MAX_FAILS_IN_A_ROW) {
    float distance = getAverageDistance(trigPin, echoPin, NUM_SAMPLES_ULTRASOUND);

    if (distance != -1) {
      return distance;
    }

    failedTimesInARow++;
    Serial.print(F("ULTRASOUND FAILED, RETRYING. FAIL COUNT = "));
    Serial.println(failedTimesInARow);
  }

  Serial.println(F("ULTRASOUND FAILED TOO MANY TIMES, MOVING BACKWARD"));
  moveBackwardCm(ULTRASOUND_SENSOR_FAIL_RECOVERY_CM);

  return -1;
}

// float getAverageDistanceFront() {
//   return constrain(getAverageDistance(TRIG_FRONT, ECHO_FRONT, NUM_SAMPLES_ULTRASOUND), 4, 1000);
// }

// float getAverageDistanceLeft() {
//   return constrain(getAverageDistance(TRIG_LEFT, ECHO_LEFT, NUM_SAMPLES_ULTRASOUND), 0, 1000);
// }

// float getAverageDistanceRight() {
//   return constrain(getAverageDistance(TRIG_RIGHT, ECHO_RIGHT, NUM_SAMPLES_ULTRASOUND), 0, 1000);
// }

float getAverageDistanceFront() {
  float distance = getAverageDistanceWithRetry(TRIG_FRONT, ECHO_FRONT);

  if (distance == -1) {
    return -1;
  }

  return constrain(distance, 4, 1000);
}

float getAverageDistanceLeft() {
  float distance = getAverageDistanceWithRetry(TRIG_LEFT, ECHO_LEFT);

  if (distance == -1) {
    return -1;
  }

  return constrain(distance, 0, 1000);
}

float getAverageDistanceRight() {
  float distance = getAverageDistanceWithRetry(TRIG_RIGHT, ECHO_RIGHT);

  if (distance == -1) {
    return -1;
  }

  return constrain(distance, 0, 1000);
}

bool getStartCondition() {
  float distance = getAverageDistanceFront();
  Serial.println(F("getStartCondition sensor reading: "));
  Serial.print(distance);
  if (distance < START_CONDITION_MAX_DISTANCE && distance > START_CONDITION_MIN_DISTANCE) {
    return true;
  } else {
    return false;
  }
}

bool isTooCloseToFrontWallForTurnLeftOrRight() {
  return (getAverageDistanceFront() <= MAX_DISTANCE_FRONT_LIMIT_TO_TURN_LEFT_OR_RIGHT && getAverageDistanceFront() >= MIN_DISTANCE_FRONT_LIMIT_TO_TURN_LEFT_OR_RIGHT);
}

bool cannotBeCloserToWallForTurnLeftOrRight() {
  return (getAverageDistanceFront() <= MIN_DISTANCE_FRONT_LIMIT_TO_TURN_LEFT_OR_RIGHT);
}

///~~~~~~~~~~~~~~~~~~~~~~ LINE SENSORS/FOLLOWING FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
void updateLineSensors() {
  // Read line sensor values
  for (int i = 0; i < NUM_SENSORS; i++) 
  {
    int raw = analogRead(LINE_SENSOR_PINS[i]);
    lineValues[i] = applyLightSensorCalibration(raw,i); // apply calibration before using the value

/// Hysterisis logic, only update the sensor value as black if it goes past hysterisis
    if (lineValues[i] >= LIGHT_SENSOR_BLACK_THRESHOLD) {
      sensorBlack[i] = true;
    }
    else if (lineValues[i] <= LIGHT_SENSOR_WHITE_THRESHOLD) {
      sensorBlack[i] = false;
    }
  }
}

///Simple function to return if a line is detected on any of the active line sensors
bool isLineDetected() {
  updateLineSensors();
  return getBlackCountSensors() > 0;
}

///get number of black sensors detected on the line sensors
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

//keep updating line sensors and gripper, stop if there's no line, also stop (victory condition) if all sensors are black
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
    while (true)
    {
      gripperUpdate();
      //PLACEHOLDEER FOR VICTORY DANCE OR SOMETHING
    }
  }

  followTheLineMovement();
  }
}


void followTheLineMovement() {
  // Convenience booleans, writes if the sensor is reading black or not (accounting for hysterisis too) to a boolean valye
  bool leftOuterBlack = sensorBlack[LEFT_OUTER_SENSOR];
  bool leftMiddleBlack = sensorBlack[LEFT_MIDDLE_SENSOR];
  bool leftInnerBlack = sensorBlack[LEFT_INNER_SENSOR];
  bool rightInnerBlack = sensorBlack[RIGHT_INNER_SENSOR];
  bool rightMiddleBlack = sensorBlack[RIGHT_MIDDLE_SENSOR];
  bool rightOuterBlack = sensorBlack[RIGHT_OUTER_SENSOR];

// Convenience booleans for if any of the left or right sensors read as black
  bool anyLeftBlack  = leftOuterBlack || leftMiddleBlack || leftInnerBlack;
  bool anyRightBlack = rightOuterBlack || rightMiddleBlack || rightInnerBlack;

///count how many sensors counted black
  int blackCount = getBlackCountSensors();

/// If all sensors are black
  if (blackCount == NUM_SENSORS) {
    driveForward(SPEED_NONE + LEFT_MOTOR_CALIBRATION, SPEED_NONE + RIGHT_MOTOR_CALIBRATION);
    return;
  }

///if any right sensor and no left sensors
    if (anyRightBlack && !anyLeftBlack) {
      // EDGE FIRST
      if(rightOuterBlack) {
        driveForward(SPEED_EDGE + CORRECTION_EDGE + LEFT_MOTOR_CALIBRATION, SPEED_EDGE + RIGHT_MOTOR_CALIBRATION);
        return;
      }

      // THEN MIDDLE
      else if(rightMiddleBlack) {
        driveForward(SPEED_MIDDLE + CORRECTION_MIDDLE + LEFT_MOTOR_CALIBRATION, SPEED_MIDDLE + RIGHT_MOTOR_CALIBRATION);
        return;
      }

      // THEN INNER
      else { //rightInnerBlack
        driveForward(SPEED_INNER + CORRECTION_INNER + LEFT_MOTOR_CALIBRATION, SPEED_INNER + RIGHT_MOTOR_CALIBRATION);
        return;
      }
      
    }
  // if any left sensors and no right sensors
    else if (!anyRightBlack && anyLeftBlack) {
      
      if (leftOuterBlack) {
        driveForward(SPEED_EDGE + LEFT_MOTOR_CALIBRATION, SPEED_EDGE + CORRECTION_EDGE + RIGHT_MOTOR_CALIBRATION);
        return;
      }

      else if(leftMiddleBlack) {
        driveForward(SPEED_MIDDLE + LEFT_MOTOR_CALIBRATION, SPEED_MIDDLE + CORRECTION_MIDDLE + RIGHT_MOTOR_CALIBRATION);
        return;
      }

      else { //leftInnerBlack
        driveForward(SPEED_INNER + LEFT_MOTOR_CALIBRATION, SPEED_INNER + CORRECTION_INNER + RIGHT_MOTOR_CALIBRATION);
        return;
      }
      
    }

    else if (rightInnerBlack) {
      driveForward(SPEED_INNER + CORRECTION_INNER + LEFT_MOTOR_CALIBRATION, SPEED_INNER + RIGHT_MOTOR_CALIBRATION);
      return;
    }

    else if (leftInnerBlack) {
      driveForward(SPEED_INNER + LEFT_MOTOR_CALIBRATION, SPEED_INNER + CORRECTION_INNER + RIGHT_MOTOR_CALIBRATION);
      return;
    }

  driveForward(SPEED_INNER + LEFT_MOTOR_CALIBRATION, SPEED_INNER + RIGHT_MOTOR_CALIBRATION);
}

// motor control
void driveForward(int left, int right) {
  showForwardLights();
// constrain because sometimes value + correction goes over or under 0 and 255 respectively
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
  analogWrite(LEFT_FORWARD_PIN, left);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, right);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}

void driveBackward(int left, int right) {
  showBackwardLights();
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, left);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, right);
}

///~~~~~~~~~~~~~~~~~~~~~~ AUTO-CALIBRATION FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
// apply calibration
int applyLightSensorCalibration(int raw, int sensorIndex) {
  int v = raw + weights[sensorIndex];
  return constrain(v, 0, 1023);
}

//hysterisis logic
bool isBlack(int sensorIndex) {
  int raw = analogRead(LINE_SENSOR_PINS[sensorIndex]);
  int calibrated = applyLightSensorCalibration(raw, sensorIndex);
  return (calibrated > LIGHT_SENSOR_BLACK_THRESHOLD);
}

bool isWhite(int sensorIndex) {
  int raw = analogRead(LINE_SENSOR_PINS[sensorIndex]);
  int calibrated = applyLightSensorCalibration(raw, sensorIndex);
  return (calibrated < LIGHT_SENSOR_WHITE_THRESHOLD);
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
    correctMotorSpeeds(LEFT_FORWARD_SPEED, RIGHT_FORWARD_SPEED, 1);
  }

  stopMotors();
}

void driveForwardUntilAllWhite() {
  resetEncoderCounts();
  startForwardMotion();

  while (!allSensorsWhite()) {
    gripperUpdate();
    correctMotorSpeeds(LEFT_FORWARD_SPEED, RIGHT_FORWARD_SPEED, 1);
  }

  stopMotors();
}

///Reading light sensors and logging SENSOR_SAMPLES_AMOUNT x into array
void readLightSensorsandLog() {
  if (logFull) {
    return;
  }
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(LINE_SENSOR_PINS[i]);
    sensorLog[i][logIndex] = raw;  // store into 2D array
  }

  logIndex++;
  if (logIndex >= SENSOR_SAMPLES_AMOUNT) {
    logFull = true;
    logIndex = SENSOR_SAMPLES_AMOUNT - 1;
  }
}

void getAvgBlackOrWhite(int blackOrWhite) {
  // 1 is white, 2 is black

  // take multiple runs and accumulate
  for (int run = 0; run < CALIB_RUNS; run++) {
    resetLog();
    while (!logFull) {
      readLightSensorsandLog();
      gripperUpdate(); // keep servo powered during sampling
    }

    // compute this run's averages and add to sums
    for (int i = 0; i < NUM_SENSORS; i++) {
      long sum = 0;
      for (int g = 0; g < SENSOR_SAMPLES_AMOUNT; g++) {
        sum += sensorLog[i][g];
      }
      float avg = (float)sum / (float)SENSOR_SAMPLES_AMOUNT;
//      adds average to white or black sum respectively
      if (blackOrWhite == 1) {
        whiteSum[i] += avg;
      }
      else {
        blackSum[i] += avg;
      }
    }
  }

  // finalize the averages from accumulated sums
  long totalAverage = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (blackOrWhite == 1) {
      whiteAvg[i] = whiteSum[i] / (float)(whiteRuns + CALIB_RUNS);
      totalAverage += (long)whiteAvg[i];
    } else {
      blackAvg[i] = blackSum[i] / (float)(blackRuns + CALIB_RUNS);
      totalAverage += (long)blackAvg[i];
    }
  }

  if (blackOrWhite == 1) {
    whiteRuns += CALIB_RUNS;
    whiteAvgTotalAverage = totalAverage / NUM_SENSORS;
  } else {
    blackRuns += CALIB_RUNS;
    blackAvgTotalAverage = totalAverage / NUM_SENSORS;
  }
}

///reset sensor calibration log
void resetLog() {
  logIndex = 0;
  logFull = false;
}

void calculateLightSensorsCalibration() {

//   getAvgBlackOrWhite(1);/
//   getAvgBlackOrWhite(2);/
///THESE NEED TO BE RUN BEFORE NOW

   long midPoint = (whiteAvgTotalAverage + blackAvgTotalAverage) / 2;

//    Per-sensor calibration (each sensor gets its own weight)
   for (int i = 0; i < NUM_SENSORS; i++) {
     long sensorMidPoint = (long)((whiteAvg[i] + blackAvg[i]) / 2.0f);
     weights[i] = (int)(targetAvg - sensorMidPoint);    
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

///~~~~~~~~~~~~~~~~~~~~~~ GRIPPER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
void gripperUpdate() {
  // Send one servo pulse every 20ms to keep it powered
  unsigned long now = millis();
  // if 20ms have passed, send another pulse to servo
  if (now - lastServoMs >= 20) {
    lastServoMs = now;
    digitalWrite(SERVO_PIN, HIGH);
    // PulseUs will be either the value to close or open the gripper, delay to keep the pulse HIGH for required amount
    delayMicroseconds(gripperPulseUs);
    digitalWrite(SERVO_PIN, LOW);
  }
}

// Functions to update gripperPulseUs depending on if the gripper should be update or closed, this is used in the gripperUpdate()
void openGripper()  {
  gripperPulseUs = GRIPPER_OPEN_US;
}

void closeGripper() {
  gripperPulseUs = GRIPPER_CLOSE_US;
}

// Replace ALL delay(ms) with this (keeps servo powered during waits)
void waitMs(unsigned long ms) {
  unsigned long start = millis();
  // While milis time - start time is less than desired wait time, do nothing and keep updating the gripper
  while (millis() - start < ms) {
    gripperUpdate();
  }
}


///~~~~~~~~~~~~~~~~~~~~~~ NEOPIXELS FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
void clearLights() {
// .show() writes the values to the neopixels
  pixels.clear();
  pixels.show();
}

void showForwardLights() {
  pixels.clear();
  pixels.setPixelColor(FRONT_LEFT, pixels.Color(0,150,0));
  pixels.setPixelColor(FRONT_RIGHT, pixels.Color(0,150,0));
  pixels.show();
}

void showBackwardLights() {
  pixels.clear();
  pixels.setPixelColor(BACK_LEFT, pixels.Color(150,0,0));
  pixels.setPixelColor(BACK_RIGHT, pixels.Color(150,0,0));
  pixels.show();
}

void showLeftLights() {
  pixels.clear();
  pixels.setPixelColor(FRONT_LEFT, pixels.Color(150,120,0));
  pixels.setPixelColor(BACK_LEFT, pixels.Color(150,120,0));
  pixels.show();
}

void showRightLights() {
  pixels.clear();
  pixels.setPixelColor(FRONT_RIGHT, pixels.Color(150,120,0));
  pixels.setPixelColor(BACK_RIGHT, pixels.Color(150,120,0));
  pixels.show();
}
