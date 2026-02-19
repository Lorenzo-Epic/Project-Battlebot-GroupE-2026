// ---------------- Servo / Gripper ----------------
#define SERVO_PIN 11 
#define GRIPPER_OPEN_US  1820
#define GRIPPER_CLOSE_US 1000

// ---------------- Motors ----------------
#define LEFT_FORWARD_PIN 5
#define LEFT_BACKWARD_PIN 10 
#define RIGHT_FORWARD_PIN 6
#define RIGHT_BACKWARD_PIN 9

// Motor PWM calibration
#define PWM_LEFT_FWD 255
#define PWM_LEFT_BWD 255
#define PWM_RIGHT_FWD 238
#define PWM_RIGHT_BWD 210

// ---------------- Encoders ----------------
#define ROTATION_LEFT_PIN 2
#define ROTATION_RIGHT_PIN 3

// Wheel geometry
#define WHEEL_DIAMETER_CM 6.5f
#define SLOTS_PER_REV 20
#define EDGES_PER_SLOT 2
#define TICKS_PER_REV (SLOTS_PER_REV * EDGES_PER_SLOT)
#define WHEEL_CIRC_CM (PI * WHEEL_DIAMETER_CM)
#define TICKS_PER_CM ((float)TICKS_PER_REV / WHEEL_CIRC_CM)

// Shared with interrupts
volatile unsigned long leftTicks = 0;
volatile unsigned long rightTicks = 0;

// Count every edge of the wheel encoders (40x per rotation instead of 20x)
void isrLeft() { 
  leftTicks++;
}
void isrRight() {
  rightTicks++; 
}

// ---------------- helper functions ----------------
void stopMotors() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);

  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
}

void driveForwardCalibrated() {
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);

  analogWrite(LEFT_FORWARD_PIN, PWM_LEFT_FWD);
  analogWrite(RIGHT_FORWARD_PIN, PWM_RIGHT_FWD);
}

void resetEncoders() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

unsigned long averageTicks() {
  unsigned long l;
  unsigned long r;
  noInterrupts();
  l = leftTicks;
  r = rightTicks;
  interrupts();
  return (l + r) / 2;
}

// Drive forward x cm, calculated with encoder ticks
void driveForwardCm(int cm) {
// Convert target cm into encoder ticks
  long targetTicks = (long)((float)cm * TICKS_PER_CM + 0.5f);
  if (targetTicks <= 0) {
    return;
  }

  resetEncoders();
  driveForwardCalibrated();

// Stop when ticks exceeded
  while (true) {
    if ((long)averageTicks() >= targetTicks) {
      break;
    }
  }

  stopMotors();
}

// ---------------- Servo pulses ----------------
// Sends one servo pulse (and waits out the 20ms frame)
void servoPulse(int pulseUs) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(SERVO_PIN, LOW);
  delay(20);
}

// Hold gripper at a position for ~1 second by repeating pulses
void setGripperFor1s(int pulseUs) {
  for (int i = 0; i < 50; i++) { // 50 * 20ms = ~1000ms  
    servoPulse(pulseUs);
  }
}

void openGripper() {
  setGripperFor1s(GRIPPER_OPEN_US);
  }
void closeGripper() {
  setGripperFor1s(GRIPPER_CLOSE_US);
  }

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  pinMode(ROTATION_LEFT_PIN, INPUT_PULLUP);
  pinMode(ROTATION_RIGHT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ROTATION_LEFT_PIN), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTATION_RIGHT_PIN), isrRight, CHANGE);

  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);

  stopMotors();
}

void loop() {
  // make sure its closed first
  closeGripper();

  // open, wait a second, close, wait a second, open
  openGripper();
  delay(1000);
  closeGripper();
  delay(1000);
  openGripper();

  // drive towards cone
  driveForwardCm(25);

  // grab cone
  closeGripper();

  // drive 25 more cm
  driveForwardCm(25);
}
