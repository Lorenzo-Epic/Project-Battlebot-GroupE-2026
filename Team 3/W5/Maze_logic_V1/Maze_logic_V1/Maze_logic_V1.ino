#include <string.h> // c library to use strcmp

// setup runs when arduino starts initializing the hardware

void setup(){

  Serial.begin(9600); // starts communication w computer its used for debugging

  pinMode(LEFT_FORWARD_PIN,OUTPUT);
  pinMode(LEFT_BACKWARD_PIN,OUTPUT);
  pinMode(RIGHT_FORWARD_PIN,OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN,OUTPUT);
// ultrasonic sensor front
  pinMode(FRONT_TRIG,OUTPUT); //send sound pulse
  pinMode(FRONT_ECHO,INPUT); // receive reflectiong
// sensor left
  pinMode(LEFT_TRIG,OUTPUT);
  pinMode(LEFT_ECHO,INPUT);
// sensor right
  pinMode(RIGHT_TRIG,OUTPUT);
  pinMode(RIGHT_ECHO,INPUT);
// encoder setup
  pinMode(LEFT_ENCODER_PIN,INPUT);
  pinMode(RIGHT_ENCODER_PIN,INPUT);
// interrupts when encoder signal changes run isrLeft or isrRight immediately
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN),isrLeft,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN),isrRight,CHANGE);
}

// loop
// runs forever robot always repeats
void loop(){
//robot moves 60cm forward
  move(CELL_DISTANCE_CM,"forward");

  if(reachedBlackZone()){
    stopMotors();
    while(true);
  }

  int front = readDistance(FRONT_TRIG,FRONT_ECHO);
  int left  = readDistance(LEFT_TRIG,LEFT_ECHO);
  int right = readDistance(RIGHT_TRIG,RIGHT_ECHO);

  Serial.print("F:");
  Serial.print(front);
  Serial.print(" L:");
  Serial.print(left);
  Serial.print(" R:");
  Serial.println(right);
// maze decisions
  if(left > LEFT_OPEN_DIST){

    move(90,"left");

  }
  else if(front > FRONT_OPEN_DIST){

    // continue forward

  }
  else{

    move(90,"right");

  }
}

// pins
#define LEFT_FORWARD_PIN 5 // left wheel forward
#define LEFT_BACKWARD_PIN 10 // left wheel backward
#define RIGHT_FORWARD_PIN 6 // right wheel forward
#define RIGHT_BACKWARD_PIN 9 // right wheel backward

#define LEFT_ENCODER_PIN 2
#define RIGHT_ENCODER_PIN 3

#define FRONT_TRIG 4
#define FRONT_ECHO 7
#define LEFT_TRIG 12
#define LEFT_ECHO 13
#define RIGHT_TRIG A0
#define RIGHT_ECHO A1

#define SERVO_PIN 11

// line sensors
const int LINE_SENSOR_PINS[] = {A3,A4,A5,A6};
#define BLACK_LINE_THRESHOLD 800

// motor calibration
const int CALIBRATION_FORWARD_LEFT = 255;
const int CALIBRATION_BACKWARD_LEFT = 255;
const int CALIBRATION_FORWARD_RIGHT = 243;
const int CALIBRATION_BACKWARD_RIGHT = 210;

// encoder geometry
const float WHEEL_DIAMETER_CM = 6.5f;
const int SLOTS_PER_REV = 20;
const int EDGES_PER_SLOT = 2;
const int TICKS_PER_REV = SLOTS_PER_REV * EDGES_PER_SLOT;

const float CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const float TICKS_PER_CM = (float)TICKS_PER_REV / CIRCUMFERENCE_CM;

// turn calibration
const float TURN_SLOTS_FOR_90_DEG = 8.0f;
const float TURN_TICKS_PER_DEG = (TURN_SLOTS_FOR_90_DEG * EDGES_PER_SLOT) / 90.0f;

const long TURN_SLOWDOWN_TICKS = 3;

const int TURN_SLOW_LEFT_FORWARD = 150;
const int TURN_SLOW_LEFT_BACKWARD = 150;
const int TURN_SLOW_RIGHT_FORWARD = 145;
const int TURN_SLOW_RIGHT_BACKWARD = 135;

// sensor limits
#define FRONT_OPEN_DIST 15
#define LEFT_OPEN_DIST 15
#define RIGHT_OPEN_DIST 15

#define CELL_DISTANCE_CM 60

const unsigned long EDGE_MIN_US = 150;
const unsigned long ULTRASOUND_TIMEOUT_US = 25000UL;

// encoder globals
volatile unsigned long g_leftTicks = 0;
volatile unsigned long g_rightTicks = 0;
volatile unsigned long g_lastLeftUs = 0;
volatile unsigned long g_lastRightUs = 0;

// encoder isr

void isrLeft(){
  unsigned long now = micros();
  if(now - g_lastLeftUs >= EDGE_MIN_US){
    g_leftTicks++;
    g_lastLeftUs = now;
  }
}

void isrRight(){
  unsigned long now = micros();
  if(now - g_lastRightUs >= EDGE_MIN_US){
    g_rightTicks++;
    g_lastRightUs = now;
  }
}

// motor control

void stopMotors(){

  analogWrite(LEFT_FORWARD_PIN,0);
  analogWrite(LEFT_BACKWARD_PIN,0);
  analogWrite(RIGHT_FORWARD_PIN,0);
  analogWrite(RIGHT_BACKWARD_PIN,0);

  digitalWrite(LEFT_FORWARD_PIN,LOW);
  digitalWrite(LEFT_BACKWARD_PIN,LOW);
  digitalWrite(RIGHT_FORWARD_PIN,LOW);
  digitalWrite(RIGHT_BACKWARD_PIN,LOW);
}

void driveForward(){

  analogWrite(LEFT_BACKWARD_PIN,0);
  analogWrite(RIGHT_BACKWARD_PIN,0);

  analogWrite(LEFT_FORWARD_PIN,CALIBRATION_FORWARD_LEFT);
  analogWrite(RIGHT_FORWARD_PIN,CALIBRATION_FORWARD_RIGHT);
}

void driveBackward(){

  analogWrite(LEFT_FORWARD_PIN,0);
  analogWrite(RIGHT_FORWARD_PIN,0);

  analogWrite(LEFT_BACKWARD_PIN,CALIBRATION_BACKWARD_LEFT);
  analogWrite(RIGHT_BACKWARD_PIN,CALIBRATION_BACKWARD_RIGHT);
}

void turnLeftInPlace(){

  analogWrite(LEFT_FORWARD_PIN,0);
  analogWrite(RIGHT_BACKWARD_PIN,0);

  analogWrite(LEFT_BACKWARD_PIN,CALIBRATION_BACKWARD_LEFT);
  analogWrite(RIGHT_FORWARD_PIN,CALIBRATION_FORWARD_RIGHT);
}

void turnRightInPlace(){

  analogWrite(LEFT_BACKWARD_PIN,0);
  analogWrite(RIGHT_FORWARD_PIN,0);

  analogWrite(LEFT_FORWARD_PIN,CALIBRATION_FORWARD_LEFT);
  analogWrite(RIGHT_BACKWARD_PIN,CALIBRATION_BACKWARD_RIGHT);
}

// encoder helpers (

void readTicks(unsigned long &l,unsigned long &r){
  noInterrupts();
  l = g_leftTicks;
  r = g_rightTicks;
  interrupts();
}

void resetTicks(){
  noInterrupts();
  g_leftTicks = 0;
  g_rightTicks = 0;
  g_lastLeftUs = 0;
  g_lastRightUs = 0;
  interrupts();
}

static inline long roundToLong(float x){
  return (x>=0.0f)?(long)(x+0.5f):(long)(x-0.5f);
}

// move function

void move(float amount,const char *direction){

  bool IS_DRIVE =
  strcmp(direction,"forward")==0 || //strcmp(string1, string2) to commpare direction with forward
  strcmp(direction,"backward")==0;

  bool IS_TURN =
  strcmp(direction,"left")==0 ||
  strcmp(direction,"right")==0;

  if(!IS_DRIVE && !IS_TURN) return;

  float ABS_AMOUNT = abs(amount);

  long targetTicks;

  if(IS_DRIVE)
    targetTicks = roundToLong(ABS_AMOUNT * TICKS_PER_CM);
  else
    targetTicks = roundToLong(ABS_AMOUNT * TURN_TICKS_PER_DEG);

  resetTicks();

  if(strcmp(direction,"forward")==0) driveForward();
  else if(strcmp(direction,"backward")==0) driveBackward();
  else if(strcmp(direction,"left")==0) turnLeftInPlace();
  else if(strcmp(direction,"right")==0) turnRightInPlace();

  unsigned long START_MS = millis();

  unsigned long TIMEOUT_MS =
  IS_TURN ? (700 + targetTicks*80)
          : (1500 + targetTicks*120);

  bool slowPhase = false;

  while(true){

    unsigned long l,r;
    readTicks(l,r);

    unsigned long progress = (l+r)/2;
    long remaining = targetTicks - progress;

    if(IS_TURN && !slowPhase && remaining <= TURN_SLOWDOWN_TICKS){

      slowPhase = true;

      if(strcmp(direction,"left")==0){

        analogWrite(LEFT_FORWARD_PIN,0);
        analogWrite(RIGHT_BACKWARD_PIN,0);
        analogWrite(LEFT_BACKWARD_PIN,TURN_SLOW_LEFT_BACKWARD);
        analogWrite(RIGHT_FORWARD_PIN,TURN_SLOW_RIGHT_FORWARD);

      }else{

        analogWrite(LEFT_BACKWARD_PIN,0);
        analogWrite(RIGHT_FORWARD_PIN,0);
        analogWrite(LEFT_FORWARD_PIN,TURN_SLOW_LEFT_FORWARD);
        analogWrite(RIGHT_BACKWARD_PIN,TURN_SLOW_RIGHT_BACKWARD);
      }
    }

    if(progress >= targetTicks) break;
    if(millis()-START_MS > TIMEOUT_MS) break;
  }

  stopMotors();
}

// ultrasonic

long readDistance(int trig,int echo){

  digitalWrite(trig,LOW);
  delayMicroseconds(2);

  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);

  long duration = pulseIn(echo,HIGH,ULTRASOUND_TIMEOUT_US);

  long distance = duration * 0.034 / 2;

  if(distance==0) distance = 100;

  return distance;
}

// inset lorenzo line detection calibration here :3
