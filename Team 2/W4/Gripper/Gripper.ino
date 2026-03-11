#define SERVO 13              // pin connected to the servo motor (gripper)
#define GRIPPER_OPEN 1620     
#define GRIPPER_CLOSE 1100     

const int LEFTMOTORBACK = 8;        // pin controlling left motor backward
const int LEFTMOTORSTRAIGHT = 5;    // pin controlling left motor forward
const int RIGHTMOTORBACK = 11;      // pin controlling right motor backward
const int RIGHTMOTORSTRAIGHT = 10;  // pin controlling right motor forward

const int R1 = 2;   
const int R2 = 3;   

const int PULSES_FOR_25CM = 60;   // number of encoder pulses needed to move 25 cm

volatile int pulsesRight = 0;   // counter for right wheel encoder pulses
volatile int pulsesLeft = 0;    // counter for left wheel encoder pulses

boolean first = true;    
boolean second = false;  
boolean third = false;   

unsigned long startTime = millis(); 

void setup() {

  pinMode(SERVO, OUTPUT); // servo pin configured as output
  
  // configure motor control pins
  pinMode(LEFTMOTORBACK, OUTPUT);
  pinMode(LEFTMOTORSTRAIGHT, OUTPUT);
  pinMode(RIGHTMOTORBACK, OUTPUT);
  pinMode(RIGHTMOTORSTRAIGHT, OUTPUT);
  
  // attach interrupts for wheel encoders
  attachInterrupt(digitalPinToInterrupt(R1), countRight, RISING); // increment right encoder count
  attachInterrupt(digitalPinToInterrupt(R2), countLeft, RISING);  // increment left encoder count
  
  stopMoving(); // ensure robot is stopped at startup
}

void loop() {

// open the gripper for 1 second
holdGripper(GRIPPER_OPEN,1000);

  if (first) {
      // close the gripper (simulate grabbing at the start)
      holdGripper(GRIPPER_CLOSE, 2000);
  
      // switch to next stage
      first = false;
      second = true;
  
      startTime = millis(); // save current time
  }

  if (second){
    // open the gripper again
    holdGripper(GRIPPER_OPEN, 2000);

    // move to the next stage
    second = false;
    third = true;

    startTime = millis(); // save current time
  }

if (third){

  // move forward approximately 25 cm
  moveDistance(PULSES_FOR_25CM);
  
  stopMoving(); // stop after movement
  
  // close the gripper to grab the cone
  holdGripper(GRIPPER_CLOSE,2000);
  
  // move forward another 25 cm
  moveDistance(PULSES_FOR_25CM);
  
  stopMoving(); // final stop
  
  // infinite loop to keep sending servo signal
  while(1) {
    gripper(0);
  }
}

}

void moveDistance(int pulsesNeeded){

int start = pulsesRight; // store current encoder value

// keep moving until the required number of pulses is reached
while(pulsesRight < start + pulsesNeeded){

moveForward(200);  
gripper(0);   // keep sending servo signal so it holds its position

}

}

void moveForward(int speed){

// drive robot forward
// slight speed difference compensates motor imbalance

analogWrite(RIGHTMOTORSTRAIGHT,speed+15);
analogWrite(LEFTMOTORSTRAIGHT,speed);

analogWrite(RIGHTMOTORBACK,0);
analogWrite(LEFTMOTORBACK,0);

}

void stopMoving(){

// stop both motors

analogWrite(RIGHTMOTORSTRAIGHT,0);
analogWrite(LEFTMOTORSTRAIGHT,0);
analogWrite(RIGHTMOTORBACK,0);
analogWrite(LEFTMOTORBACK,0);

}

void countLeft(){

// interrupt handler for left wheel encoder
pulsesLeft++;

}

void countRight(){

// interrupt handler for right wheel encoder
pulsesRight++;

}

void holdGripper(int pulse,int timeMs){

// keep the gripper in a specific position for a given time

unsigned long start = millis();

while(millis() - start < timeMs){
gripper(pulse);
}

}

void gripper(int newPulse){
  // generates control pulses for the servo
  static unsigned long timer; // timer to control pulse interval
  static int pulse;           // stored pulse width
  
  if(millis() > timer){
    // update stored pulse if a new value is provided
    if(newPulse > 0){  
      pulse = newPulse;  
    }  
    // generate servo control pulse
    digitalWrite(SERVO, HIGH);  
    delayMicroseconds(pulse);  
     digitalWrite(SERVO, LOW);  
    // servo signal repeated roughly every 20 ms
    timer = millis() + 20;
  }
}
