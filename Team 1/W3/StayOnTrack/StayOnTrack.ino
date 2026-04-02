#define sensorThreshold 850  //instead of hysteresis, we use a threshold

const int sensorWeights[8]{ -200, -170, -100, -90, 90, 100, 170, 200};  //depending on how far the sensor is from the center, it has more weight
const int sensorPins[8]{ A0, A1, A2, A3, A4, A5, A6, A7 };               //declaring the sensor pins as an array
bool sensorRead[8]{ 0, 0, 0, 0, 0, 0, 0, 0 };                            //the initial sensor readings are set to 0

int lastDestination;
int sensorVal = 0;

int newLeftSpeed;
int newRightSpeed;

float ancoefficient = 2.6; //THE ANDRII COEFFICIENT - thank you andrii :)

#define MOTOR_A_FW 10  //A1 to D10, this is motor A forwards
#define MOTOR_A_BW 9   //A2 to D9, this is motor A backwards
#define MOTOR_B_BW 6   //B1 to D6, this is motor B backwards
#define MOTOR_B_FW 5   //B2 to D5, this is motor B forwards

#define SPEED_A_FW 249  //motor A forwards speed
#define SPEED_B_FW 255  //motor B forwards speed

void setup() {
  Serial.begin(9600);
  //initialising motors as OUTPUTs
  pinMode(MOTOR_A_FW, OUTPUT);
  pinMode(MOTOR_A_BW, OUTPUT);
  pinMode(MOTOR_B_BW, OUTPUT);
  pinMode(MOTOR_B_FW, OUTPUT);

  //initial state of motors should be LOW
  digitalWrite(MOTOR_A_FW, LOW);
  digitalWrite(MOTOR_A_BW, LOW);
  digitalWrite(MOTOR_B_BW, LOW);
  digitalWrite(MOTOR_B_FW, LOW);
}

void loop() {
  sensorVal = sensorReading();

  if (sensorVal != 0) {
    lastDestination = sensorVal;
  }

  motorCalc(lastDestination);
  moveForwardsNoStop();
}


int sensorReading() {
  sensorVal = 0;  //resetting value

  for (int i = 0; i < 8; i++) {
    if (analogRead(sensorPins[i]) > sensorThreshold) {
      sensorVal += sensorWeights[i];
    }
  }

  return sensorVal;
}

void motorCalc(int lastDestination) {
  int sensorVal = sensorReading();

  if (sensorVal == 0) { //checking value for sensorVal -> 0 means all white
    if (lastDestination > 150) { //if lastDestination was to the right -> turn right first
      analogWrite(MOTOR_A_FW, 200);
      analogWrite(MOTOR_B_FW, 0);

      analogWrite(MOTOR_A_BW, 0);
      analogWrite(MOTOR_B_BW, 200);
      delay(20);
    }
    
    else if (lastDestination < -150) { //if lastDestination was to the left -> turn left first
      analogWrite(MOTOR_A_FW, 0);
      analogWrite(MOTOR_B_FW, 200);

      analogWrite(MOTOR_A_BW, 200);
      analogWrite(MOTOR_B_BW, 0);
      delay(20);
    }
  }

  newLeftSpeed = (SPEED_A_FW - sensorVal) * ancoefficient;
  newRightSpeed = (SPEED_B_FW + sensorVal) * ancoefficient;

  newLeftSpeed = constrain(newLeftSpeed, 0, SPEED_A_FW);
  newRightSpeed = constrain(newRightSpeed, 0, SPEED_B_FW);
}

void moveForwardsNoStop() {
  analogWrite(MOTOR_A_FW, newLeftSpeed);
  analogWrite(MOTOR_B_FW, newRightSpeed);

  analogWrite(MOTOR_A_BW, LOW);
  analogWrite(MOTOR_B_BW, LOW);
}