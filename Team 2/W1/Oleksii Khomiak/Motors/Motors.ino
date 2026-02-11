
const int LEFT_MOTOR_1 = 10;
const int LEFT_MOTOR_2 = 9;
const int RIGHT_MOTOR_1 = 6;
const int RIGHT_MOTOR_2 = 5;

int Speed = 200;

//27 difference

void setup() {

  pinMode(LEFT_MOTOR_1, OUTPUT);
  pinMode(LEFT_MOTOR_2, OUTPUT);
  pinMode(RIGHT_MOTOR_1, OUTPUT);
  pinMode(RIGHT_MOTOR_2, OUTPUT);
 

}

void loop() {
  analogWrite(LEFT_MOTOR_1, Speed);
  analogWrite(LEFT_MOTOR_2, 0);

  analogWrite(RIGHT_MOTOR_1, 227);
  analogWrite(RIGHT_MOTOR_2, 0); 
  
}
