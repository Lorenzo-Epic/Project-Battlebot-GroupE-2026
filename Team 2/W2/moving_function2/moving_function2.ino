#include <Arduino.h> 
const int MOTORA1 = 6;  // right forward 
const int MOTORA2 = 5;  // right back
const int MOTORB1 = 10; // left back
const int MOTORB2 = 11;  //left forward

void MoveForward();     // Move forwards 
void MoveBackward();    // Move backwards 
void TurnL();           // Turn on its axis 90 degrees Left 
void TurnR();           // Turn on its axis 90 degrees Right 
void stop();            // Stops  all the wheels 

 

void setup() { 

  pinMode(MOTORA1, OUTPUT); 
  pinMode(MOTORA2, OUTPUT); 
  pinMode(MOTORB1, OUTPUT); 
  pinMode(MOTORB2, OUTPUT); 
} 

 

void loop() { 

  MoveForward(); 
  stop(); 
  MoveBackward(); 
  stop(); 
  TurnR(); 
  stop(); 
  TurnL();
  stop();  
} 

void MoveForward(){ 

 

  analogWrite(MOTORA1, 250); //right forward
  analogWrite(MOTORA2, 0); //right back
  analogWrite(MOTORB1, 0); //left back
  analogWrite(MOTORB2, 200); //left forward

 

  unsigned long stopStartTime = millis(); 

  while (millis() - stopStartTime < 5000){ 

     

  } 

} 

void MoveBackward(){ 

 

  analogWrite(MOTORA1, 0); 
  analogWrite(MOTORA2, 238); 
  analogWrite(MOTORB1, 200); 
  analogWrite(MOTORB2, 0); 

  unsigned long stopStartTime = millis(); 

  while (millis() - stopStartTime < 5000){ 

  } 

} 


void TurnL(){ 

 

  analogWrite(MOTORA1, 250); 

  analogWrite(MOTORA2, 0); 

  analogWrite(MOTORB1, 250); 

  analogWrite(MOTORB2, 0); 

 

  unsigned long stopStartTime = millis(); 

  while (millis() - stopStartTime < 500){ 

     

  } 

} 

void TurnR(){ 

 

  analogWrite(MOTORA1, 0); 

  analogWrite(MOTORA2, 250); 

  analogWrite(MOTORB1, 0); 

  analogWrite(MOTORB2, 250); 

 

  unsigned long stopStartTime = millis(); 

  while (millis() - stopStartTime < 500){ 

 

  } 

} 

void stop(){ 

 

  analogWrite(MOTORA1, 0); 
  analogWrite(MOTORA2, 0); 
  analogWrite(MOTORB1, 0); 
  analogWrite(MOTORB2, 0); 

 

  unsigned long stopStartTime = millis(); 

  while (millis() - stopStartTime < 1500){ 

 

  } 

}
