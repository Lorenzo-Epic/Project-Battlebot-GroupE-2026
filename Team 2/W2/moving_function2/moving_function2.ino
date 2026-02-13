const int RIGHTFORWARD_A1 = 6;  // right forward 
const int RIGHTBACK_A2 = 5;  // right back
const int LEFTBACK_B1 = 10; // left back
const int LEFTFORWARD_B2 = 11;  //left forward

void MoveForward();     // Move forwards 
void MoveBackward();    // Move backwards 
void TurnL();           // Turn on its axis 90 degrees Left 
void TurnR();           // Turn on its axis 90 degrees Right 
void stop();            // Stops  all the wheels 

 

void setup() { 
  pinMode(RIGHTFORWARD_A1, OUTPUT); 
  pinMode(RIGHTBACK_A2, OUTPUT); 
  pinMode(LEFTBACK_B1, OUTPUT); 
  pinMode(LEFTFORWARD_B2, OUTPUT); 
} 

 

void loop() { 
  MoveForward(); 
  stop(); 
  MoveBackward(); 
  stop();
  TurnL();
  stop(); 
  TurnR(); 
  stop();   
} 

void MoveForward(){ 
  analogWrite(RIGHTFORWARD_A1, 250); //right forward
  analogWrite(RIGHTBACK_A2, 0); //right back
  analogWrite(LEFTBACK_B1, 0); //left back
  analogWrite(LEFTFORWARD_B2, 195); //left forward

  unsigned long stopStartTime = millis(); 
  while (millis() - stopStartTime < 5000){ 
  } 
} 

void MoveBackward(){ 
  analogWrite(RIGHTFORWARD_A1, 0); 
  analogWrite(RIGHTBACK_A2, 245); 
  analogWrite(LEFTBACK_B1, 225); 
  analogWrite(LEFTFORWARD_B2, 0); 

  unsigned long stopStartTime = millis(); 
  while (millis() - stopStartTime < 5000){ 
  } 

} 


void TurnL(){ 
  analogWrite(RIGHTFORWARD_A1, 250); 
  analogWrite(RIGHTBACK_A2, 0); 
  analogWrite(LEFTBACK_B1, 0); 
  analogWrite(LEFTFORWARD_B2, 0); 

  unsigned long stopStartTime = millis(); 
  while (millis() - stopStartTime < 1100){ 
 } 

} 

void TurnR(){ 
  analogWrite(RIGHTFORWARD_A1, 0); 
  analogWrite(RIGHTBACK_A2, 0); 
  analogWrite(LEFTBACK_B1, 0); 
  analogWrite(LEFTFORWARD_B2, 250); 

 
  unsigned long stopStartTime = millis(); 
  while (millis() - stopStartTime < 800){ 

 

  } 

} 

void stop(){ 
  analogWrite(RIGHTFORWARD_A1, 0); 
  analogWrite(RIGHTBACK_A2, 0); 
  analogWrite(LEFTBACK_B1, 0); 
  analogWrite(LEFTFORWARD_B2, 0); 

  unsigned long stopStartTime = millis(); 
  while (millis() - stopStartTime < 1100){ 
  } 

}
