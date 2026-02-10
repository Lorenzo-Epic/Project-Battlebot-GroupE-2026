//initialising constants for LEDs
const int ledRed = 12; //red
const int ledYellow = 11; //yellow
const int ledGreen = 10; //green

//initialising constants for buttonpins
const int switch1 = 9;
const int switch2 = 7;

//unsigned -> can never be negative, long -> data type to store numbers larger than an int
//unsigned long -> a number larger than int that can never be negative
unsigned long delayTime = 1000; //delayTime if we don't click anything
const int longTime = 2000; //2 second delay
const int shortTime = 500; //0.5 second delay
unsigned long timeOfLastToggle = 0; //by default, timeOfLastToggle is 0
bool ledOn = false; //by default, the LEDs are off

void setup() {
  //setting up the LEDs as outputs
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);

  //setting up the button pins
  //INPUT_PULLUP -> default reading: HIGH, button pushed -> LOW
  pinMode(switch1, INPUT);
  pinMode(switch2, INPUT);
}

void loop() {

  //due to buttonPin2 not activating, it now has most priority as it is on top
  //digitalRead() == LOW, because pushing a button in mode INPUT_PULLUP sets it to LOW
  if (digitalRead(switch2) == LOW) {
    delayTime = longTime;
  }
  if (digitalRead(switch1) == LOW) {
    delayTime = shortTime;
  }

  //millis: the time (in ms) passed since the board started running the current program
  //if how long it has been since the LEDs were toggled is at least the current delayTime
  if (millis() - timeOfLastToggle >= delayTime) {
    timeOfLastToggle = millis(); //current time becomes the new reference for next delayTime
    ledOn = !ledOn;

    digitalWrite(ledRed, ledOn); //if ledOn is true -> digitalRead(ledRed, HIGH), if ledOn is false -> digitalRead(ledRed, LOW)
    digitalWrite(ledYellow, ledOn);
    digitalWrite(ledGreen, ledOn);
  }
}