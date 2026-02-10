//initialising constants for LEDs
const int ledRed = 12; //red
const int ledYellow = 11; //yellow
const int ledGreen = 10; //green

//initialising constants for buttonpins
const int switch1 = 9;

//setting up delay times
const int shortDelay = 1000; //1 second delay from red to green and from yellow to red
const int longDelay = 3000; //3 second delay from green to yellow

void setup() {
  //setting up the LEDs as outputs
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);

  //setting up the button pin
  pinMode(switch1, INPUT);
}

void loop() {

  digitalWrite(ledYellow, HIGH);
  digitalWrite(ledGreen, HIGH);

  if (digitalRead(switch1) == LOW) {

    delay(shortDelay);

    digitalWrite(ledRed, HIGH);
    digitalWrite(ledYellow, HIGH);
    digitalWrite(ledGreen, LOW);

    delay(longDelay);

    digitalWrite(ledRed, HIGH);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, HIGH);

    delay(shortDelay);

    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, HIGH);
    digitalWrite(ledGreen, HIGH);
  }
}