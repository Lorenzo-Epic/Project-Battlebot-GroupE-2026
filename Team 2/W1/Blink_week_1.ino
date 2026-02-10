const int button2 = 3;
const int button1 = 2;
const int ledPin = 13;
int blinkDelay = 1000;

void setup() {
  pinMode(ledPin, OUTPUT);
  
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  
}
void loop() {
  if(digitalRead(button1)==LOW ) {
    blinkDelay = 500;
    }
  if(digitalRead(button2)==LOW) {
    blinkDelay = 2000;
  }
  digitalWrite ( ledPin, HIGH );
  delay(blinkDelay);

  digitalWrite(ledPin, LOW);
  delay(blinkDelay);
  }
  
