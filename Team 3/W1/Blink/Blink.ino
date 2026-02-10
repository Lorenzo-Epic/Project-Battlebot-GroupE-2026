// constants won't change. They're used here to set pin numbers:
const int buttonPin1 = 2;     // the number of the pushbutton pin
const int buttonPin2 = 4;
const int buttonPin3 = 7;
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState1 = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;
int buttonState3 = 0;

bool fastBlink = false;
bool slowBlink = false;

unsigned long timerOne = 0;
bool state;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);
  buttonState3 = digitalRead(buttonPin3);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState1 == LOW) {
    fastBlink = true;
    slowBlink = false;
  }
  if (buttonState2 == LOW) {
    slowBlink = true;
    fastBlink = false;
  }

  if (fastBlink == true) {
    blink(500); 
  } else if (slowBlink == true) {
    blink(2000);
  }
  
}

void blink (int INTERVAL) {
  if (millis() >= timerOne) {
    timerOne = millis() + INTERVAL;

    state = !state;
    digitalWrite(ledPin, state);
  }
}
