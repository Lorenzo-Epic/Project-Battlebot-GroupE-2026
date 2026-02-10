// constants won't change. They're used here to set pin numbers:
const int buttonPin1 = 2;     //B1/S1
const int buttonPin2 = 4;     //B2/S2
const int buttonPin3 = 7;     //B3/S3
const int ledPin1 =  13;      //L1 Red/13
const int ledPin2 =  12;      //L2 Yellow/12
const int ledPin3 =  8;      //L3 Green/11

// variables will change:
int buttonState1 = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;
int buttonState3 = 0;

bool button1Press = false;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);

  //  initialize LED output
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, HIGH);
  digitalWrite(ledPin3, HIGH);

}

void loop() {
  // read the state of the pushbutton value:
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);
  buttonState3 = digitalRead(buttonPin3);


  if (buttonState1 == LOW) {
    delay(1000);
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin3, LOW);
    delay(3000);
    digitalWrite(ledPin3, HIGH);
    digitalWrite(ledPin2, LOW);
    delay(1000);
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin1, LOW);
  }
  
}
