// constants won't change. They're used here to set pin numbers:
const int BUTTON_PIN_1 = 2;     //B1/S1  
const int BUTTON_PIN_2 = 4;     //B2/S2
const int BUTTON_PIN_3 = 7;     //B3/S3
const int LED_PIN_1 =  13;      //L1 Red/13
const int LED_PIN_2 =  12;      //L2 Yellow/12
const int LED_PIN_3 =  8;      //L3 Green/11

// variables will change:
int buttonState1 = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;
int buttonState3 = 0;

bool button1Press = false;

void setup() {
  // initialize the LED pin as an output:
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(BUTTON_PIN_1, INPUT);
  pinMode(BUTTON_PIN_2, INPUT);
  pinMode(BUTTON_PIN_3, INPUT);

  //  initialize LED output
  digitalWrite(LED_PIN_1, LOW);
  digitalWrite(LED_PIN_2, HIGH);
  digitalWrite(LED_PIN_3, HIGH);

}

void loop() {
  // read the state of the pushbutton value:
  buttonState1 = digitalRead(BUTTON_PIN_1);
  buttonState2 = digitalRead(BUTTON_PIN_2);
  buttonState3 = digitalRead(BUTTON_PIN_3);


  if (buttonState1 == LOW) {
    delay(1000);
    digitalWrite(LED_PIN_1, HIGH);
    digitalWrite(LED_PIN_3, LOW);
    delay(3000);
    digitalWrite(LED_PIN_3, HIGH);
    digitalWrite(LED_PIN_2, LOW);
    delay(1000);
    digitalWrite(LED_PIN_2, HIGH);
    digitalWrite(LED_PIN_1, LOW);
  }
  
}
