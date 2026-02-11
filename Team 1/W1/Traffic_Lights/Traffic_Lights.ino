//initialising constants for LEDs
const int LED_RED = 12; //red
const int LED_YELLOW = 11; //yellow
const int LED_GREEN = 10; //green

//initialising constants for buttonpins
const int SWITCH_1 = 9;

//setting up delay times
const int SHORT_DELAY = 1000; //1 second delay from red to green and from yellow to red
const int LONG_DELAY = 3000; //3 second delay from green to yellow

void setup() {
  //setting up the LEDs as outputs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  //setting up the button pin
  pinMode(SWITCH_1, INPUT);
}

void loop() {

  //turning off the yellow and green LEDs initially
  digitalWrite(LED_YELLOw, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  if (digitalRead(SWITCH_1) == LOW) { //if button is pressed

    delay(SHORT_DELAY);

    //GREEN
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_GREEN, LOW);

    delay(LONG_DELAY);

    //YELLOW
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_GREEN, HIGH);

    delay(SHORT_DELAY);

    //RED
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_GREE, HIGH);
  }
}