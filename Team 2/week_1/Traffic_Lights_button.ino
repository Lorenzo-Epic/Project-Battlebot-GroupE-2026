
const int redPin = 13;
const int greenPin = 12;
const int yellowPin = 11;
const int button1 = 2;



void setup() {
  // put your setup code here, to run once:
  pinMode (redPin, OUTPUT);
  pinMode (greenPin, OUTPUT);
  pinMode (yellowPin, OUTPUT);

  
  
  pinMode (button1, INPUT_PULLUP);

  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, HIGH);
  digitalWrite(yellowPin, HIGH);
}

void loop() {

  digitalWrite(redPin, LOW);

  if (digitalRead(button1) == LOW) {   

    delay(30);
    if (digitalRead(button1) == LOW) { 

      delay(1000);
      digitalWrite(redPin, HIGH);

      delay(1000);                 
      digitalWrite(greenPin, LOW);
      delay(3000);                
      digitalWrite(greenPin, HIGH);

                       
      digitalWrite(yellowPin, LOW);
      delay(1000);                 
      digitalWrite(yellowPin, HIGH);

      digitalWrite(redPin, LOW  );

      
      while (digitalRead(button1) == LOW) {
        delay(10);
      }
    }
  }
}
