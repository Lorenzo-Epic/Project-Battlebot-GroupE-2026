const int leftBackward = 3;
const int leftForward  = 5;

const int rightForward  = 6;
const int rightBackward = 9;

// rotation sensors (pins)
const int rotationLeft  = 10;
const int rotationRight = 11;

void setup() {
  Serial.begin(9600);          // or 115200, just match Serial Monitor
  // while (!Serial) {}        // only needed on some boards (e.g., Leonardo/Micro)

  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);

  pinMode(rotationLeft, INPUT_PULLUP);   // common for hall/encoder switches
  pinMode(rotationRight, INPUT_PULLUP);
}

void loop() {
  // If you want to print the PIN NUMBER:
//  Serial.print("rotationLeft pin = ");/
//  Serial.println(rotationLeft);/

  // If you want to print the SENSOR STATE on that pin:
  Serial.print("rotationLeft state = ");
  Serial.println(digitalRead(rotationLeft));

  delay(200);
}
