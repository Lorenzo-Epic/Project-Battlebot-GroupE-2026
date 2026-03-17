// Pin mapping.
// ---------------- Servo / Gripper ----------------
#define SERVO_PIN 11 
#define GRIPPER_OPEN_US  1820
#define GRIPPER_CLOSE_US 1000

// ---------------- Motors ----------------
#define LEFT_FORWARD_PIN 5
#define LEFT_BACKWARD_PIN 10 
#define RIGHT_FORWARD_PIN 6
#define RIGHT_BACKWARD_PIN 9

// Motor PWM calibration
#define PWM_LEFT_FWD 255
#define PWM_LEFT_BWD 255
#define PWM_RIGHT_FWD 238
#define PWM_RIGHT_BWD 210

///Ultrasound sensors
#define ULTRASOUND_FORWARD_TRIG 12
#define ULTRASOUND_FORWARD_ECHO 13

#define ULTRASOUND_LEFT_TRIG 4
#define ULTRASOUND_LEFT_ECHO 7

#define ULTRASOUND_RIGHT_TRIG A0
#define ULTRASOUND_RIGHT_ECHO 8

// ---------------- Encoders ----------------
#define ROTATION_LEFT_PIN 2
#define ROTATION_RIGHT_PIN 3

// Ignore encoder edges that arrive too quickly.
const unsigned long EDGE_MIN_US = 150;

const unsigned long ULTRASOUND_TIMEOUT_US = 25000UL;



void setup() {
  Serial.begin(9600);

  pinMode(ULTRASOUND_FORWARD_TRIG, OUTPUT);
  pinMode(ULTRASOUND_LEFT_TRIG, OUTPUT);
  pinMode(ULTRASOUND_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRASOUND_FORWARD_ECHO, INPUT);
  pinMode(ULTRASOUND_LEFT_ECHO, INPUT);
  pinMode(ULTRASOUND_RIGHT_ECHO, INPUT);

  
  // Ensure triggers start low
  digitalWrite(ULTRASOUND_FORWARD_TRIG, LOW);
  digitalWrite(ULTRASOUND_LEFT_TRIG, LOW);
  digitalWrite(ULTRASOUND_RIGHT_TRIG, LOW);
}

// Send a 10 us trigger pulse and measure echo high time.
float getUltrasoundDuration(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  return pulseIn(echoPin, HIGH, ULTRASOUND_TIMEOUT_US);
}

float getUltrasoundDistance(int trigPin, int echoPin) {
  // 0.0343 cm/us is speed of sound in air. Divide by 2 for round trip.
  float ultrasoundDuration = getUltrasoundDuration(trigPin, echoPin);
  float ultrasoundDistance = (ultrasoundDuration * 0.0343f) / 2.0f;

  Serial.print("Distance: ");
  Serial.println(ultrasoundDistance);

  return ultrasoundDistance;
}

void loop() {
  Serial.print("Forward sensor ");
  getUltrasoundDistance(ULTRASOUND_FORWARD_TRIG, ULTRASOUND_FORWARD_ECHO);
  Serial.print("Left sensor ");
  getUltrasoundDistance(ULTRASOUND_LEFT_TRIG, ULTRASOUND_LEFT_ECHO);
  Serial.print("Right sensor ");
  getUltrasoundDistance(ULTRASOUND_RIGHT_TRIG, ULTRASOUND_RIGHT_ECHO);

  delay(3000);
}
