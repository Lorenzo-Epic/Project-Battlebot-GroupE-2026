// pin mapping
const int LEFT_FORWARD_PIN  = 7;
const int LEFT_BACKWARD_PIN = 10;
const int RIGHT_FORWARD_PIN = 11;
const int RIGHT_BACKWARD_PIN = 9;

// D1 = A0, D2 = A1, D3 = A2, D4 = A3, D5 = A4, D6 = A5, D7 = A6, D8 = A7
const int numberOfSensors = 7;
const int lineSensor[numberOfSensors] = {A1, A2, A3, A4, A5, A6, A7};
int lineValues[numberOfSensors];

// thresholds
const int LIGHT_SENSOR_WHITE_THRESHOLD = 400;
const int LIGHT_SENSOR_BLACK_THRESHOLD = 600;

// speed settings
int baseSpeed = 120;
int correctionSpeed = 30;
int recoverySpeed = 150;

// last black detection
bool lastBlackDetected = false;


// setup
void setup()
{
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  for (int i = 0; i < numberOfSensors; i++)
    pinMode(lineSensor[i], INPUT);

  stopMotors();
}


// loop
void loop()
{
  defaultLineSensor();
  delay(10);
}


// line sensor logic
void defaultLineSensor()
{
  int maxSensorValue = 0;

  // read sensors
  for (int i = 0; i < numberOfSensors; i++)
  {
    lineValues[i] = analogRead(lineSensor[i]);
    if (lineValues[i] > maxSensorValue)
      maxSensorValue = lineValues[i];
  }

  // robot sees black line
  if (maxSensorValue >= LIGHT_SENSOR_BLACK_THRESHOLD)
  {
    lastBlackDetected = true;

    // center sensor > forward
    if (lineValues[3] >= LIGHT_SENSOR_BLACK_THRESHOLD || lineValues[4] >= LIGHT_SENSOR_BLACK_THRESHOLD || lineValues[5] >= LIGHT_SENSOR_BLACK_THRESHOLD)
    {
      driveForward(baseSpeed + correctionSpeed, baseSpeed + correctionSpeed);
    }

    // right sensors > turn right
    else if (lineValues[1] >= LIGHT_SENSOR_BLACK_THRESHOLD || lineValues[2] >= LIGHT_SENSOR_BLACK_THRESHOLD)
    {
      driveRight(baseSpeed + 40, baseSpeed + 50);
    }

    // left sensors > turn left
    else if (lineValues[6] >= LIGHT_SENSOR_BLACK_THRESHOLD || lineValues[7] >= LIGHT_SENSOR_BLACK_THRESHOLD)
    {
      driveLeft(baseSpeed + 50, baseSpeed + 40);
    }
  }

  // line completely lost (all white)
  else if (maxSensorValue <= LIGHT_SENSOR_WHITE_THRESHOLD && lastBlackDetected)
  {
    driveBackward(recoverySpeed, recoverySpeed);
  }

  // uncertain area
  else
  {
    stopMotors();
  }
}


// motor control
void driveForward(int left, int right)
{
  analogWrite(LEFT_FORWARD_PIN, left);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, right);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}

void driveBackward(int left, int right)
{
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, left);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, right);
}

void driveRight(int left, int right)
{
  analogWrite(LEFT_FORWARD_PIN, left);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, right);
}

void driveLeft(int left, int right)
{
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, left);
  analogWrite(RIGHT_FORWARD_PIN, right);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}

void stopMotors()
{
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}
