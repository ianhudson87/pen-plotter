// This Arduino example demonstrates bidirectional operation of a
// 28BYJ-48, using a ULN2003 interface board to drive the stepper.
// The 28BYJ-48 motor is a 4-phase, 8-beat motor, geared down by
// a factor of 68. One bipolar winding is on motor pins 1 & 3 and
// the other on motor pins 2 & 4. The step angle is 5.625/64 and the
// operating Frequency is 100pps. Current draw is 92mA.
////////////////////////////////////////////////

bool started = false;
double clockDelayMs = 5;
int shoulderMotorPins[4] = {13,12,11,10};
int elbowMotorPins[4] = {9,8,7,6};
int wristMotorPins[4] = {5,4,3,2};
Motor shoulderMotor(shoulderMotorPins, clockDelayMs);
Motor elbowMotor(elbowMotorPins, clockDelayMs);
Motor wristMotor(wristMotorPins, clockDelayMs);
AngleSolver angleSolver(12.9, 18.7);

double startingX = 13.7;
double startingY = 13.5;
double targetX = 25;
double targetY = 7;

//////////////////////////////////////////////////////////////////////////////
void setup() {
  double startingShoulder = angleSolver.GetShoulderAngle(startingX, startingY);
  double startingElbow = angleSolver.GetElbowAngle(startingX, startingY);

  double targetShoulder = angleSolver.GetShoulderAngle(targetX, targetY);
  double targetElbow = angleSolver.GetElbowAngle(targetX, targetY);

  Serial.println("start: " + String(startingShoulder) + "," + String(startingElbow));
  Serial.println("target: " + String(targetShoulder) + "," + String(targetElbow));


  shoulderMotor.QueueRotation(targetShoulder - startingShoulder, 2);
  elbowMotor.QueueRotation(startingElbow - targetElbow, 2);
  wristMotor.QueueRotation(false, 180, 1);

  shoulderMotor.QueueRotation(0, 2);
  elbowMotor.QueueRotation(90, 2);

  Serial.begin(9600);
  Serial.println("stepper");
}

//////////////////////////////////////////////////////////////////////////////
void loop(){
  if (!started && Serial.available() == 0) {
    return;
  }
  else
  {
    started = true;
    // read the incoming byte:
    int incomingByte = Serial.read();
  }

  int startTime = millis();
  
  // shoulderMotor.ProcessRotation();
  // elbowMotor.ProcessRotation();
  wristMotor.ProcessRotation();

  // double startingShoulder = angleSolver.GetShoulderAngle(startingX, startingY);
  // double startingElbow = angleSolver.GetElbowAngle(startingX, startingY);

  // double targetShoulder = angleSolver.GetShoulderAngle(targetX, targetY);
  // double targetElbow = angleSolver.GetElbowAngle(targetX, targetY);

  // Serial.println("start: " + String(startingShoulder) + "," + String(startingElbow));
  // Serial.println("target: " + String(targetShoulder) + "," + String(targetElbow));

  int endTime = millis();
  int deltaTime = endTime - startTime;
  if(deltaTime < clockDelayMs)
  {
    delay(clockDelayMs - deltaTime);
  }
  else
  {
    Serial.println("missed timing by" + String(deltaTime - clockDelayMs, 10));
  }
}

// //////////////////////////////////////////////////////////////////////////////
// //set pins to ULN2003 high in sequence from 1 to 4
// //delay "motorSpeed" between each pin setting (to determine speed)
// void anticlockwise()
// {
//   for(int i = 0; i < 8; i++)
//   {
//     setOutput(i);
//     delay(motorSpeed);
//   }
// }

// void clockwise()
// {
//   for(int i = 7; i >= 0; i--)
//   {
//     setOutput(i);
//     delay(motorSpeed);
//   }
// }

// void setOutput(int out)
// {
//   digitalWrite(motorPin1, bitRead(lookup[out], 0));
//   digitalWrite(motorPin2, bitRead(lookup[out], 1));
//   digitalWrite(motorPin3, bitRead(lookup[out], 2));
//   digitalWrite(motorPin4, bitRead(lookup[out], 3));
// }