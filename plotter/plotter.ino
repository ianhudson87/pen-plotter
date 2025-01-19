// This Arduino example demonstrates bidirectional operation of a
// 28BYJ-48, using a ULN2003 interface board to drive the stepper.
// The 28BYJ-48 motor is a 4-phase, 8-beat motor, geared down by
// a factor of 68. One bipolar winding is on motor pins 1 & 3 and
// the other on motor pins 2 & 4. The step angle is 5.625/64 and the
// operating Frequency is 100pps. Current draw is 92mA.
////////////////////////////////////////////////
#include "MovementPlanner.cpp"
#include <Vector.h>

bool started = false;
int buttonPin = 14;
int buttonState = 0;
double clockDelayMs = 5;
int shoulderMotorPins[4] = {13,12,11,10};
int elbowMotorPins[4] = {9,8,7,6};
int wristMotorPins[4] = {5,4,3,2};
Motor shoulderMotor(shoulderMotorPins, clockDelayMs);
Motor elbowMotor(elbowMotorPins, clockDelayMs);
Motor wristMotor(wristMotorPins, clockDelayMs);
AngleSolver angleSolver(12.9, 18.7);

Coordinates c1(9,0);
Coordinates c2(9,10);
Coordinates c3(9,5);
Coordinates c4(12,5);
Coordinates c5(12,10);
Coordinates c6(12,0);
Coordinates c7(19,0);
Coordinates c8(17,0);
Coordinates c9(17,10);
Coordinates c10(15,10);
Coordinates c11(19,10);
Coordinates c12(23,0);
CoordinatesQueue* coordinatesQueue;

Coordinates* currentCoords = new Coordinates(22.5,0);
MovementPlanner* movementPlanner;

bool plannedMovement = true;

// double startingX = 13.7;
// double startingY = 13.5;
// double targetX = 25;
// double targetY = 7;

//////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  Serial.println("setup");
  coordinatesQueue = new CoordinatesQueue();
  coordinatesQueue->QueueCoords(&c1);
  coordinatesQueue->QueueCoords(&c2);
  coordinatesQueue->QueueCoords(&c3);
  coordinatesQueue->QueueCoords(&c4);
  coordinatesQueue->QueueCoords(&c5);
  coordinatesQueue->QueueCoords(&c6);
  coordinatesQueue->QueueCoords(&c7);
  coordinatesQueue->QueueCoords(&c8);
  coordinatesQueue->QueueCoords(&c9);
  coordinatesQueue->QueueCoords(&c10);
  coordinatesQueue->QueueCoords(&c11);
  coordinatesQueue->QueueCoords(&c12);




  movementPlanner = new MovementPlanner(shoulderMotor, elbowMotor, wristMotor, angleSolver, coordinatesQueue, currentCoords, 10);


  // Coordinates* test = coordinatesQueue->DequeueCoords();
  // Serial.println("(" + String(test->x) + ", " + String(test->y) + ")");
  // test = coordinatesQueue->DequeueCoords();
  // Serial.println("(" + String(test->x) + ", " + String(test->y) + ")");
  // test = coordinatesQueue->DequeueCoords();
  // Serial.println("(" + String(test->x) + ", " + String(test->y) + ")");
  // test = coordinatesQueue->DequeueCoords();
  // Serial.println("(" + String(test->x) + ", " + String(test->y) + ")");
  // test = coordinatesQueue->DequeueCoords();
  // Serial.println("(" + String(test->x) + ", " + String(test->y) + ")");

  
  // double startingShoulder = angleSolver.GetShoulderAngle(startingX, startingY);
  // double startingElbow = angleSolver.GetElbowAngle(startingX, startingY);

  // double targetShoulder = angleSolver.GetShoulderAngle(targetX, targetY);
  // double targetElbow = angleSolver.GetElbowAngle(targetX, targetY);

  // Serial.println("start: " + String(startingShoulder) + "," + String(startingElbow));
  // Serial.println("target: " + String(targetShoulder) + "," + String(targetElbow));


  // shoulderMotor.QueueRotation(targetShoulder - startingShoulder, 2);
  // elbowMotor.QueueRotation(startingElbow - targetElbow, 2);
  // wristMotor.QueueRotation(false, 180, 1);

  shoulderMotor.QueueRotation(10, 15);
  elbowMotor.QueueRotation(0, 10);

  // Coordinates test2 = {1,1};
  // int storage_array[5];
  // Vector<int> coordinatesTest(storage_array);
  // coordinatesTest.push_back(5);
  // int test3 = coordinatesTest.front();
  // Serial.println("(" + String(test.x) + ", " + String(test.y) + ")");
  // Serial.println("(" + String(test2.x) + ", " + String(test2.y) + ")");
  // Serial.println("(" + String(test3) + ", " + String(test3) + ")");

  pinMode(buttonPin, INPUT); // button
}

//////////////////////////////////////////////////////////////////////////////
void loop(){
  if (!started && digitalRead(buttonPin) == 0) {
    return;
  }
  else
  {
    started = true;
    // read the incoming byte:
  }

  int startTime = millis();

  if(plannedMovement)
  {
    movementPlanner->ProcessMovement();
  }
  else
  {
    shoulderMotor.ProcessRotation();
    elbowMotor.ProcessRotation();
  }
  
  // shoulderMotor.ProcessRotation();
  // elbowMotor.ProcessRotation();
  // wristMotor.ProcessRotation();

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