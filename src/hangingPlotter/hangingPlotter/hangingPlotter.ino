#include "MotorDriver.cpp"
#include "Planner.cpp"
#include "Coordinates.cpp"

// DATAMODELS
enum PlotterState {
  Lowering,
  LeftRetracting,
  RightRetracting,
  Calculating,
  Moving
};
// END DATAMODELS

// CONSTANTS
int leftMotorPins[4] = {13, 12, 14, 27};
int rightMotorPins[4] = {26, 25, 33, 32};

int stateChangeButtonPin = 35;
int retractButtonPin = 34;

float d_betweenMotors = 17.2; // distance between motors
float d_betweenConnections = 2.75; // distance between connection points
float radius_Motor = 1.435;
float circum_Motor = radius_Motor * TWO_PI;

float stringLengthStartingVal = 10.0;
float plotterHeadStartingX = 8.625;
float plotterHeadStartingY = 6.88;

float motorSpeed = 10; // degrees per seconds

MotorDriver leftMotor(leftMotorPins);
MotorDriver rightMotor(rightMotorPins);
// END CONSTANTS

// RUNTIME VARIABLES
float d_leftString = stringLengthStartingVal; // current distance between left motor and left connection
float d_rightString = stringLengthStartingVal; // current distance between right motor and right connection

enum PlotterState plotterStateMachine = Lowering;



float xPos = plotterHeadStartingX;
float yPos = plotterHeadStartingY;

int prevStateChangeButtonVal = 0;
// END RUNTIME VARIABLES

void setup() {
  Serial.begin(115200);

  pinMode(stateChangeButtonPin, INPUT);
  pinMode(retractButtonPin, INPUT);

}

void loop() {
  // Serial.print("state machine: ");
  // Serial.println(plotterStateMachine);
  bool currentStateChangeButtonVal = digitalRead(stateChangeButtonPin);

  if(plotterStateMachine == Lowering)
  {
    if(currentStateChangeButtonVal == HIGH && prevStateChangeButtonVal == LOW)
    {
      plotterStateMachine = LeftRetracting;
    }
    // rightMotor.ProcessRotation();
    // leftMotor.ProcessRotation();
    rightMotor.DoStep(false); // counter-clockwise
    leftMotor.DoStep(true); // clockwise
    delay(5);
  }
  else if(plotterStateMachine == LeftRetracting)
  {
    if(currentStateChangeButtonVal == HIGH && prevStateChangeButtonVal == LOW)
    {
      plotterStateMachine = RightRetracting;
    }
    if(digitalRead(retractButtonPin) == HIGH)
    {
      leftMotor.DoStep(false);
      delay(5);
    }
  }
  else if(plotterStateMachine == RightRetracting)
  {
    if(currentStateChangeButtonVal == HIGH && prevStateChangeButtonVal == LOW)
    {
      plotterStateMachine = Calculating;
    }
    if(digitalRead(retractButtonPin) == HIGH)
    {
      rightMotor.DoStep(true);
      delay(5);
    }
  }
  else if(plotterStateMachine == Calculating)
  {
    Serial.print("Spiral iteration: ");
    Serial.println(spiralIterations);
    float distanceChange = 0.5 * (spiralIterations / 2 + 1);
    switch(spiralIterations % 4)
    {
      case 0:
        xPos += distanceChange;
        break;
      case 1:
        yPos -= distanceChange;
        break;
      case 2:
        xPos -= distanceChange;
        break;
      case 3:
        yPos += distanceChange;
        break;
    }
    spiralIterations++;
    Serial.print("currentLengths: ");
    Serial.print(d_leftString);
    Serial.print(", ");
    Serial.println(d_rightString);
    Serial.print("target coords: ");
    Serial.print(xPos);
    Serial.print(", ");
    Serial.println(yPos);
    Coordinates targetLengths = GetTargetLengths(xPos, yPos);
    float leftMotorRotation = (d_leftString - targetLengths.x) / circum_Motor * 360.0f;
    float rightMotorRotation = (d_rightString - targetLengths.y) / circum_Motor * -360.0f;
    Serial.print("rotation degrees: ");
    Serial.print(leftMotorRotation);
    Serial.print(", ");
    Serial.println(rightMotorRotation);
    d_leftString = targetLengths.x;
    d_rightString = targetLengths.y;

    float leftMotorSpeed = abs(leftMotorRotation) > abs(rightMotorRotation) ? motorSpeed : motorSpeed * abs(leftMotorRotation / rightMotorRotation);
    float rightMotorSpeed = abs(rightMotorRotation) > abs(leftMotorRotation) ? motorSpeed : motorSpeed * abs(rightMotorRotation / leftMotorRotation);

    leftMotor.QueueRotation(leftMotorRotation, leftMotorSpeed);
    rightMotor.QueueRotation(rightMotorRotation, rightMotorSpeed);

    Serial.print("rotation speeds: ");
    Serial.print(leftMotorSpeed);
    Serial.print(", ");
    Serial.println(rightMotorSpeed);

    plotterStateMachine = Moving;
  }
  else if(plotterStateMachine == Moving)
  {
    if(leftMotor.IsDoneRotating() && rightMotor.IsDoneRotating())
    {
      plotterStateMachine = Calculating;
    }

    if(currentStateChangeButtonVal == HIGH && prevStateChangeButtonVal == LOW)
    {
      plotterStateMachine = Lowering;
      d_leftString = stringLengthStartingVal; // current distance between left motor and left connection
      d_rightString = stringLengthStartingVal; // current distance between right motor and right connection
      spiralIterations = 0;
      xPos = plotterHeadStartingX;
      yPos = plotterHeadStartingY;
    }

    rightMotor.ProcessRotation();
    leftMotor.ProcessRotation();
  }

  prevStateChangeButtonVal = currentStateChangeButtonVal;
}

Coordinates GetTargetLengths(float x, float y)
{
  float targetLeftStringLength = sqrt(sq(x - d_betweenConnections / 2) + sq(y)); // distance from left motor to left connection point
  float targetRightStringLength = sqrt(sq(d_betweenMotors - d_betweenConnections / 2 - x) + sq(y)); // distance from right motor to right connection point
  Coordinates result = {targetLeftStringLength, targetRightStringLength};
  Serial.print("target lengths: ");
  Serial.print(targetLeftStringLength);
  Serial.print(", ");
  Serial.println(targetRightStringLength);
  return result;
}









