#include <Servo.h>
#include <Arduino.h>

struct Tuple {
 double x;
 double y;
};

bool finishedPlotting = false;
bool started = false;

int buttonPin = 14;
int buttonState = 0;

Servo elbowServo;
Servo shoulderServo;

int shoulderPin = 2;
int elbowPin = 3;

double currentX = 20;
double currentY = 0;
double targetX;
double targetY;
double intermediateTargetX = currentX;
double intermediateTargetY = currentY;

double forearmLen = 12.3;
double bicepLen = 12.4; // TODO



double shoulderAngle;
double elbowAngle;

double shoulderAngleSpeed; // degreesPerSecond
double elbowAngleSpeed;

int currentTargetIndex = 0;
int maxTargetIndex = 19;
// double xTargetArray[] = {5, 10, 5, 5, 10, 10, 20};
// double yTargetArray[] = {0, 5, 5, 10, 10, 5, 0};
double xTargetArray[] = {7.5, 7.5, 2.5, 5, 6, 7.5, 6, 7.5, 12, 10, 8, 7.5, 9, 7.5, 14, 8, 10, 15, 14, 20};
double yTargetArray[] = {0, 2.5, 7.5, 10, 7.5, 9, 11, 12.5, 7.5, 6, 7.5, 6, 5, 2.5, 10, 13, 14, 11, 10, 0};

int clockDelay = 10;

double intermediateStepSize = 1; // centimeters
int intermediateStepTime = 1000; // milliseconds

int cyclesRemaining = 0; // steps to get to intermediateTarget

int i = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  elbowServo.attach(elbowPin);
  shoulderServo.attach(shoulderPin);

  shoulderAngle = GetShoulderAngle(currentX, currentY);
  elbowAngle = GetElbowAngle(currentX, currentY);

  WriteMotorAngles(shoulderAngle, elbowAngle);

  targetX = xTargetArray[currentTargetIndex];
  targetY = yTargetArray[currentTargetIndex];


  // Tuple intermediateTarget = GetIntermediateTarget(currentX, currentY, targetX, targetY);
  // intermediateTargetX = intermediateTarget.x;
  // intermediateTargetY = intermediateTarget.y;
  pinMode(buttonPin, INPUT); // button
}

void loop() {

  int startTime = millis();

  if (!started && digitalRead(buttonPin) == 0) {
    return;
  }
  else if (!started && digitalRead(buttonPin) == 1)
  {
    delay(3000);
    started = true;
  }

  if(finishedPlotting) { return; }

  if(cyclesRemaining <= 0) // reached intermediate target
  {
    Serial.println("reached intermediate");
    // set next target if needed
    if(intermediateTargetX == targetX && intermediateTargetY == targetY)
    {
      currentTargetIndex++;

      if(currentTargetIndex > maxTargetIndex)
      {
        finishedPlotting = true;
        return;
      }
      targetX = xTargetArray[currentTargetIndex];
      targetY = yTargetArray[currentTargetIndex];

      Serial.println("setting new target: " + String(targetX) +  "," + String(targetY));
    }
    
    // set next intermediate
    currentX = intermediateTargetX;
    currentY = intermediateTargetY;

    Tuple intermediateTarget = GetIntermediateTarget(currentX, currentY, targetX, targetY);
    intermediateTargetX = intermediateTarget.x;
    intermediateTargetY = intermediateTarget.y;

    Serial.println("setting new intermediate: " + String(intermediateTargetX) + "," + String(intermediateTargetY));

    // set motor speed and number of remaining steps
    shoulderAngle = GetShoulderAngle(currentX, currentY);
    elbowAngle = GetElbowAngle(currentX, currentY);
    
    double intermediateTargetShoulderAngle = GetShoulderAngle(intermediateTargetX, intermediateTargetY);
    double intermediateTargetElbowAngle = GetElbowAngle(intermediateTargetX, intermediateTargetY);

    shoulderAngleSpeed = (intermediateTargetShoulderAngle - shoulderAngle) / (double)intermediateStepTime;
    elbowAngleSpeed = (intermediateTargetElbowAngle - elbowAngle) / (double)intermediateStepTime;

    // Serial.println("speeds: " + String(elbowAngleSpeed, 10) + "," + String(shoulderAngleSpeed, 10));

    cyclesRemaining = intermediateStepTime / clockDelay;
  }
  // move towards intermediate
  elbowAngle += elbowAngleSpeed * clockDelay;
  shoulderAngle += shoulderAngleSpeed * clockDelay;
  WriteMotorAngles(shoulderAngle, elbowAngle);


  cyclesRemaining--;


  int endTime = millis();
  int deltaTime = endTime - startTime;
  if(deltaTime < clockDelay)
  {
    delay(clockDelay - deltaTime);
  }
  else
  {
    Serial.println("missed timing by" + String(deltaTime - clockDelay, 10));
  }
}

double GetShoulderAngle(double x, double y)
{
  double hypSq = pow(x, 2) + pow(y, 2);
  double hyp = sqrt(hypSq);
  double beta = acos((pow(forearmLen,2) - pow(bicepLen,2) - hypSq) / (-2 * bicepLen * hyp));
  double alpha = atan(x/y);
  double result = (alpha + beta) / M_PI * 180;
  // Serial.println("Solved shoulder angle (" + String(x) + ", " + String(y) + ") --> " + String(result) + " degrees");
  return 180 - result;
}

double GetElbowAngle(double x, double y)
{
  double hypSq = pow(x, 2) + pow(y, 2);
  double phi = acos((hypSq - pow(forearmLen,2) - pow(bicepLen,2)) / (-2 * forearmLen * bicepLen));
  double result = phi / M_PI * 180;
  // Serial.println("Solved elbow angle (" + String(x) + ", " + String(y) + ") --> " + String(result) + " degrees");
  return result;
}

void WriteMotorAngles(double shoulderAngle, double elbowAngle)
{
  elbowServo.write(elbowAngle);
  shoulderServo.write(shoulderAngle);
}

double GetDist(double x1, double y1, double x2, double y2)
{
  double result = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
  return result;
}

Tuple GetDirection(double x1, double y1, double x2, double y2, double multiplier)
{
  double distance = GetDist(x1, y1, x2, y2);
  double xDir = (x2 - x1) / distance * multiplier;
  double yDir = (y2 - y1) / distance * multiplier;
  Tuple result = {xDir, yDir};
  return result;
}

Tuple GetIntermediateTarget(double currentX, double currentY, double targetX, double targetY)
{
  Serial.println("calculating intermediate from " + String(currentX) + "," + String(currentX) + " to " + String(targetX) + "," + String(targetY));
  // if distance between target and current is less than intermediate step distance, return target x and y
  if(GetDist(currentX, currentY, targetX, targetY) <= intermediateStepSize)
  {
    Tuple result = {targetX, targetY};
    return result;
  }
  else
  {
    // get direction, then normalize and multiply by intermediateStepSize
    Tuple stepVector = GetDirection(currentX, currentY, targetX, targetY, intermediateStepSize);
    Tuple result = {currentX + stepVector.x, currentY + stepVector.y};
    return result;
  }
}
