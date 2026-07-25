#include <MotorDriver.h>

void MotorDriver::WriteToPins()
{
  digitalWrite(pins[0], motorSeq[currentStep] & 1);
  digitalWrite(pins[1], motorSeq[currentStep] & 2);
  digitalWrite(pins[2], motorSeq[currentStep] & 4);
  digitalWrite(pins[3], motorSeq[currentStep] & 8);
}

MotorDriver::MotorDriver(int pins[])
{
  this->pins[0] = pins[0];
  this->pins[1] = pins[1];
  this->pins[2] = pins[2];
  this->pins[3] = pins[3];
  pinMode(this->pins[0], OUTPUT);
  pinMode(this->pins[1], OUTPUT);
  pinMode(this->pins[2], OUTPUT);
  pinMode(this->pins[3], OUTPUT);
}

void MotorDriver::QueueRotation(double rotateDegrees, double degreesPerSecond)
{
  this->queuedStartTime = millis();
  this->steppingClockwise = rotateDegrees < 0;
  rotateDegrees = rotateDegrees > 0 ? rotateDegrees : -rotateDegrees;
  this->stepsRemaining = rotateDegrees / this->stepSize;

  this->stepsTaken = 0;
  this->msPerStep = 1 / degreesPerSecond * this->stepSize * 1000;
}

bool MotorDriver::IsDoneRotating() const
{
  return stepsRemaining <= 0;
}

int MotorDriver::GetStepsRemaining() const
{
  return stepsRemaining;
}

void MotorDriver::ProcessRotation()
{
  if (this->stepsRemaining <= 0)
  {
    return;
  }

  if (queuedStartTime + msPerStep * stepsTaken <= millis())
  {
    this->stepsRemaining -= 1;
    this->stepsTaken += 1;
    this->DoStep(this->steppingClockwise);
  }
}

void MotorDriver::DoStep(bool isClockwise)
{
  if (isClockwise)
  {
    currentStep -= 1;
    if (currentStep < 0)
    {
      currentStep += 8;
    }
  }
  else
  {
    currentStep += 1;
    if (currentStep >= 8)
    {
      currentStep -= 8;
    }
  }

  WriteToPins();
}
