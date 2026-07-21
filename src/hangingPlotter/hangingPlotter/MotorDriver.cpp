#include <sys/types.h>
#include <Arduino.h>

// How to use:
// 1. Queue Rotation
// 2. ProcessRotation and check if IsDoneRotating in loop
// 4. Repeat

// For 28BYJ-48 motor: https://www.mouser.com/datasheet/2/758/stepd-01-data-sheet-1143075.pdf
class MotorDriver
{
  private:
    int pins[4] = {0, 1, 2, 3};
    int currentStep = 0;
    const double stepSize = 5.625/64; // degrees per step
    const int motorSeq[8] = {0b01000, 0b01100, 0b00100, 0b00110, 0b00010, 0b00011, 0b00001, 0b01001};
    double clockDelayMs;

    // Queued rotation variables
    unsigned long queuedStartTime = 0; // in millis
    int stepsRemaining = 0;
    int stepsTaken = 0;
    unsigned long msPerStep = 0;
    bool steppingClockwise = true;

    void WriteToPins()
    {
      digitalWrite(pins[0], motorSeq[currentStep] & 1);
      digitalWrite(pins[1], motorSeq[currentStep] & 2);
      digitalWrite(pins[2], motorSeq[currentStep] & 4);
      digitalWrite(pins[3], motorSeq[currentStep] & 8);
    }

  public:
    MotorDriver(int pins[])
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

    // Positive rotateDegrees is counterclockwise rotation
    void QueueRotation(double rotateDegrees, double degreesPerSecond)
    {
      Serial.println("asdf");
      this->queuedStartTime = millis();
      this->steppingClockwise = rotateDegrees < 0;
      rotateDegrees = rotateDegrees > 0 ? rotateDegrees : -rotateDegrees;
      this->stepsRemaining = rotateDegrees / this->stepSize;
      Serial.println(this->stepsRemaining);

      this->stepsTaken = 0;
      this->msPerStep = 1 / degreesPerSecond * this->stepSize * 1000; // If this is less than the time delay between calls to processRotation, then it will move as quick as possible
    }

    bool IsDoneRotating()
    {
      return stepsRemaining <= 0;
    }

    int GetStepsRemaining()
    {
      return stepsRemaining;
    }

    void ProcessRotation()
    {
      if( this->stepsRemaining <= 0 )
      {
        return;
      }

      if(queuedStartTime + msPerStep * stepsTaken <= millis())
      {
        // Serial.print("process rotation steps remaining");
        // Serial.println(this->stepsRemaining);
        this->stepsRemaining -= 1;
        this->stepsTaken += 1;
        this->DoStep(this->steppingClockwise);
      }
    }

    void DoStep(bool isClockwise)
    {
      if(isClockwise)
      {
        currentStep -= 1;
        if(currentStep < 0 )
        {
          currentStep += 8;
        }
        
      }
      else
      {
        currentStep += 1;
        if(currentStep >=8 )
        {
          currentStep -= 8;
        }
      }
      WriteToPins();
    }
};