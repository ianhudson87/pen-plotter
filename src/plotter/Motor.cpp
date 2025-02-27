#include <Arduino.h>

class Motor
{
  private:
    int pins[4] = {0, 1, 2, 3};
    int currentStep = 0;
    const double stepSize = 5.625/64; // degrees per step
    const int motorSeq[8] = {0b01000, 0b01100, 0b00100, 0b00110, 0b00010, 0b00011, 0b00001, 0b01001};
    double clockDelayMs;

    // Queued rotation variables
    int stepsRemaining = 0;
    double turnSpeed = 0; // degrees per second
    double msPerStep = 0;
    bool steppingClockwise = true;
    double msUntilNextStep = 0;

    void WriteToPins()
    {
      digitalWrite(pins[0], motorSeq[currentStep] & 1);
      digitalWrite(pins[1], motorSeq[currentStep] & 2);
      digitalWrite(pins[2], motorSeq[currentStep] & 4);
      digitalWrite(pins[3], motorSeq[currentStep] & 8);
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

  public:
    Motor(int pins[], double clockDelayMs)
    {
      this->clockDelayMs = clockDelayMs;
      this->pins[0] = pins[0];
      this->pins[1] = pins[1];
      this->pins[2] = pins[2];
      this->pins[3] = pins[3];
      pinMode(this->pins[0], OUTPUT);
      pinMode(this->pins[1], OUTPUT);
      pinMode(this->pins[2], OUTPUT);
      pinMode(this->pins[3], OUTPUT);
    }

    void QueueRotation(bool isClockwise, double degrees, double degreesPerSecond)
    {
      this->steppingClockwise = isClockwise;
      this->stepsRemaining = degrees / this->stepSize;
      this->turnSpeed = degreesPerSecond;
      this->msPerStep = 1 / degreesPerSecond * this->stepSize * 1000; // TODO: check if this is less than the clock delay. Meaning it is not possible to rotate the motor as quickly as desired
      this->msUntilNextStep = 0;
    }

    void QueueRotation(double degrees, double degreesPerSecond)
    {
      bool isClockwise = degrees > 0;
      degrees = degrees > 0 ? degrees : -degrees;
      this->QueueRotation(isClockwise, degrees, degreesPerSecond);
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

      if(this->msUntilNextStep > 0)
      {
        this->msUntilNextStep -= this->clockDelayMs;
      }
      else
      {
        this->stepsRemaining -= 1;
        this->DoStep(this->steppingClockwise);
        this->msUntilNextStep += this->msPerStep;
      }
    }
};