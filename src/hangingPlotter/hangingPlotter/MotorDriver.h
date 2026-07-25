#pragma once

#include <Arduino.h>

class MotorDriver
{
  private:
    int pins[4] = {0, 1, 2, 3};
    int currentStep = 0;
    const double stepSize = 5.625 / 64;
    const int motorSeq[8] = {0b01000, 0b01100, 0b00100, 0b00110, 0b00010, 0b00011, 0b00001, 0b01001};

    unsigned long queuedStartTime = 0;
    int stepsRemaining = 0;
    int stepsTaken = 0;
    unsigned long msPerStep = 0;
    bool steppingClockwise = true;

    void WriteToPins();

  public:
    explicit MotorDriver(int pins[]);
    void QueueRotation(double rotateDegrees, double degreesPerSecond);
    bool IsDoneRotating() const;
    int GetStepsRemaining() const;
    void ProcessRotation();
    void DoStep(bool isClockwise);
};
