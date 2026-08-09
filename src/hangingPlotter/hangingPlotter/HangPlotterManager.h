#pragma once

#include <Vector2D.h>

class HangPlotterManager
{
  private:
    const float distanceBetweenMotors = 172.0;
    const float distanceBetweenConnections = 27.5;
    const float motorRadius = 14.35;
    const float motorCircumference = motorRadius * TWO_PI;
    const float startingStringLength = 100.0;
    const float motorDegreesPerSecond = 10;

    float leftStringLength = startingStringLength;
    float rightStringLength = startingStringLength;

  public:
    HangPlotterManager();
    void Reset();
    Vector2D GetTargetLengths(Vector2D targetPosition) const;
    Vector2D GetMotorRotations(Vector2D targetLengths) const;
    Vector2D GetMotorRotationSpeeds(Vector2D motorRotations) const;
    void SetCurrentLengths(Vector2D currentLengths);
};
