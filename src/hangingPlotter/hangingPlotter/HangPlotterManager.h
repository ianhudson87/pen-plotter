#pragma once

#include <Vector2D.h>

class HangPlotterManager
{
  private:
    const float d_betweenMotors = 17.2;
    const float d_betweenConnections = 2.75;
    const float radius_Motor = 1.435;
    const float circum_Motor = radius_Motor * TWO_PI;
    const float stringLengthStartingVal = 10.0;
    const float motorSpeed = 10;

    float d_leftString = stringLengthStartingVal;
    float d_rightString = stringLengthStartingVal;

  public:
    HangPlotterManager();
    void Reset();
    Vector2D GetTargetLengths(Vector2D targetPos) const;
    Vector2D GetMotorRotations(Vector2D targetLengths) const;
    Vector2D GetMotorRotationSpeeds(Vector2D motorRotations) const;
    void SetCurrentLengths(Vector2D currentLengths);
};
