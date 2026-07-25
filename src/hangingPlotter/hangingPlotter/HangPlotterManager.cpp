#include <Arduino.h>
#include <HangPlotterManager.h>

HangPlotterManager::HangPlotterManager()
{
}

void HangPlotterManager::Reset()
{
  this->d_leftString = stringLengthStartingVal;
  this->d_rightString = stringLengthStartingVal;
}

Vector2D HangPlotterManager::GetTargetLengths(Vector2D targetPos) const
{
  float targetLeftStringLength = sqrt(sq(targetPos.x - d_betweenConnections / 2) + sq(targetPos.y));
  float targetRightStringLength = sqrt(sq(d_betweenMotors - d_betweenConnections / 2 - targetPos.x) + sq(targetPos.y));
  Vector2D result{targetLeftStringLength, targetRightStringLength};
  return result;
}

Vector2D HangPlotterManager::GetMotorRotations(Vector2D targetLengths) const
{
  float leftMotorRotation = (d_leftString - targetLengths.x) / circum_Motor * 360.0f;
  float rightMotorRotation = (d_rightString - targetLengths.y) / circum_Motor * -360.0f;
  Vector2D result{leftMotorRotation, rightMotorRotation};
  return result;
}

Vector2D HangPlotterManager::GetMotorRotationSpeeds(Vector2D motorRotations) const
{
  float leftRotationAbs = abs(motorRotations.x);
  float rightRotationAbs = abs(motorRotations.y);

  if (leftRotationAbs == 0 && rightRotationAbs == 0)
  {
    return Vector2D(0, 0);
  }

  if (rightRotationAbs == 0)
  {
    return Vector2D(motorSpeed, 0);
  }

  if (leftRotationAbs == 0)
  {
    return Vector2D(0, motorSpeed);
  }

  float leftMotorSpeed = leftRotationAbs > rightRotationAbs ? motorSpeed : motorSpeed * abs(motorRotations.x / motorRotations.y);
  float rightMotorSpeed = rightRotationAbs > leftRotationAbs ? motorSpeed : motorSpeed * abs(motorRotations.y / motorRotations.x);
  Vector2D result{leftMotorSpeed, rightMotorSpeed};
  return result;
}

void HangPlotterManager::SetCurrentLengths(Vector2D currentLengths)
{
  this->d_leftString = currentLengths.x;
  this->d_rightString = currentLengths.y;
}
