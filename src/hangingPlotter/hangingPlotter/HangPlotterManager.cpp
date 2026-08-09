#include <Arduino.h>
#include <HangPlotterManager.h>

HangPlotterManager::HangPlotterManager()
{
}

void HangPlotterManager::Reset()
{
  this->leftStringLength = startingStringLength;
  this->rightStringLength = startingStringLength;
}

Vector2D HangPlotterManager::GetTargetLengths(Vector2D targetPosition) const
{
  float targetLeftStringLength = sqrt(sq(targetPosition.x - distanceBetweenConnections / 2) + sq(targetPosition.y));
  float targetRightStringLength = sqrt(sq(distanceBetweenMotors - distanceBetweenConnections / 2 - targetPosition.x) + sq(targetPosition.y));
  Vector2D result{targetLeftStringLength, targetRightStringLength};
  return result;
}

Vector2D HangPlotterManager::GetMotorRotations(Vector2D targetLengths) const
{
  float leftMotorRotation = (leftStringLength - targetLengths.x) / motorCircumference * 360.0f;
  float rightMotorRotation = (rightStringLength - targetLengths.y) / motorCircumference * -360.0f;
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
    return Vector2D(motorDegreesPerSecond, 0);
  }

  if (leftRotationAbs == 0)
  {
    return Vector2D(0, motorDegreesPerSecond);
  }

  float leftMotorDegreesPerSecond = leftRotationAbs > rightRotationAbs ? motorDegreesPerSecond : motorDegreesPerSecond * abs(motorRotations.x / motorRotations.y);
  float rightMotorDegreesPerSecond = rightRotationAbs > leftRotationAbs ? motorDegreesPerSecond : motorDegreesPerSecond * abs(motorRotations.y / motorRotations.x);
  Vector2D result{leftMotorDegreesPerSecond, rightMotorDegreesPerSecond};
  return result;
}

void HangPlotterManager::SetCurrentLengths(Vector2D currentLengths)
{
  this->leftStringLength = currentLengths.x;
  this->rightStringLength = currentLengths.y;
}
