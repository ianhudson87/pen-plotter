#include "Coordinates.cpp"

class MotorDriver
{
  private:
    int spiralIterations = 0;
    Coordinates lastPos;

  public:
    MotorDriver(Coordinates startingPosition)
    {
      this->lastPos = startingPosition;
    }

    Coordinates GetNextPos()
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
}