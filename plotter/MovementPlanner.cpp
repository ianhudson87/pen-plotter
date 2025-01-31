#include "Motor.cpp"
#include "AngleSolver.cpp"
#include "Helpers.cpp"
#include <Arduino.h>

class MovementPlanner
{
  private:
    Motor shoulderMotor;
    Motor elbowMotor;
    Motor wristMotor;
    CoordinatesQueue* coordinatesQueue;
    Coordinates* lastKnownCoordinates;
    Coordinates* targetCoordinates;
    AngleSolver angleSolver;
    double movementSpeed; // cm per second

    void QueueMotorRotations(Coordinates* startCoord, Coordinates* targetCoord)
    {
      double startingShoulder = this->angleSolver.GetShoulderAngle(startCoord->x, startCoord->y);
      double startingElbow = this->angleSolver.GetElbowAngle(startCoord->x, startCoord->y);

      double targetShoulder = this->angleSolver.GetShoulderAngle(targetCoord->x, targetCoord->y);
      double targetElbow = this->angleSolver.GetElbowAngle(targetCoord->x, targetCoord->y);

      double shoulderAngleDelta = targetShoulder - startingShoulder;
      double elbowAngleDelta = startingElbow - targetElbow;

      double shoulderRotationSpeed = abs(shoulderAngleDelta) / Helpers::GetR2Distance(startCoord, targetCoord) * this->movementSpeed;
      double elbowRotationSpeed = abs(elbowAngleDelta) / Helpers::GetR2Distance(startCoord, targetCoord) * this->movementSpeed;

      Serial.println(String(abs(shoulderAngleDelta)) + ":" + String(this->movementSpeed));
      Serial.println("calculated rotation speeds: (" + String(shoulderRotationSpeed) + ", " + String(elbowRotationSpeed) + ")");

      this->shoulderMotor.QueueRotation(shoulderAngleDelta, shoulderRotationSpeed);
      this->elbowMotor.QueueRotation(elbowAngleDelta, elbowRotationSpeed);
    }

    void ProcessMotorRotations()
    {
      // Serial.println("processing motor rotations");
      this->shoulderMotor.ProcessRotation();
      this->elbowMotor.ProcessRotation();
    }

  public:
    MovementPlanner(Motor shoulderMotor, Motor elbowMotor, Motor wristMotor, AngleSolver angleSolver, CoordinatesQueue* targetPoints, Coordinates* currentCoordinates, double movementSpeed, double movementIntervalSize)
      : shoulderMotor(shoulderMotor), elbowMotor(elbowMotor), wristMotor(wristMotor),
      angleSolver(angleSolver), coordinatesQueue(coordinatesQueue), lastKnownCoordinates(currentCoordinates), targetCoordinates(currentCoordinates)
    {
      this->movementSpeed = movementSpeed;
      this->coordinatesQueue = Helpers::SplitUpCoordinatesQueue(targetPoints, movementIntervalSize);
      // break up coordinates queue lines into multiple lines
    }

    void ProcessMovement()
    {
      // Serial.println("shoulder done:" + String(this->shoulderMotor.GetStepsRemaining()) + "elbow done:" + String(this->elbowMotor.GetStepsRemaining()));
      if(this->shoulderMotor.IsDoneRotating() && this->elbowMotor.IsDoneRotating())
      {
        if(this->coordinatesQueue->Size() >= 0)
        {
          this->lastKnownCoordinates = this->targetCoordinates;
          this->targetCoordinates = this->coordinatesQueue->DequeueCoords();
          Serial.println("targeting new coordinates: (" + String(this->targetCoordinates->x) + ", " + String(this->targetCoordinates->y) + ")");
          this->QueueMotorRotations(this->lastKnownCoordinates, this->targetCoordinates);
        }
        else
        {
          Serial.println("done moving");
        }
      }

      this->ProcessMotorRotations();
    }
};