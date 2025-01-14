struct Coordinates
{
  double x;
  double y;
};

class CoordinatesQueue
{
  private:
    std::vector<Coordinates> coordinates;

  public:
    void QueueCoords(Coordinates coordinates)
    {
      this->coordinates.push_back(coordinates);
    }

    Coordinates DequeueCoords(Coordinates coordinates)
    {
      Coordinates firstCoords = this->coordinates.front;
      this->coordinates.erase(this->coordinates.begin());
      return firstCoords;
    }
}

class MovementPlanner
{
  private:
    Motor shoulderMotor;
    Motor elbowMotor;
    Motor wristMotor;
    CoordinatesQueue coordindatesQueue;
    Coordinates lastKnownCoordinates;
    Coordinates targetCoordinates;
    double movementSpeed; // cm per second

    void QueueMotorRotations(Coordinates startCoord, Coordinates targetCoord)
    {
      double startingShoulder = angleSolver.GetShoulderAngle(startCoord.x, startCoord.y);
      double startingElbow = angleSolver.GetElbowAngle(startCoord.x, startCoord.y);

      double targetShoulder = angleSolver.GetShoulderAngle(targetCoord.x, targetCoord.y;
      double targetElbow = angleSolver.GetElbowAngle(targetCoord.x, targetCoord.y);

      double shoulderAngleDelta = targetShoulder - startingShoulder;
      double elbowAngleDelta = startingElbow - targetElbow;

      double shoulderRotationSpeed = abs(shoulderAngleDelta) / Helpers.GetR2Distance(startCoord, targetCoord) * this->movementSpeed;
      double elbowRotationSpeed = abs(shoulderAngleDelta) / Helpers.GetR2Distance(startCoord, targetCoord) * this->movementSpeed;

      this->shoulderMotor.QueueRotation(shoulderAngleDelta, shoulderRotationSpeed);
      this->elbowMotor.QueueRotation(elbowAngleDelta, elbowRotationSpeed);
    }

    void ProcessMotorRotations();
    {
      this->shoulderMotor.ProcessRotation();
      this->elbowMotor.ProcessRotation();
    }

  public:
    MovementPlanner(Motor shoulderMotor, Motor elbowMotor, Motor wristMotor, CoordinatesQueue coordinatesQueue, Coordinates, currentCoordinates, double movementSpeed)
    {
      this->shoulderMotor = shoulderMotor;
      this->elbowMotor = elbowMotor;
      this->wristMotor = wristMotor;
      this->coordindatesQueue = coordinatesQueue;
      this->movementSpeed = movementSpeed;
      this->lastKnownCoordinates = currentCoordinates;
      this->targetCoordinates = currentCoordinates;
    }

    void ProcessMovement()
    {
      if(this->shoulderMotor.IsDoneRotating() && this->shoulderMotor.IsDoneRotating())
      {
        this->lastKnownCoordinates = this->targetCoordinates;
        this->targetCoordinates = this->coordinatesQueue.DequeueCoords();

        Serial.println("targeting new coordinates" + String(this->targetCoordinates.x) + String(this->targetCoordinates.y));
        
        this->QueueMotorRotations(this->lastKnownCoordinates, this->targetCoordinates);
      }

      this->ProcessMotorRotations();
    }
}