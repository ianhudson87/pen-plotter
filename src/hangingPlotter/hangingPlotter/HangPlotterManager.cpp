// keeps tracks of the current lengths of the hanging plotter lengths
// calculates how many rotations the motor needs to move and how fast

class HangPlotterManager
{
  private:
    // readonly
    const float d_betweenMotors = 17.2; // distance between motors
    const float d_betweenConnections = 2.75; // distance between connection points
    const float radius_Motor = 1.435;
    const float circum_Motor = radius_Motor * TWO_PI;
    const float stringLengthStartingVal = 10.0; // string length from pulley to plotter head
    const float plotterHeadStartingX = 8.625; // top left is (0,0)
    const float plotterHeadStartingY = 6.88;
    const float motorSpeed = 10; // degrees per seconds

    // variable
    float d_leftString = stringLengthStartingVal; // current distance between left motor and left connection
    float d_rightString = stringLengthStartingVal; // current distance between right motor and right connection

  public:
    HangPlotterManager() {}

    void Reset()
    {
      this->d_leftString = stringLengthStartingVal;
      this->d_rightString = stringLengthStartingVal
    }

    // returns the lengths of the strings based on target position. x = left string. y = right string.
    Vector2D GetTargetLengths(Vector2D targetPos)
    {
      float targetLeftStringLength = sqrt(sq(targetPos.x - d_betweenConnections / 2) + sq(targetPos.y)); // distance from left motor to left connection point
      float targetRightStringLength = sqrt(sq(d_betweenMotors - d_betweenConnections / 2 - targetPos.x) + sq(targetPos.y)); // distance from right motor to right connection point
      Vector2D result{targetLeftStringLength, targetRightStringLength};
      return result;
    }

    // returns rotations needed for each motor to reach target lengths based on current lengths
    // input: targetLengths x = left string. y = right string
    // output: rotations x = left motor rotation in degrees. y = right motor rotation in degrees.
    Vector2D GetMotorRotations(Vector2D targetLengths)
    {
        float leftMotorRotation = (d_leftString - targetLengths.x) / circum_Motor * 360.0f;
        float rightMotorRotation = (d_rightString - targetLengths.y) / circum_Motor * -360.0f;
        Vector2D result{leftMotorRotation, rightMotorRotation};
        return result;
    }

    // Returns rotation speeds given rotation degrees needed per motor.
    // Motor that needs to rotate most will rotate at 'motorSpeed'. Second motor will rotate at constant speed that allows it to complete its rotation at same time.
    // input: rotations x = left motor. y = right motor
    // output: rotations speed x = left motor. y = right motor.
    Vector2D GetMotorRotationSpeeds(Vector2D motorRotations)
    {
        float leftMotorSpeed = abs(motorRotations.x) > abs(motorRotations.y) ? motorSpeed : motorSpeed * abs(motorRotations.x / motorRotations.y);
        float rightMotorSpeed = abs(motorRotations.y) > abs(motorRotations.x) ? motorSpeed : motorSpeed * abs(motorRotations.y / motorRotations.x);
        Vector2D result{leftMotorSpeed, rightMotorSpeed};
        return result;
    }

    // x = left string. y = right string
    void SetCurrentLengths(Vector2D currentLengths)
    {
      this->d_leftString = currentLengths.x;
      this->d_rightString = currentLengths.y;
    }
}