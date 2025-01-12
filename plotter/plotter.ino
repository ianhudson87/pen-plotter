// This Arduino example demonstrates bidirectional operation of a
// 28BYJ-48, using a ULN2003 interface board to drive the stepper.
// The 28BYJ-48 motor is a 4-phase, 8-beat motor, geared down by
// a factor of 68. One bipolar winding is on motor pins 1 & 3 and
// the other on motor pins 2 & 4. The step angle is 5.625/64 and the
// operating Frequency is 100pps. Current draw is 92mA.
////////////////////////////////////////////////

class AngleSolver
{
  private:
    double bicepLen;
    double forearmLen;
  
  public:
    AngleSolver(double bicepLen, double forearmLen)
    {
      this->bicepLen = bicepLen;
      this->forearmLen = forearmLen;
    }

    double GetShoulderAngle(double x, double y)
    {
      double hypSq = pow(x, 2) + pow(y, 2);
      double hyp = sqrt(hypSq);
      double beta = acos((pow(forearmLen,2) - pow(bicepLen,2) - hypSq) / (-2 * bicepLen * hyp));
      double alpha = atan(x/y);
      return (alpha + beta) / PI * 180;
    }

    double GetElbowAngle(double x, double y)
    {
      double hypSq = pow(x, 2) + pow(y, 2);
      double phi = acos((hypSq - pow(forearmLen,2) - pow(bicepLen,2)) / (-2 * forearmLen * bicepLen));
      return phi / PI * 180;
    }
};

class Motor
{
  private:
    int pins[4] = {0, 1, 2, 3};
    int currentStep = 0;
    const double stepSize = 5.625/64; // degrees per step
    const int motorSeq[8] = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001};
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

bool started = false;
double clockDelayMs = 5;
int shoulderMotorPins[4] = {13,12,11,10};
int elbowMotorPins[4] = {9,8,7,6};
int wristMotorPins[4] = {5,4,3,2};
Motor shoulderMotor(shoulderMotorPins, clockDelayMs);
Motor elbowMotor(elbowMotorPins, clockDelayMs);
Motor wristMotor(wristMotorPins, clockDelayMs);
AngleSolver angleSolver(12.9, 18.7);

double startingX = 13.7;
double startingY = 13.5;
double targetX = 25;
double targetY = 7;

//////////////////////////////////////////////////////////////////////////////
void setup() {
  double startingShoulder = angleSolver.GetShoulderAngle(startingX, startingY);
  double startingElbow = angleSolver.GetElbowAngle(startingX, startingY);

  double targetShoulder = angleSolver.GetShoulderAngle(targetX, targetY);
  double targetElbow = angleSolver.GetElbowAngle(targetX, targetY);

  Serial.println("start: " + String(startingShoulder) + "," + String(startingElbow));
  Serial.println("target: " + String(targetShoulder) + "," + String(targetElbow));


  shoulderMotor.QueueRotation(targetShoulder - startingShoulder, 2);
  elbowMotor.QueueRotation(startingElbow - targetElbow, 2);
  wristMotor.QueueRotation(false, 180, 1);

  shoulderMotor.QueueRotation(0, 2);
  elbowMotor.QueueRotation(90, 2);

  Serial.begin(9600);
  Serial.println("stepper");
}

//////////////////////////////////////////////////////////////////////////////
void loop(){
  if (!started && Serial.available() == 0) {
    return;
  }
  else
  {
    started = true;
    // read the incoming byte:
    int incomingByte = Serial.read();
  }

  int startTime = millis();
  
  // shoulderMotor.ProcessRotation();
  // elbowMotor.ProcessRotation();
  wristMotor.ProcessRotation();

  // double startingShoulder = angleSolver.GetShoulderAngle(startingX, startingY);
  // double startingElbow = angleSolver.GetElbowAngle(startingX, startingY);

  // double targetShoulder = angleSolver.GetShoulderAngle(targetX, targetY);
  // double targetElbow = angleSolver.GetElbowAngle(targetX, targetY);

  // Serial.println("start: " + String(startingShoulder) + "," + String(startingElbow));
  // Serial.println("target: " + String(targetShoulder) + "," + String(targetElbow));

  int endTime = millis();
  int deltaTime = endTime - startTime;
  if(deltaTime < clockDelayMs)
  {
    delay(clockDelayMs - deltaTime);
  }
  else
  {
    Serial.println("missed timing by" + String(deltaTime - clockDelayMs, 10));
  }
}

// //////////////////////////////////////////////////////////////////////////////
// //set pins to ULN2003 high in sequence from 1 to 4
// //delay "motorSpeed" between each pin setting (to determine speed)
// void anticlockwise()
// {
//   for(int i = 0; i < 8; i++)
//   {
//     setOutput(i);
//     delay(motorSpeed);
//   }
// }

// void clockwise()
// {
//   for(int i = 7; i >= 0; i--)
//   {
//     setOutput(i);
//     delay(motorSpeed);
//   }
// }

// void setOutput(int out)
// {
//   digitalWrite(motorPin1, bitRead(lookup[out], 0));
//   digitalWrite(motorPin2, bitRead(lookup[out], 1));
//   digitalWrite(motorPin3, bitRead(lookup[out], 2));
//   digitalWrite(motorPin4, bitRead(lookup[out], 3));
// }