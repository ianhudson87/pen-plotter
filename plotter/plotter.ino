// This Arduino example demonstrates bidirectional operation of a
// 28BYJ-48, using a ULN2003 interface board to drive the stepper.
// The 28BYJ-48 motor is a 4-phase, 8-beat motor, geared down by
// a factor of 68. One bipolar winding is on motor pins 1 & 3 and
// the other on motor pins 2 & 4. The step angle is 5.625/64 and the
// operating Frequency is 100pps. Current draw is 92mA.
////////////////////////////////////////////////

int shoulderPin1 = 0;
int shoulderPin2 = 0;
int shoulderPin3 = 0;
int shoulderPin4 = 0;

int elbowPin1 = 0;
int elbowPin2 = 0;
int elbowPin3 = 0;
int elbowPin4 = 0;

const double clockDelayMs = 5;

class Motor
{
  private:
    int[4] pins;
    int currentStep = 0;
    const double stepSize = 5.625/64; // degrees per step
    const static int[8] motorSeq = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001};
    double clockDelayMs;

    // Queued rotation variables
    int stepsRemaining = 0;
    double turnSpeed = 0; // degrees per second
    double msPerStep = 0;
    bool isClockwise = true;
    double msUntilNextStep = 0;

    void WriteToPins()
    {
      digitalWrite(pins[0], motorSeq[currentStep] & 1);
      digitalWrite(pins[1], motorSeq[currentStep] & 2);
      digitalWrite(pins[2], motorSeq[currentStep] & 4);
      digitalWrite(pins[3], motorSeq[currentStep] & 8);
    }

    void StepClockwise()
    {
      currentStep += 1;
      if(currentStep >=8 )
      {
        currentStep -= 8;
      }
      WriteToPins();
    }

    void StepAnticlockwise()
    {
      currentStep -= 1;
      if(currentStep < 0 )
      {
        currentStep += 8;
      }
      WriteToPins();
    }

  public:
    Motor(int[4] pins, double clockDelayMs)
    {
      this.clockDelayMs = clockDelayMs
      this.pins = pins;
      pinMode(this.pins[0], OUTPUT);
      pinMode(this.pins[1], OUTPUT);
      pinMode(this.pins[2], OUTPUT);
      pinMode(this.pins[3], OUTPUT);
    }

    void QueueRotation(bool isClockwise, double degrees, double degreesPerSecond)
    {
      this.isClockwise = isClockwise;
      this.stepsRemaining = degrees / this.stepSize;
      this.turnSpeed = degreesPerSecond;
      this.msPerStep = 1 / degreesPerSecond * this.stepSize * 1000; // TODO: check if this is less than the clock delay. Meaning it is not possible to rotate the motor as quickly as desired
      this.msUntilNextStep = 0;
    }

    void ProcessRotation()
    {
      if( this.stepsRemaining == 0 )
      {
        return;
      }

      if(this.msUntilNextStep > 0)
      {
        this.msUntilNextStep -= this.clockDelayMs;
      }
      else
      {
        
      }
    }
}

//declare variables for the motor pins
int motorPin1 = 8;    // Blue   - 28BYJ48 pin 1
int motorPin2 = 9;    // Pink   - 28BYJ48 pin 2
int motorPin3 = 10;    // Yellow - 28BYJ48 pin 3
int motorPin4 = 11;    // Orange - 28BYJ48 pin 4
                        // Red    - 28BYJ48 pin 5 (VCC)

int motorSpeed = 32;//variable to set stepper speed
int count = 0;          // count of steps made
int countsperrev = 150; // number of steps per full revolution
int lookup[8] = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001};

//////////////////////////////////////////////////////////////////////////////
void setup() {
  //declare the motor pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  Serial.begin(9600);
  Serial.println("stepper");
}

//////////////////////////////////////////////////////////////////////////////
void loop(){
  int startTime = millis();
  

  int endTime = millis();
  int deltaTime = endTime - startTime;
  if(deltaTime < clockDelayMs)
  {
    delay(clockDelayMs - deltaTime);
  }
  else
  {
    Serial.println("missed timing by" + (deltaTime - clockDelayMs));
  }
}

//////////////////////////////////////////////////////////////////////////////
//set pins to ULN2003 high in sequence from 1 to 4
//delay "motorSpeed" between each pin setting (to determine speed)
void anticlockwise()
{
  for(int i = 0; i < 8; i++)
  {
    setOutput(i);
    delay(motorSpeed);
  }
}

void clockwise()
{
  for(int i = 7; i >= 0; i--)
  {
    setOutput(i);
    delay(motorSpeed);
  }
}

void setOutput(int out)
{
  digitalWrite(motorPin1, bitRead(lookup[out], 0));
  digitalWrite(motorPin2, bitRead(lookup[out], 1));
  digitalWrite(motorPin3, bitRead(lookup[out], 2));
  digitalWrite(motorPin4, bitRead(lookup[out], 3));
}