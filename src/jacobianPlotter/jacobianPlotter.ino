#include <Servo.h>
#include <Arduino.h>

// Conversion factor from degrees to radians
#define DEG_TO_RAD 0.017453292519943295769236907684886
 
// Conversion factor from radians to degrees
#define RAD_TO_DEG 57.295779513082320876798154814105

#define SAFETY_DEG 10

#define SHOULDER_CORRECTION_DEG 4.0
#define ELBOW_CORRECTION_DEG -7.0

int shoulderPin = 2;
int elbowPin = 3;

Servo shoulderServo;
Servo elbowServo;

double xVel = 0.0;
double yVel = 0.0;

double a = 11.05;
double b = 11.3;

double theta = 90.0;
double phi = 90.0;

double clockPeriodMs = 200;

struct Matrix {
 double m11;
 double m12;
 double m21;
 double m22;
};

struct Tuple {
 double x;
 double y;
};

void setup() {
  Serial.begin(9600);

  elbowServo.attach(elbowPin);
  shoulderServo.attach(shoulderPin);

  WriteMotorAngles(theta, phi);
}

void loop() {
  int startTime = millis();

  //// START ////
  ReadAndSetDirection();

  Matrix jInv = CalculateInverseJacobian(theta, phi, a, b);

  double thetaVel = jInv.m11 * xVel + jInv.m12 * yVel;
  double phiVel = jInv.m21 * xVel + jInv.m22 * yVel;

  theta += thetaVel * clockPeriodMs / 1000.0;
  phi += phiVel * clockPeriodMs / 1000.0;

  if(theta > 180-SAFETY_DEG || theta < SAFETY_DEG || phi > 180-SAFETY_DEG || phi < SAFETY_DEG)
  {
    theta -= thetaVel * clockPeriodMs / 1000.0;
    phi -= phiVel * clockPeriodMs / 1000.0;
  }

  WriteMotorAngles(theta, phi);

  Tuple pos = GetGlobalPos(theta, phi, a, b);
  SendGlobalPos(pos.x, pos.y);

  //// END ////

  int endTime = millis();
  int deltaTime = endTime - startTime;
  if(deltaTime < clockPeriodMs)
  {
    delay(clockPeriodMs - deltaTime);
  }
  else
  {
    Serial.println("missed timing by" + String(deltaTime - clockPeriodMs, 10));
  }
}

void WriteMotorAngles(double shoulderAngle, double elbowAngle)
{
  double correctedShoulderAngle = shoulderAngle + SHOULDER_CORRECTION_DEG;
  double correctedElbowAngle = 180.0 - elbowAngle + ELBOW_CORRECTION_DEG;
  shoulderServo.write(correctedShoulderAngle);
  // shoulderServo.writeMicroseconds((correctedShoulderAngle / 90.0 + 0.5) * 1000.0);
  elbowServo.write(correctedElbowAngle);
  // elbowServo.writeMicroseconds(((correctedElbowAngle) / 90.0 + 0.5) * 1000.0);
}

void WriteTrueMotorAngles(double shoulderAngle, double elbowAngle)
{
  shoulderServo.write(shoulderAngle);
  // shoulderServo.writeMicroseconds((shoulderAngle / 90.0 + 0.5) * 1000.0);
  elbowServo.write(180.0 - elbowAngle);
  // elbowServo.writeMicroseconds(((180.0 - elbowAngle) / 90.0 + 0.5) * 1000.0);
}

void ReadAndSetDirection()
{
  if(Serial.available() > 0)
  {
    float x = Serial.parseFloat();
    float y = Serial.parseFloat();

    xVel = double(x);
    yVel = double(y);

    Serial.println(String(x));
    Serial.println(String(y));
    Serial.println(String(xVel));
    Serial.println(String(yVel));

    while(Serial.available() > 0)
    {
      Serial.read();
    }
  }
}

Matrix CalculateInverseJacobian(double theta, double phi, double a, double b)
{
  double thetaRad = theta * DEG_TO_RAD;
  double phiRad = phi * DEG_TO_RAD;

  double j11 = b * cos(thetaRad + phiRad);
  double j12 = b * sin(thetaRad + phiRad);
  double j21 = -b * cos(thetaRad + phiRad) - a * cos(thetaRad);
  double j22 = -b * sin(thetaRad + phiRad) - a * sin(thetaRad);
  double det = 1.0 / (-a * b * ( (sin(thetaRad) * cos(thetaRad + phiRad)) - (sin(thetaRad + phiRad) * cos(thetaRad)) ));
  Matrix result = {j11 * det, j12 * det, j21 * det, j22 * det};
  return result;
}

void SendGlobalPos(double x, double y)
{
  Serial.println("[pos] " + String(x) + " " + String(y));
}

Tuple GetGlobalPos(double theta, double phi, double a, double b)
{
  double thetaRad = theta * DEG_TO_RAD;
  double phiRad = phi * DEG_TO_RAD;

  double x = b * cos(thetaRad + phiRad) + a * cos(thetaRad);
  double y = b * sin(thetaRad + phiRad) + a * sin(thetaRad);

  Tuple result = {x, y};
  return result;
}