#include <math.h>
#include <Arduino.h>

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
      double result = (alpha + beta) / M_PI * 180;
      Serial.println("Solved shoulder angle (" + String(x) + ", " + String(y) + ") --> " + String(result) + " degrees");
      return result;
    }

    double GetElbowAngle(double x, double y)
    {
      double hypSq = pow(x, 2) + pow(y, 2);
      double phi = acos((hypSq - pow(forearmLen,2) - pow(bicepLen,2)) / (-2 * forearmLen * bicepLen));
      double result = phi / M_PI * 180;
      Serial.println("Solved elbow angle (" + String(x) + ", " + String(y) + ") --> " + String(result) + " degrees");
      return result;
    }
};