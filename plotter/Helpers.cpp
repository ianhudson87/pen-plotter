#include <math.h>
#include <Arduino.h>
#include <Vector.h>

class Coordinates
{
  public:
    double x;
    double y;

    Coordinates(double x, double y)
    {
      this->x = x;
      this->y = y;
    }

    String ToString()
    {
      return "(" + String(this->x) + ", " + String(this->y) + ")";
    }
};

class Helpers
{
  public:
    static double GetR2Distance(Coordinates* coord1, Coordinates* coord2)
    {
      double result = sqrt(pow(coord1->x - coord2->x, 2) + pow(coord1->y - coord2->y, 2));
      Serial.println("Calculated R2Dist " + coord1->ToString() + " -> " + coord2->ToString() + "=" + String(result));
      return result;
    }
};

class CoordinatesQueue
{
  private:
    Coordinates* storageArray[20];
    int getPointer = 0;
    int putPointer = 0;

  public:
    CoordinatesQueue()
    {
    }

    void QueueCoords(Coordinates* coordinates)
    {
      storageArray[putPointer] = coordinates;
      putPointer++;
    }

    Coordinates* DequeueCoords()
    {
      Coordinates* firstCoords = storageArray[getPointer];
      getPointer++;
      return firstCoords;
    }

    int Size()
    {
      return putPointer - getPointer;
    }
};