#include <math.h>
#include <Arduino.h>

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

    Coordinates(Coordinates obj)
    {
      this->x = obj->x;
      this->y = obj->y;
    }

    String ToString()
    {
      return "(" + String(this->x) + ", " + String(this->y) + ")";
    }

    void Normalize()
    {
      double length = Helpers.GetR2Length(this);
      this->x /= length;
      this->y /= length;
    }

    void Add(Coordinates obj)
    {
      this->x += obj->x;
      this->y +=  obj->y;
    }

    void Substract(Coordinates obj)
    {
      this->x -= obj->x;
      this->y -= obj->y;
    }

    void Multiply(float scalar)
    {
      this->x *= scalar;
      this->y *= scalar;
    }

    Coordinates operator-(Coordinates obj)
    {
      return Coordinates(this->x - obj.x, this->y - obj.y);
    }

    Coordinates operator+(Coordinates obj)
    {
      return Coordinates(this->x + obj.x, this->y + obj.y);
    }

    Coordinates operator*(double scalar)
    {
      return Coordinates(this->x * scalar, this->y * scalar);
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

    static double GetR2Distance(double x1, double y1, double x2, double y2)
    {
      double result = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
      return result;
    }

    static double GetR2Length(Coordinates* coords)
    {
      double result = sqrt(pow(coords->x, 2) + pow(coords->y, 2));
      return result;
    }

    static double GetR2Length(double x1, double y1)
    {
      double result = sqrt(pow(x1, 2) + pow(y1, 2));
      return result;
    }

    static CoordinatesQueue* SplitUpCoordinatesQueue(CoorindatesQueue* originalQueue, double intervalSize)
    {
      CoordinatesQueue* splitUpQueue = new CoordinatesQueue();

      Coordinates* startPos = originalQueue->DequeueCoords();
      splitUpQueue.QueueCoords(startPos);

      while(originalQueue->Size() > 0)
      {
        Coordinates* endPos = originalQueue->DequeueCoords();

        if(Helpers.GetR2Distance(startPos, endPos) > intervalSize) // TODO: maybe special logic if distance between startPos and endPos is close to intervalSize
        {
          Coordinates directionUnitVector = (*endPos - *startPos).Normalize();

          Coordindates currentPos(*startPos + (directionUnitVector * intervalSize));

          while(Helpers.GetR2Distance(currentPos, endPos) > intervalSize)
          {
            splitUpQueue.QueueCoords(new Coordinates(currentPos));

            currentPos += (directionUnitVector * intervalSize);
          }
        }

        splitUpQueue.QueueCoords(endPos);
        startPos = endPos;
      }

      Serial.println("Created split up queue of size: " + String(splitUpQueue.Size()));
      return splitUpQueue;
    }
};

class CoordinatesQueue
{
  private:
    Coordinates* storageArray[500];
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