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

    Coordinates(const Coordinates& obj)
    {
      this->x = obj.x;
      this->y = obj.y;
    }

    String ToString()
    {
      return "(" + String(this->x) + ", " + String(this->y) + ")";
    }

    Coordinates Normalize()
    {
      double length = sqrt(pow(this->x, 2) + pow(this->y, 2));
      this->x /= length;
      this->y /= length;
      return *this;
    }

    void Add(Coordinates obj)
    {
      this->x += obj.x;
      this->y +=  obj.y;
    }

    void Substract(Coordinates obj)
    {
      this->x -= obj.x;
      this->y -= obj.y;
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

    Coordinates operator+=(Coordinates obj)
    {
      this->Add(obj);
      return *this;
    }

    Coordinates operator*(double scalar)
    {
      return Coordinates(this->x * scalar, this->y * scalar);
    }
};

class CoordinatesQueue
{
  private:
    Coordinates* storageArray[50];
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
      Serial.println(String(getPointer));
      Coordinates* firstCoords = storageArray[getPointer];
      Serial.println(firstCoords->ToString());
      getPointer++;
      return firstCoords;
    }

    int Size()
    {
      return putPointer - getPointer;
    }
};