#pragma once

#include <Arduino.h>

class Vector2D
{
  public:
    double x;
    double y;

    Vector2D()
    {
      this->x = 0;
      this->y = 0;
    }

    Vector2D(double x, double y)
    {
      this->x = x;
      this->y = y;
    }

    Vector2D(const Vector2D& obj)
    {
      this->x = obj.x;
      this->y = obj.y;
    }

    String ToString()
    {
      return "(" + String(this->x) + ", " + String(this->y) + ")";
    }

    Vector2D Normalize()
    {
      double length = sqrt(pow(this->x, 2) + pow(this->y, 2));
      this->x /= length;
      this->y /= length;
      return *this;
    }

    void Add(Vector2D obj)
    {
      this->x += obj.x;
      this->y +=  obj.y;
    }

    void Substract(Vector2D obj)
    {
      this->x -= obj.x;
      this->y -= obj.y;
    }

    void Multiply(float scalar)
    {
      this->x *= scalar;
      this->y *= scalar;
    }

    Vector2D operator-(Vector2D obj)
    {
      return Vector2D(this->x - obj.x, this->y - obj.y);
    }

    Vector2D operator+(Vector2D obj)
    {
      return Vector2D(this->x + obj.x, this->y + obj.y);
    }

    Vector2D operator+=(Vector2D obj)
    {
      this->Add(obj);
      return *this;
    }

    Vector2D operator*(double scalar)
    {
      return Vector2D(this->x * scalar, this->y * scalar);
    }

    Vector2D operator*=(double scalar)
    {
      this->x *= scalar;
      this->y *= scalar;
      return *this;
    }
};

class Vector2DQueue
{
  private:
    Vector2D* storageArray[50];
    int getPointer = 0;
    int putPointer = 0;

  public:
    Vector2DQueue()
    {
    }

    void QueueCoords(Vector2D* Vector2D)
    {
      storageArray[putPointer] = Vector2D;
      putPointer++;
    }

    Vector2D* DequeueCoords()
    {
      Serial.println(String(getPointer));
      Vector2D* firstCoords = storageArray[getPointer];
      Serial.println(firstCoords->ToString());
      getPointer++;
      return firstCoords;
    }

    int Size()
    {
      return putPointer - getPointer;
    }
};
