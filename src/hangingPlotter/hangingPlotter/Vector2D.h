#pragma once

#include <Arduino.h>

class Vector2D
{
  public:
    double x;
    double y;

    Vector2D();
    Vector2D(double x, double y);
    Vector2D(const Vector2D& obj);

    String ToString() const;
    Vector2D Normalize();

    void Add(const Vector2D& obj);
    void Substract(const Vector2D& obj);
    void Multiply(float scalar);

    Vector2D operator-(const Vector2D& obj) const;
    Vector2D operator+(const Vector2D& obj) const;
    Vector2D operator+=(const Vector2D& obj);
    Vector2D operator*(double scalar) const;
    Vector2D operator*=(double scalar);
};

class Vector2DQueue
{
  private:
    Vector2D* storageArray[50];
    int getPointer = 0;
    int putPointer = 0;

  public:
    Vector2DQueue();
    void QueueCoords(Vector2D* vector2D);
    Vector2D* DequeueCoords();
    int Size() const;
};
