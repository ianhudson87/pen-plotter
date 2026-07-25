#include <Vector2D.h>

Vector2D::Vector2D()
{
  this->x = 0;
  this->y = 0;
}

Vector2D::Vector2D(double x, double y)
{
  this->x = x;
  this->y = y;
}

Vector2D::Vector2D(const Vector2D& obj)
{
  this->x = obj.x;
  this->y = obj.y;
}

String Vector2D::ToString() const
{
  return String('(') + String(this->x) + String(',') + String(' ') + String(this->y) + String(')');
}

Vector2D Vector2D::Normalize()
{
  double length = sqrt(pow(this->x, 2) + pow(this->y, 2));
  this->x /= length;
  this->y /= length;
  return *this;
}

void Vector2D::Add(const Vector2D& obj)
{
  this->x += obj.x;
  this->y += obj.y;
}

void Vector2D::Substract(const Vector2D& obj)
{
  this->x -= obj.x;
  this->y -= obj.y;
}

void Vector2D::Multiply(float scalar)
{
  this->x *= scalar;
  this->y *= scalar;
}

Vector2D Vector2D::operator-(const Vector2D& obj) const
{
  return Vector2D(this->x - obj.x, this->y - obj.y);
}

Vector2D Vector2D::operator+(const Vector2D& obj) const
{
  return Vector2D(this->x + obj.x, this->y + obj.y);
}

Vector2D Vector2D::operator+=(const Vector2D& obj)
{
  this->Add(obj);
  return *this;
}

Vector2D Vector2D::operator*(double scalar) const
{
  return Vector2D(this->x * scalar, this->y * scalar);
}

Vector2D Vector2D::operator*=(double scalar)
{
  this->x *= scalar;
  this->y *= scalar;
  return *this;
}

Vector2DQueue::Vector2DQueue()
{
}

void Vector2DQueue::QueueCoords(Vector2D* vector2D)
{
  storageArray[putPointer] = vector2D;
  putPointer++;
}

Vector2D* Vector2DQueue::DequeueCoords()
{
  Serial.println(String(getPointer));
  Vector2D* firstCoords = storageArray[getPointer];
  Serial.println(firstCoords->ToString());
  getPointer++;
  return firstCoords;
}

int Vector2DQueue::Size() const
{
  return putPointer - getPointer;
}
