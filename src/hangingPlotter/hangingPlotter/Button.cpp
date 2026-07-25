#include <Button.h>

Button::Button(int pin)
{
  this->pin = pin;
  pinMode(pin, INPUT);
}

void Button::ReadPin()
{
  prevState = currentState;
  currentState = digitalRead(pin);
}

bool Button::IsStateLowToHigh() const
{
  return prevState == LOW && currentState == HIGH;
}

bool Button::IsHigh() const
{
  return currentState == HIGH;
}
