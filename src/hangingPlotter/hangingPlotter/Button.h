#pragma once

#include <Arduino.h>

class Button
{
  private:
    int pin = 0;
    int prevState = LOW;
    int currentState = LOW;

  public:
    explicit Button(int pin);
    void ReadPin();
    bool IsStateLowToHigh() const;
    bool IsHigh() const;
};
