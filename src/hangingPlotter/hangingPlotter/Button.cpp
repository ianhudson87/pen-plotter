// How to use:
// Call ReadPin() every beginning of every loop.
// IsStateLowToHigh() tells you whether the button was pressed on the current loop


class Button
{
  private:
    int pin = 0;
    int prevState = LOW;
    int currentState = LOW;

  public:
    Button (int pin)
    {
      this->pin = pin;
      pinMode(pin, INPUT);
    }

    void ReadPin()
    {
      prevState = currentState;
      currentState = digitalRead(stateChangeButtonPin);
    }

    bool IsStateLowToHigh()
    {
      return prevState == LOW && currentState == HIGH;
    }
}