#include <Arduino.h>

enum class PlotterState
{
  Lowering,
  LeftRetracting,
  RightRetracting,
  Calculating,
  Moving,
  Reset
};

enum class PlotterEvent
{
  ButtonPress,
  MotorRotationComplete,
  LoopComplete,
};

class PlotterStateMachine
{
  private:
    PlotterState currentState;
  
  public:
    PlotterStateMachine(PlotterState startState = PlotterState::Lowering)
    {
      this->currentState = startState;
    }

    PlotterState GetCurrentState()
    {
      return currentState;
    }

    void SetState(PlotterState newState)
    {
      this->currentState = newState;
    }

    void HandleEvent(PlotterEvent event)
    {
        switch (currentState)
        {
          case PlotterState::Lowering:
            if (event == PlotterEvent::ButtonPress)
            {
              currentState = PlotterState::LeftRetracting;
              Serial.println("Transitioned: Lowering -> LeftRetracting");
            }
            break;
          case PlotterState::LeftRetracting:
            if (event == PlotterEvent::ButtonPress)
            {
              currentState = PlotterState::RightRetracting;
              Serial.println("Transitioned: LeftRetracting -> RightRetracting");
            }
            break;
          case PlotterState::RightRetracting:
            if (event == PlotterEvent::ButtonPress)
            {
              currentState = PlotterState::Calculating;
              Serial.println("Transitioned: RightRetracting -> Calculating");
            }
            break;
          case PlotterState::Calculating:
            if (event == PlotterEvent::LoopComplete)
            {
              currentState = PlotterState::Moving;
              Serial.println("Transitioned: Calculating -> Moving");
            }
            break;
          case PlotterState::Moving:
            if (event == PlotterEvent::MotorRotationComplete)
            {
              currentState = PlotterState::Calculating;
              Serial.println("Transitioned: Moving -> Calculating");
            }
            else if (event == PlotterEvent::ButtonPress)
            {
              currentState = PlotterState::Reset;
              Serial.println("Transitioned: Moving -> Reset");
            }
            break;
          case PlotterState::Reset:
            if (event == PlotterEvent::LoopComplete)
            {
              currentState = PlotterState::Lowering;
              Serial.println("Transitioned: Reset -> Lowering");
            }
            break;
        }
    }
};
