#include <PlotterStateMachine.h>

PlotterStateMachine::PlotterStateMachine(PlotterState startState)
{
  this->currentState = startState;
}

PlotterState PlotterStateMachine::GetCurrentState() const
{
  return currentState;
}

void PlotterStateMachine::SetState(PlotterState newState)
{
  this->currentState = newState;
}

void PlotterStateMachine::HandleEvent(PlotterEvent event)
{
  switch (currentState)
  {
    case PlotterState::Lowering:
      if (event == PlotterEvent::ButtonPress)
      {
        currentState = PlotterState::LeftRetracting;
      }
      break;
    case PlotterState::LeftRetracting:
      if (event == PlotterEvent::ButtonPress)
      {
        currentState = PlotterState::RightRetracting;
      }
      break;
    case PlotterState::RightRetracting:
      if (event == PlotterEvent::ButtonPress)
      {
        currentState = PlotterState::Calculating;
      }
      break;
    case PlotterState::Calculating:
      if (event == PlotterEvent::LoopComplete)
      {
        currentState = PlotterState::Moving;
      }
      break;
    case PlotterState::Moving:
      if (event == PlotterEvent::MotorRotationComplete)
      {
        currentState = PlotterState::Calculating;
      }
      else if (event == PlotterEvent::ButtonPress)
      {
        currentState = PlotterState::Reset;
      }
      break;
    case PlotterState::Reset:
      if (event == PlotterEvent::LoopComplete)
      {
        currentState = PlotterState::Lowering;
      }
      break;
  }
}
