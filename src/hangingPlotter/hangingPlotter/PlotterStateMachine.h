#pragma once

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
    explicit PlotterStateMachine(PlotterState startState = PlotterState::Lowering);
    PlotterState GetCurrentState() const;
    void SetState(PlotterState newState);
    void HandleEvent(PlotterEvent event);
};
