#include <MotorDriver.h>
#include <Vector2D.h>
#include <MovementPlanner.h>
#include <Button.h>
#include <PlotterStateMachine.h>
#include <HangPlotterManager.h>

// CONSTANTS
int leftMotorPins[4] = {13, 12, 14, 27};
int rightMotorPins[4] = {26, 25, 33, 32};

MotorDriver leftMotor(leftMotorPins);
MotorDriver rightMotor(rightMotorPins);

Button stateChangeButton(35); // pin 35
Button retractButton(34); // pin 34

Vector2D plotterHeadStartingPos(8.625, 6.88);
HangPlotterManager hangPlotterManager;
MovementPlanner movementPlanner(plotterHeadStartingPos);
PlotterStateMachine plotterStateMachine(PlotterState::Lowering);
// END CONSTANTS

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  stateChangeButton.ReadPin();
  retractButton.ReadPin();

  //////////////////////////////////////////////////
  // Start handle current state
  //////////////////////////////////////////////////
  PlotterState currentState = plotterStateMachine.GetCurrentState();

  if (currentState == PlotterState::Lowering)
  {
    rightMotor.DoStep(false); // counter-clockwise
    leftMotor.DoStep(true); // clockwise
    delay(5);
  }
  else if (currentState == PlotterState::LeftRetracting)
  {
    if (retractButton.IsHigh())
    {
      leftMotor.DoStep(false);
      delay(5);
    }
  }
  else if (currentState == PlotterState::RightRetracting)
  {
    if (retractButton.IsHigh())
    {
      rightMotor.DoStep(true);
      delay(5);
    }
  }
  else if (currentState == PlotterState::Calculating)
  {
    Vector2D nextPos = movementPlanner.GetNextPos();
    Vector2D targetLengths = hangPlotterManager.GetTargetLengths(nextPos);
    Vector2D rotations = hangPlotterManager.GetMotorRotations(targetLengths);
    Vector2D rotationSpeeds = hangPlotterManager.GetMotorRotationSpeeds(rotations);

    // Keep track of what lengths the plotter will have after movement is complete.
    hangPlotterManager.SetCurrentLengths(targetLengths);

    leftMotor.QueueRotation(rotations.x, rotationSpeeds.x);
    rightMotor.QueueRotation(rotations.y, rotationSpeeds.y);
  }
  else if (currentState == PlotterState::Moving)
  {
    rightMotor.ProcessRotation();
    leftMotor.ProcessRotation();
  }
  else if (currentState == PlotterState::Reset)
  {
    movementPlanner.Reset();
    hangPlotterManager.Reset();
  }
  //////////////////////////////////////////////////
  // End handle current state
  //////////////////////////////////////////////////

  //////////////////////////////////////////////////
  // Start handle state change. Only process one event at a time.
  //////////////////////////////////////////////////
  if (stateChangeButton.IsStateLowToHigh())
  {
    plotterStateMachine.HandleEvent(PlotterEvent::ButtonPress);
  }
  else if (currentState == PlotterState::Moving && leftMotor.IsDoneRotating() && rightMotor.IsDoneRotating())
  {
    plotterStateMachine.HandleEvent(PlotterEvent::MotorRotationComplete);
  }
  else
  {
    plotterStateMachine.HandleEvent(PlotterEvent::LoopComplete);
  }
  //////////////////////////////////////////////////
  // End handle state change.
  //////////////////////////////////////////////////
}
