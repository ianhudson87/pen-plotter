#include "MotorDriver.cpp"
#include "MovementPlanner.cpp"
#include "Vector2D.cpp"
#include "PlotterStateMachine.cpp"
#include "HangPlotterManager.cpp"

// CONSTANTS
int leftMotorPins[4] = {13, 12, 14, 27};
int rightMotorPins[4] = {26, 25, 33, 32};

MotorDriver leftMotor(leftMotorPins);
MotorDriver rightMotor(rightMotorPins);

Button stateChangeButton(35); // pin 35
Button retractButton(34); // pin 34

HangPlotterManager hangPlotterManager(PlotterState::Lowering);
MovementPlanner movementPlanner(plotterHeadStartingX, plotterHeadStartingY);
PlotterStateMachine plotterStateMachine();
// END CONSTANTS

void setup() {
  Serial.begin(115200);
}

void loop() {
  stateChangeButton.ReadPin();
  retractButton.ReadPin();

  //////////////////////////////////////////////////
  // Start handle current state
  //////////////////////////////////////////////////
  if(plotterStateMachine.GetCurrentState() == PlotterState::Lowering)
  {
    rightMotor.DoStep(false); // counter-clockwise
    leftMotor.DoStep(true); // clockwise
    delay(5);
  }
  else if(plotterStateMachine.GetCurrentState() == PlotterState::LeftRetracting)
  {
    
    if(digitalRead(retractButtonPin) == HIGH)
    {
      leftMotor.DoStep(false);
      delay(5);
    }
  }
  else if(plotterStateMachine.GetCurrentState() == PlotterState::RightRetracting)
  {
    if(digitalRead(retractButtonPin) == HIGH)
    {
      rightMotor.DoStep(true);
      delay(5);
    }
  }
  else if(plotterStateMachine.GetCurrentState() == PlotterState::Calculating)
  {
    Vector2D nextPos = movementPlanner.GetNextPos()
    Vector2D targetLengths = hangPlotterManager.GetTargetLengths(nextPos);
    Vector2D rotations = hangPlotterManager.GetMotorRotations(targetLengths);
    Vector2D rotationSpeeds = hangPlotterManager.GetMotorRotationSpeeds(rotations);

    hangPlotterManager.SetCurrentLengths(targetLengths); // Keep track of what lengths the plotter will have after movement is complete

    leftMotor.QueueRotation(leftMotorRotation, leftMotorSpeed);
    rightMotor.QueueRotation(rightMotorRotation, rightMotorSpeed);
  }
  else if(plotterStateMachine.GetCurrentState() == PlotterState::Calculating)
  {
    rightMotor.ProcessRotation();
    leftMotor.ProcessRotation();
  }
  else if(plotterStateMachine.GetCurrentState() == PlotterState::Reset)
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
  if(stateChangeButton.IsStateLowToHigh())
  {
    plotterStateMachine.HandleEvent(PlotterEvent::ButtonPress)
  }
  else if(leftMotor.IsDoneRotating() && rightMotor.IsDoneRotating())
  {
    plotterStateMachine.HandleEvent(PlotterEvent::MotorRotationComplete)
  }
  else
  {
    plotterStateMachine.HandleEvent(PlotterEvent::LoopComplete)
  }
  //////////////////////////////////////////////////
  // End handle state change.
  //////////////////////////////////////////////////
}









