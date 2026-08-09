#include <MovementPlanner.h>

MovementPlanner::MovementPlanner(Vector2D startingPosition, Vector2D gcodeOriginOffset, GCodeServer& server)
  : startPos(startingPosition),
    currentPos(startingPosition),
    gcodeOriginOffset(gcodeOriginOffset),
    gcodeServer(server)
{
}

void MovementPlanner::SetCurrentPos(Vector2D pos)
{
  this->startPos = pos;
  this->currentPos = pos;
}

void MovementPlanner::Reset()
{
  if (moveAwaitingAcknowledgement)
  {
    gcodeServer.SendResponse("ERR reset");
  }

  currentPos = startPos;
  gcodeParser.Reset();
  moveAwaitingAcknowledgement = false;
}

Vector2D MovementPlanner::GetNextPos()
{
  if (moveAwaitingAcknowledgement)
  {
    gcodeServer.SendResponse("OK");
    moveAwaitingAcknowledgement = false;
  }

  while (true)
  {
    gcodeServer.Poll();

    if (!gcodeServer.HasLine())
    {
      delay(1);
      continue;
    }

    Vector2D gcodePosition;
    GCodeParseResult parseResult = gcodeParser.ParseLine(gcodeServer.GetLine(), gcodePosition);
    gcodeServer.ConsumeLine();

    if (parseResult == GCodeParseResult::Move)
    {
      currentPos = gcodeOriginOffset + gcodePosition;
      moveAwaitingAcknowledgement = true;
      return currentPos;
    }

    if (parseResult == GCodeParseResult::Skipped)
    {
      gcodeServer.SendResponse("OK SKIPPED");
      continue;
    }

    char response[64];
    snprintf(response, sizeof(response), "ERR %s", gcodeParser.GetLastError());
    gcodeServer.SendResponse(response);
  }

  /*
  Previous spiral movement generation:

  float distanceChange = 5.0 * (spiralIterations / 2 + 1);
  Vector2D unitChange;
  switch (spiralIterations % 4)
  {
    case 0:
      unitChange = Vector2D{1, 0};
      break;
    case 1:
      unitChange = Vector2D{0, -1};
      break;
    case 2:
      unitChange = Vector2D{-1, 0};
      break;
    case 3:
      unitChange = Vector2D{0, 1};
      break;
  }

  spiralIterations++;
  this->currentPos += (unitChange * distanceChange);

  return this->currentPos;
  */
}
