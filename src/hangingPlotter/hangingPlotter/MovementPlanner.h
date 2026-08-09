#pragma once

#include <Vector2D.h>
#include <GCodeParser.h>
#include <GCodeServer.h>

class MovementPlanner
{
  private:
    Vector2D startPos;
    Vector2D currentPos;
    Vector2D gcodeOriginOffset;
    GCodeParser gcodeParser;
    GCodeServer& gcodeServer;
    bool moveAwaitingAcknowledgement = false;

  public:
    MovementPlanner(Vector2D startingPosition, Vector2D gcodeOriginOffset, GCodeServer& server);
    void SetCurrentPos(Vector2D pos);
    void Reset();
    Vector2D GetNextPos();
};
