#pragma once

#include <Vector2D.h>

class MovementPlanner
{
  private:
    int spiralIterations = 0;
    Vector2D startPos;
    Vector2D currentPos;

  public:
    explicit MovementPlanner(Vector2D startingPosition);
    void SetCurrentPos(Vector2D pos);
    void Reset();
    Vector2D GetNextPos();
};
