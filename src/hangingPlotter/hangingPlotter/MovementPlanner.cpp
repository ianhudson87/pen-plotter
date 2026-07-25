#include "Vector2D.cpp"

class MovementPlanner
{
  private:
    int spiralIterations = 0;
    Vector2D startPos;
    Vector2D currentPos;

  public:
    MovementPlanner (Vector2D startingPosition)
      : startPos(startingPosition), currentPos(startingPosition)
    {
    }

    void Reset()
    {
      spiralIterations = 0;
      currentPos = startPos;
    }

    Vector2D GetNextPos()
    {
      float distanceChange = 0.5 * (spiralIterations / 2 + 1);
      Vector2D unitChange;
      switch(spiralIterations % 4)
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
    }
};
