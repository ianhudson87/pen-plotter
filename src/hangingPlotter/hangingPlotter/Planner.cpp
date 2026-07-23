#include "Vector2D.cpp"

class Planner
{
  private:
    int spiralIterations = 0;
    Vector2D currentPos;

  public:
    Planner (Vector2D startingPosition)
    {
      this->currentPos = startingPosition;
    }

    void SetCurrentPos(Vector2D pos)
    {
      this->currentPos = pos;
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

      this->currentPos += (unitChange *= distanceChange)

      return this->currentPos;
    }
}