class Helpers
{
  public:
    static double GetR2Distance(Coordinates coord1, Coordinates coord2)
    {
      return sqrt(pow(coord1.x, 2) + pow(coord2.x, 2));
    }
}