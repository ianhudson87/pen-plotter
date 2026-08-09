#pragma once

#include <Vector2D.h>

enum class GCodeParseResult
{
  Move,
  Skipped,
  Error
};

class GCodeParser
{
  private:
    double currentXMillimeters = 0;
    double currentYMillimeters = 0;
    bool isAbsoluteMode = true;
    bool isMillimeterMode = true;
    char lastError[48] = {0};

    void SetError(const char* message);

  public:
    void Reset();
    GCodeParseResult ParseLine(const char* line, Vector2D& targetPosition);
    const char* GetLastError() const;
};
