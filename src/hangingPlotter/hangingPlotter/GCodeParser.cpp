#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <GCodeParser.h>

void GCodeParser::Reset()
{
  currentXMillimeters = 0;
  currentYMillimeters = 0;
  isAbsoluteMode = true;
  isMillimeterMode = true;
  lastError[0] = '\0';
}

void GCodeParser::SetError(const char* message)
{
  strncpy(lastError, message, sizeof(lastError) - 1);
  lastError[sizeof(lastError) - 1] = '\0';
}

const char* GCodeParser::GetLastError() const
{
  return lastError;
}

GCodeParseResult GCodeParser::ParseLine(const char* line, Vector2D& targetPosition)
{
  lastError[0] = '\0';

  if (line == nullptr)
  {
    SetError("missing line");
    return GCodeParseResult::Error;
  }

  bool hasMoveCommand = false;
  bool hasX = false;
  bool hasY = false;
  bool hasUnsupportedCommand = false;
  double nextXMillimeters = currentXMillimeters;
  double nextYMillimeters = currentYMillimeters;
  const char* cursor = line;

  while (*cursor != '\0')
  {
    while (isspace(static_cast<unsigned char>(*cursor)))
    {
      cursor++;
    }

    if (*cursor == '\0' || *cursor == ';')
    {
      break;
    }

    if (*cursor == '(')
    {
      cursor++;
      while (*cursor != '\0' && *cursor != ')')
      {
        cursor++;
      }
      if (*cursor == ')')
      {
        cursor++;
      }
      continue;
    }

    if (!isalpha(static_cast<unsigned char>(*cursor)))
    {
      SetError("expected command word");
      return GCodeParseResult::Error;
    }

    char word = static_cast<char>(toupper(static_cast<unsigned char>(*cursor)));
    cursor++;

    while (isspace(static_cast<unsigned char>(*cursor)))
    {
      cursor++;
    }

    char* valueEnd = nullptr;
    double value = strtod(cursor, &valueEnd);
    bool hasValue = valueEnd != cursor;

    if (!hasValue)
    {
      if (word == 'G' || word == 'X' || word == 'Y')
      {
        SetError("missing numeric value");
        return GCodeParseResult::Error;
      }

      continue;
    }

    cursor = valueEnd;

    if ((word == 'G' || word == 'X' || word == 'Y') && !isfinite(value))
    {
      SetError("invalid numeric value");
      return GCodeParseResult::Error;
    }

    if (word == 'G')
    {
      int command = static_cast<int>(value);
      if (fabs(value - command) > 0.0001)
      {
        SetError("invalid G code");
        return GCodeParseResult::Error;
      }

      if (command == 0 || command == 1)
      {
        hasMoveCommand = true;
      }
      else if (command == 20)
      {
        isMillimeterMode = false;
      }
      else if (command == 21)
      {
        isMillimeterMode = true;
      }
      else if (command == 90)
      {
        isAbsoluteMode = true;
      }
      else if (command == 91)
      {
        isAbsoluteMode = false;
      }
      else
      {
        hasUnsupportedCommand = true;
      }
    }
    else if (word == 'X')
    {
      nextXMillimeters = value;
      hasX = true;
    }
    else if (word == 'Y')
    {
      nextYMillimeters = value;
      hasY = true;
    }
  }

  if (hasUnsupportedCommand || !hasMoveCommand || (!hasX && !hasY))
  {
    return GCodeParseResult::Skipped;
  }

  if (!isAbsoluteMode || !isMillimeterMode)
  {
    return GCodeParseResult::Skipped;
  }

  currentXMillimeters = nextXMillimeters;
  currentYMillimeters = nextYMillimeters;
  targetPosition.x = currentXMillimeters;
  targetPosition.y = currentYMillimeters;
  return GCodeParseResult::Move;
}
