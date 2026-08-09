#include <GCodeServer.h>

GCodeServer::GCodeServer(uint16_t port)
  : server(port)
{
}

bool GCodeServer::Begin(const char* accessPointName, const char* password)
{
  WiFi.mode(WIFI_AP);
  bool started = WiFi.softAP(accessPointName, password);
  if (!started)
  {
    return false;
  }

  server.begin();
  server.setNoDelay(true);
  return true;
}

void GCodeServer::AcceptClient()
{
  if (client && client.connected())
  {
    return;
  }

  if (client)
  {
    client.stop();
  }

  WiFiClient waitingClient = server.available();
  if (!waitingClient)
  {
    return;
  }

  client = waitingClient;
  client.setNoDelay(true);
  ResetLine();
}

void GCodeServer::ResetLine()
{
  lineLength = 0;
  lineReady = false;
  discardingLongLine = false;
  lineBuffer[0] = '\0';
}

void GCodeServer::Poll()
{
  AcceptClient();

  if (!client || !client.connected() || lineReady)
  {
    return;
  }

  while (client.available() > 0 && !lineReady)
  {
    char nextCharacter = static_cast<char>(client.read());

    if (nextCharacter == '\r')
    {
      continue;
    }

    if (nextCharacter == '\n')
    {
      if (discardingLongLine)
      {
        SendResponse("ERR line too long");
        ResetLine();
        continue;
      }

      lineBuffer[lineLength] = '\0';
      lineReady = true;
      continue;
    }

    if (discardingLongLine)
    {
      continue;
    }

    if (lineLength >= lineBufferSize - 1)
    {
      discardingLongLine = true;
      continue;
    }

    lineBuffer[lineLength] = nextCharacter;
    lineLength++;
  }
}

bool GCodeServer::HasLine() const
{
  return lineReady;
}

const char* GCodeServer::GetLine() const
{
  return lineBuffer;
}

void GCodeServer::ConsumeLine()
{
  ResetLine();
}

void GCodeServer::SendResponse(const char* response)
{
  if (client && client.connected())
  {
    client.println(response);
  }
}

IPAddress GCodeServer::GetIpAddress() const
{
  return WiFi.softAPIP();
}
