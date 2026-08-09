#pragma once

#include <Arduino.h>
#include <WiFi.h>

class GCodeServer
{
  private:
    static const size_t lineBufferSize = 192;

    WiFiServer server;
    WiFiClient client;
    char lineBuffer[lineBufferSize] = {0};
    size_t lineLength = 0;
    bool lineReady = false;
    bool discardingLongLine = false;

    void AcceptClient();
    void ResetLine();

  public:
    explicit GCodeServer(uint16_t port);
    bool Begin(const char* accessPointName, const char* password);
    void Poll();
    bool HasLine() const;
    const char* GetLine() const;
    void ConsumeLine();
    void SendResponse(const char* response);
    IPAddress GetIpAddress() const;
};
