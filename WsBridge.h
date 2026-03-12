// WsBridge.h
#pragma once

#include <Arduino.h>
#include <ArduinoWebsockets.h>
#include <functional>
#include "Config.h"

class WsBridge {
public:
  struct DebugInfo {
    bool started = false;
    bool connected = false;
    bool everConnected = false;
    bool lastConnectCallOk = false;

    uint16_t connectAttempts = 0;
    uint8_t openEvents = 0;
    uint8_t closeEvents = 0;
    uint8_t lastEvent = 0; // 0 none, 1 open, 2 close, 3 ping, 4 pong

    String currentUrl;
  };

  WsBridge();

  void begin(const char* url);
  void loop();

  bool isConnected() const;
  bool sendText(const String& s);

  void onText(std::function<void(const String&)> cb);
  void onConnected(std::function<void()> cb);

  void close();
  void setUrl(const char* url);

  DebugInfo debugInfo() const;

private:
  void ensureConnected();
  void flushOutbox();
  void enqueue(const String& s);
  void clearOutbox();
  void installHandlersIfNeeded();

  String m_url;
  websockets::WebsocketsClient m_client;
  uint32_t m_lastConnectAttemptMs = 0;

  bool m_started = false;
  bool m_handlersInstalled = false;
  bool m_prevConnected = false;

  bool m_everConnected = false;
  bool m_lastConnectCallOk = false;
  uint16_t m_connectAttempts = 0;
  uint8_t m_openEvents = 0;
  uint8_t m_closeEvents = 0;
  uint8_t m_lastEvent = 0;

  String m_outbox[WS_OUTBOX_MAX];
  uint8_t m_head = 0;
  uint8_t m_tail = 0;
  uint8_t m_count = 0;

  std::function<void(const String&)> m_onText;
  std::function<void()> m_onConnected;
};