// WsBridge.h
#pragma once

#include <Arduino.h>
#include <ArduinoWebsockets.h>
#include <functional>

// IMPORTANT: include Config so WS_OUTBOX_MAX is visible here
#include "Config.h"

class WsBridge {
public:
  WsBridge();

  void begin(const char* url);
  void loop();

  bool isConnected();
  bool sendText(const String& s);

  void onText(std::function<void(const String&)> cb);
  void onConnected(std::function<void()> cb);

  void close();
  void setUrl(const char* url);

private:
  void ensureConnected();
  void flushOutbox();
  void enqueue(const String& s);
  void clearOutbox();

  String m_url;
  websockets::WebsocketsClient m_client;
  uint32_t m_lastConnectAttemptMs = 0;

  bool m_prevConnected = false;

  // Outbox ring buffer
  String m_outbox[WS_OUTBOX_MAX];
  uint8_t m_head = 0;
  uint8_t m_tail = 0;
  uint8_t m_count = 0;

  std::function<void(const String&)> m_onText;
  std::function<void()> m_onConnected;
};