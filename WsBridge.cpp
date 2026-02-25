// WsBridge.cpp
#include "WsBridge.h"
#include "Config.h"
#include <ESP8266WiFi.h>

using namespace websockets;

WsBridge::WsBridge() {}

void WsBridge::begin(const char* url) {
  m_url = String(url);

  // Install handlers once. (Safe to call begin() again; this overwrites lambdas.)
  m_client.onMessage([this](WebsocketsMessage message) {
    if (message.isText()) {
      if (m_onText) m_onText(message.data());
    }
  });

  // Kick off async connection attempts
  m_prevConnected = false;
  ensureConnected();
}

bool WsBridge::isConnected() {
  return m_client.available();
}

void WsBridge::onText(std::function<void(const String&)> cb) {
  m_onText = cb;
}

void WsBridge::onConnected(std::function<void()> cb) {
  m_onConnected = cb;
}

void WsBridge::clearOutbox() {
  m_head = 0;
  m_tail = 0;
  m_count = 0;
}

void WsBridge::enqueue(const String& s) {
  if (WS_OUTBOX_MAX == 0) return;

  // If full, drop oldest (keeps most recent traffic)
  if (m_count >= WS_OUTBOX_MAX) {
    m_tail = (uint8_t)((m_tail + 1) % WS_OUTBOX_MAX);
    m_count--;
  }

  m_outbox[m_head] = s;
  m_head = (uint8_t)((m_head + 1) % WS_OUTBOX_MAX);
  m_count++;
}

bool WsBridge::sendText(const String& s) {
  // If connected, attempt immediate send.
  if (isConnected()) {
    bool ok = m_client.send(s);
    if (ok) return true;
    // If send fails unexpectedly, queue it.
    enqueue(s);
    return false;
  }

  // Not connected: queue and return false (but message is preserved)
  enqueue(s);
  return false;
}

void WsBridge::flushOutbox() {
  if (!isConnected()) return;
  while (m_count > 0) {
    const String& msg = m_outbox[m_tail];
    if (!m_client.send(msg)) {
      // Stop if send fails; keep remaining queued.
      return;
    }
    m_tail = (uint8_t)((m_tail + 1) % WS_OUTBOX_MAX);
    m_count--;
    yield();
  }
}

void WsBridge::close() {
  m_client.close();
  // allow a quick reconnect attempt, still respects WS_RECONNECT_INTERVAL_MS
  m_lastConnectAttemptMs = 0;
}

void WsBridge::setUrl(const char* url) {
  String nu(url);
  if (nu == m_url) return;

  m_url = nu;

  // Messages meant for old server shouldn't leak to new server
  clearOutbox();

  close(); // reconnect will happen in loop()
}

void WsBridge::ensureConnected() {
  if (!WiFi.isConnected()) return;
  if (isConnected()) return;

  const uint32_t now = millis();
  if (now - m_lastConnectAttemptMs < WS_RECONNECT_INTERVAL_MS) return;
  m_lastConnectAttemptMs = now;

  m_client.close();
  m_client.connect(m_url);
}

void WsBridge::loop() {
  ensureConnected();

  if (isConnected()) {
    m_client.poll();

    // detect connect transition
    if (!m_prevConnected) {
      m_prevConnected = true;
      if (m_onConnected) m_onConnected();
    }

    flushOutbox();
  } else {
    m_prevConnected = false;
  }
}