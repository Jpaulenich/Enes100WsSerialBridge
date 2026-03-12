// WsBridge.cpp
#include "WsBridge.h"
#include "Config.h"
#include <ESP8266WiFi.h>

using namespace websockets;

WsBridge::WsBridge() {}

void WsBridge::installHandlersIfNeeded() {
  if (m_handlersInstalled) return;

  m_client.onMessage([this](WebsocketsMessage message) {
    if (message.isText()) {
      if (m_onText) m_onText(message.data());
    }
  });

  m_client.onEvent([this](WebsocketsEvent event, String data) {
    (void)data;
    switch (event) {
      case WebsocketsEvent::ConnectionOpened:
        m_lastEvent = 1;
        m_openEvents++;
        m_everConnected = true;
        break;
      case WebsocketsEvent::ConnectionClosed:
        m_lastEvent = 2;
        m_closeEvents++;
        break;
      case WebsocketsEvent::GotPing:
        m_lastEvent = 3;
        break;
      case WebsocketsEvent::GotPong:
        m_lastEvent = 4;
        break;
    }
  });

  m_handlersInstalled = true;
}

void WsBridge::begin(const char* url) {
  installHandlersIfNeeded();

  m_started = true;
  m_url = (url != nullptr) ? String(url) : String("");
  m_prevConnected = false;

  if (m_url.length() > 0) {
    ensureConnected();
  }
}

bool WsBridge::isConnected() const {
  if (!m_started) return false;
  return const_cast<WebsocketsClient&>(m_client).available();
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

  if (m_count >= WS_OUTBOX_MAX) {
    m_tail = (uint8_t)((m_tail + 1) % WS_OUTBOX_MAX);
    m_count--;
  }

  m_outbox[m_head] = s;
  m_head = (uint8_t)((m_head + 1) % WS_OUTBOX_MAX);
  m_count++;
}

bool WsBridge::sendText(const String& s) {
  if (isConnected()) {
    if (m_client.send(s)) return true;
  }

  enqueue(s);
  return false;
}

void WsBridge::flushOutbox() {
  if (!isConnected()) return;

  while (m_count > 0) {
    const String& msg = m_outbox[m_tail];
    if (!m_client.send(msg)) {
      return;
    }
    m_tail = (uint8_t)((m_tail + 1) % WS_OUTBOX_MAX);
    m_count--;
    yield();
  }
}

void WsBridge::close() {
  if (!m_started) return;
  m_client.close();
  m_lastConnectAttemptMs = 0;
}

void WsBridge::setUrl(const char* url) {
  String nu = (url != nullptr) ? String(url) : String("");
  if (nu == m_url && m_started) return;

  m_url = nu;
  m_started = true;

  clearOutbox();
  close();
}

void WsBridge::ensureConnected() {
  if (!m_started) return;
  if (m_url.length() == 0) return;
  if (!WiFi.isConnected()) return;
  if (isConnected()) return;

  const uint32_t now = millis();
  if (now - m_lastConnectAttemptMs < WS_RECONNECT_INTERVAL_MS) return;
  m_lastConnectAttemptMs = now;

  m_client.close();
  m_connectAttempts++;
  m_lastConnectCallOk = m_client.connect(m_url);
}

void WsBridge::loop() {
  if (!m_started) return;

  if (WiFi.isConnected()) {
    ensureConnected();
    m_client.poll();
  }

  const bool connectedNow = isConnected();

  if (connectedNow) {
    if (!m_prevConnected) {
      m_prevConnected = true;
      if (m_onConnected) m_onConnected();
    }
    flushOutbox();
  } else {
    m_prevConnected = false;
  }
}

WsBridge::DebugInfo WsBridge::debugInfo() const {
  DebugInfo d;
  d.started = m_started;
  d.connected = const_cast<WsBridge*>(this)->isConnected();
  d.everConnected = m_everConnected;
  d.lastConnectCallOk = m_lastConnectCallOk;
  d.connectAttempts = m_connectAttempts;
  d.openEvents = m_openEvents;
  d.closeEvents = m_closeEvents;
  d.lastEvent = m_lastEvent;
  d.currentUrl = m_url;
  return d;
}