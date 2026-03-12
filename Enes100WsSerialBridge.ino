// Enes100WsSerialBridge.ino
#include <Arduino.h>
#include <ESP8266WiFi.h>

#include "Config.h"
#include "WifiProvision.h"
#include "WsBridge.h"
#include "Enes100WifiModule.h"

static String wsUrlForIp(const char* ip) {
  String path = String(WS_PATH);
  if (!path.startsWith("/")) path = "/" + path;

  String url = "ws://";
  url += ip;
  url += ":";
  url += String(WS_PORT);
  url += path;
  return url;
}

static const char* ipForRoom(uint16_t room) {
  for (size_t i = 0; i < ROOM_IP_MAP_LEN; i++) {
    if (ROOM_IP_MAP[i].room == room) {
      return ROOM_IP_MAP[i].ip;
    }
  }
  return WS_DEFAULT_IP;
}

static String wsUrlForRoom(uint16_t room) {
  return wsUrlForIp(ipForRoom(room));
}

WsBridge ws;
Enes100WifiModule enes(ws);

static uint16_t g_lastRoom = 0;
static String g_wifiPassword;

// Pinned route state
static String g_selectedUrl;
static bool g_wsStarted = false;
static bool g_routeIsFallback = false;

// WiFi diagnostics/state
static WiFiEventHandler g_onGotIp;
static WiFiEventHandler g_onDisconnected;
static WiFiEventHandler g_onDhcpTimeout;

static uint32_t g_wifiLastBeginMs = 0;
static uint16_t g_wifiBeginCount = 0;
static uint16_t g_wifiGotIpCount = 0;
static uint8_t g_wifiLastDisconnectReason = 0;
static bool g_wifiJoinInProgress = false;
static bool g_wifiHadIpEver = false;

static void applyPinnedRoomRoute(uint16_t room) {
  g_routeIsFallback = false;

  String primaryUrl = wsUrlForRoom(room);
  String chosenUrl = primaryUrl;

  if (WS_ALLOW_DEFAULT_FALLBACK) {
    const String defaultUrl = wsUrlForIp(WS_DEFAULT_IP);
    if (primaryUrl != defaultUrl) {
      // Only allow default fallback if explicitly enabled.
      chosenUrl = primaryUrl;
    }
  }

  g_selectedUrl = chosenUrl;
  enes.setRouteDebug(0, 1, g_routeIsFallback);

#ifdef BRIDGE_DEBUG
  Serial1.print(F("[WS ROUTE] room "));
  Serial1.print(room);
  Serial1.print(F(" -> "));
  Serial1.println(g_selectedUrl);
#endif

  if (!g_wsStarted) {
    ws.begin(g_selectedUrl.c_str());
    g_wsStarted = true;
  } else {
    ws.setUrl(g_selectedUrl.c_str());
  }
}

static void startWifiConnect(bool doDisconnectFirst) {
  if (g_wifiPassword.length() == 0) return;

  if (doDisconnectFirst) {
    WiFi.disconnect();
    delay(50);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, g_wifiPassword.c_str());

  g_wifiLastBeginMs = millis();
  g_wifiBeginCount++;
  g_wifiJoinInProgress = true;

#ifdef BRIDGE_DEBUG
  Serial1.print(F("[WIFI] begin #"));
  Serial1.println(g_wifiBeginCount);
#endif
}

static void installWifiHandlers() {
  g_onGotIp = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& evt) {
    (void)evt;
    g_wifiJoinInProgress = false;
    g_wifiGotIpCount++;
    g_wifiHadIpEver = true;

#ifdef BRIDGE_DEBUG
    Serial1.print(F("[WIFI] GOT IP: "));
    Serial1.println(WiFi.localIP());
#endif
  });

  g_onDisconnected = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& evt) {
    g_wifiJoinInProgress = false;
    g_wifiLastDisconnectReason = evt.reason;
    ws.close();

#ifdef BRIDGE_DEBUG
    Serial1.print(F("[WIFI] DISCONNECTED reason="));
    Serial1.println((int)evt.reason);
#endif
  });

  g_onDhcpTimeout = WiFi.onStationModeDHCPTimeout([]() {
    g_wifiJoinInProgress = false;
    g_wifiLastDisconnectReason = 253; // synthetic: DHCP timeout
    ws.close();

#ifdef BRIDGE_DEBUG
    Serial1.println(F("[WIFI] DHCP TIMEOUT"));
#endif
  });
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);

#ifdef BRIDGE_DEBUG
  Serial1.begin(DEBUG_BAUD);
  Serial1.println();
  Serial1.println(F("=== ENES100 ESP8266 WiFi Module ==="));
#endif

  WifiProvision::Credentials cred;
  if (!WifiProvision::getCredentialsForThisDevice(cred)) {
#ifdef BRIDGE_DEBUG
    Serial1.println(F("[FATAL] No WiFi password found for this MAC."));
#endif
    while (true) { delay(1000); yield(); }
  }

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  WifiProvision::applyHostname(cred.hostname);

  enes.begin();

  g_wifiPassword = cred.password;
  installWifiHandlers();
  startWifiConnect(false);
}

void loop() {
  // Process serial first so BEGIN gets captured quickly.
  enes.loop();

  // Keep wifi debug info fresh for the Uno query path.
  uint8_t joinAgeSec = 0;
  if (g_wifiJoinInProgress) {
    uint32_t ageMs = millis() - g_wifiLastBeginMs;
    joinAgeSec = (ageMs >= 255000UL) ? 255 : (uint8_t)(ageMs / 1000UL);
  }
  enes.setWifiDebug(
    g_wifiLastDisconnectReason,
    g_wifiBeginCount,
    g_wifiGotIpCount,
    g_wifiJoinInProgress,
    joinAgeSec
  );

  // Latch the room and pin the websocket target to that room only.
  const uint16_t roomNow = enes.currentRoom();
  if (roomNow != 0 && roomNow != g_lastRoom) {
    g_lastRoom = roomNow;
    applyPinnedRoomRoute(roomNow);
  }

  // Let WiFi join attempts finish before restarting them.
  if (!WiFi.isConnected()) {
    if (g_wifiJoinInProgress) {
      if ((millis() - g_wifiLastBeginMs) >= WIFI_CONNECT_TIMEOUT_MS) {
#ifdef BRIDGE_DEBUG
        Serial1.println(F("[WIFI] JOIN TIMEOUT"));
#endif
        g_wifiJoinInProgress = false;
        g_wifiLastDisconnectReason = 254; // synthetic: join timed out
        WiFi.disconnect();
      }
    } else {
      if ((millis() - g_wifiLastBeginMs) >= WIFI_RETRY_BACKOFF_MS) {
        startWifiConnect(true);
      }
    }
  }

  // Only service websocket while connected to WiFi.
  if (WiFi.isConnected() && g_wsStarted) {
    ws.loop();
  }

  yield();
}