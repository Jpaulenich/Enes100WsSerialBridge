// Enes100WsSerialBridge.ino
#include <Arduino.h>

#include "Config.h"
#include "WifiProvision.h"
#include "WsBridge.h"
#include "Enes100WifiModule.h"

static String wsUrlForRoom(uint16_t room) {
  const char* ip = WS_DEFAULT_IP;
  for (size_t i = 0; i < ROOM_IP_MAP_LEN; i++) {
    if (ROOM_IP_MAP[i].room == room) {
      ip = ROOM_IP_MAP[i].ip;
      break;
    }
  }

  String path = String(WS_PATH);
  if (!path.startsWith("/")) path = "/" + path;

  String url = "ws://";
  url += ip;
  url += ":";
  url += String(WS_PORT);
  url += path;
  return url;
}

WsBridge ws;
Enes100WifiModule enes(ws);

static uint16_t lastRoom = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(50);

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

  WifiProvision::applyHostname(cred.hostname);
  WifiProvision::connect(WIFI_SSID, cred.password);

  // Start WS on default server until we receive OP_BEGIN with a room
  ws.begin(wsUrlForRoom(0).c_str());
  enes.begin();

  // optional ready byte
  Serial.write((uint8_t)0x00);
}

void loop() {
  WifiProvision::ensureConnected(WIFI_SSID);

  // Keep websocket serviced (PONGs etc.)
  ws.loop();

  // Run UNO protocol + manual ping (inside Enes100WifiModule)
  enes.loop();

  // Retarget websocket server if room changed (comes from UNO begin packet)
  uint16_t roomNow = enes.currentRoom();
  if (roomNow != 0 && roomNow != lastRoom) {
    lastRoom = roomNow;
    String url = wsUrlForRoom(roomNow);

#ifdef BRIDGE_DEBUG
    Serial1.print(F("[ROOM] "));
    Serial1.print(roomNow);
    Serial1.print(F(" -> "));
    Serial1.println(url);
#endif

    ws.setUrl(url.c_str());
  }

  yield();
}