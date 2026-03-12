// Enes100WifiModule.cpp
#include "Enes100WifiModule.h"
#include "Config.h"
#include <ESP8266WiFi.h>
#include <math.h>

extern "C" {
#include "mission.h"
}

static const char* teamTypeToString(uint8_t t) {
  switch (t) {
    case 0: return "CRASH_SITE";
    case 1: return "DATA";
    case 2: return "MATERIAL";
    case 3: return "FIRE";
    case 4: return "WATER";
    case 5: return "SEED";
    case 6: return "HYDROGEN";
    default: return "UNKNOWN";
  }
}

Enes100WifiModule::Enes100WifiModule(WsBridge& ws) : m_ws(ws) {}

void Enes100WifiModule::begin() {
  m_ws.onText([this](const String& s) { onWsText(s); });
  m_ws.onConnected([this]() { onWsConnected(); });

  m_lastPingMs = millis();
  m_missedPongs = 0;
  m_lastPoseReqMs = 0;

  m_bootStartMs = millis();
  m_bootFlushed = false;

  m_pendingOpValid = false;
  m_pendingOp = 0;
}

void Enes100WifiModule::setRouteDebug(uint8_t routeIndex, uint8_t routeCount, bool routeIsFallback) {
  m_routeIndex = routeIndex;
  m_routeCount = routeCount;
  m_routeIsFallback = routeIsFallback;
}

void Enes100WifiModule::setWifiDebug(uint8_t lastDisconnectReason,
                                     uint16_t wifiBeginCount,
                                     uint16_t wifiGotIpCount,
                                     bool wifiJoinInProgress,
                                     uint8_t wifiJoinAgeSec) {
  m_lastDisconnectReason = lastDisconnectReason;
  m_wifiBeginCount = wifiBeginCount;
  m_wifiGotIpCount = wifiGotIpCount;
  m_wifiJoinInProgress = wifiJoinInProgress;
  m_wifiJoinAgeSec = wifiJoinAgeSec;
}

bool Enes100WifiModule::isValidOpcode(uint8_t b) const {
  switch (b) {
    case OP_BEGIN:
    case OP_PRINT:
    case OP_CHECK:
    case OP_MISSION:
    case OP_ML_PRED:
    case OP_ML_CAPTURE:
    case OP_IS_CONNECTED:
    case OP_DEBUG_STATUS:
      return true;
    default:
      return false;
  }
}

void Enes100WifiModule::discardInputUntilIdle(uint32_t quietMs, uint32_t maxMs) {
  const uint32_t start = millis();
  uint32_t lastRx = millis();

  while ((millis() - start) < maxMs) {
    bool consumed = false;
    while (Serial.available() > 0) {
      Serial.read();
      consumed = true;
      lastRx = millis();
      yield();
    }

    if (!consumed && (millis() - lastRx) >= quietMs) {
      return;
    }

    yield();
  }
}

void Enes100WifiModule::bootFlushLoop() {
  if (m_bootFlushed) return;

  const uint32_t now = millis();
  const uint32_t elapsed = now - m_bootStartMs;
  const uint32_t windowMs = BOOT_QUIET_TIME_MS + BOOT_FLUSH_TIME_MS;

  if (elapsed < windowMs) {
    while (Serial.available() > 0) {
      int c = Serial.read();
      if (c < 0) break;
      uint8_t b = (uint8_t)c;

      if (isValidOpcode(b)) {
        m_pendingOpValid = true;
        m_pendingOp = b;
        m_bootFlushed = true;
        return;
      }
    }
    return;
  }

  m_bootFlushed = true;
}

bool Enes100WifiModule::readByte(uint8_t& out, uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (Serial.available() <= 0) {
    if (millis() - start > timeoutMs) return false;
    yield();
  }
  out = (uint8_t)Serial.read();
  return true;
}

bool Enes100WifiModule::readBytes(uint8_t* out, size_t n, uint32_t timeoutMs) {
  for (size_t i = 0; i < n; i++) {
    if (!readByte(out[i], timeoutMs)) return false;
  }
  return true;
}

bool Enes100WifiModule::readUntilNull(String& out, uint32_t timeoutMs) {
  out = "";
  while (true) {
    uint8_t b;
    if (!readByte(b, timeoutMs)) return false;
    if (b == 0x00) return true;
    out += (char)b;
    if (out.length() > 512) return false;
  }
}

bool Enes100WifiModule::readFlush(uint32_t timeoutMs) {
  uint8_t seq[4];
  if (!readBytes(seq, 4, timeoutMs)) return false;
  return true;
}

String Enes100WifiModule::jsonEscape(const String& s) const {
  String o;
  o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\\') o += "\\\\";
    else if (c == '"') o += "\\\"";
    else if (c == '\n') o += "\\n";
    else if (c == '\r') o += "\\r";
    else if (c == '\t') o += "\\t";
    else o += c;
  }
  return o;
}

void Enes100WifiModule::handleIsConnected() {
  Serial.write(m_ws.isConnected() ? (uint8_t)0x01 : (uint8_t)0x00);
}

void Enes100WifiModule::handleDebugStatus() {
  const WsBridge::DebugInfo dbg = m_ws.debugInfo();
  const IPAddress ip = WiFi.localIP();

  uint8_t flags = 0;
  if (WiFi.isConnected())      flags |= 0x01;
  if (dbg.started)             flags |= 0x02;
  if (dbg.connected)           flags |= 0x04;
  if (m_hasBegin)              flags |= 0x08;
  if (dbg.everConnected)       flags |= 0x10;
  if (dbg.lastConnectCallOk)   flags |= 0x20;
  if (m_routeIsFallback)       flags |= 0x40;
  if (m_wifiJoinInProgress)    flags |= 0x80;

  int rssi = WiFi.isConnected() ? WiFi.RSSI() : 0;
  if (rssi < -128) rssi = -128;
  if (rssi > 127) rssi = 127;

  String url = dbg.currentUrl;
  if (url.length() > 63) {
    url = url.substring(0, 63);
  }

  // Packet v2:
  // [0]  0xA5
  // [1]  0x02
  // [2]  flags
  // [3]  WiFi.status()
  // [4]  room hi
  // [5]  room lo
  // [6]  routeIndex
  // [7]  routeCount
  // [8]  connectAttempts lo
  // [9]  connectAttempts hi
  // [10] openEvents
  // [11] closeEvents
  // [12] lastEvent
  // [13] rssi (signed byte)
  // [14..17] local IP
  // [18] lastDisconnectReason
  // [19] wifiBeginCount lo
  // [20] wifiBeginCount hi
  // [21] wifiGotIpCount lo
  // [22] wifiGotIpCount hi
  // [23] wifiJoinAgeSec
  // [24] urlLen
  // [25..] url bytes
  Serial.write((uint8_t)0xA5);
  Serial.write((uint8_t)0x02);
  Serial.write(flags);
  Serial.write((uint8_t)WiFi.status());
  Serial.write((uint8_t)(m_room >> 8));
  Serial.write((uint8_t)(m_room & 0xFF));
  Serial.write(m_routeIndex);
  Serial.write(m_routeCount);
  Serial.write((uint8_t)(dbg.connectAttempts & 0xFF));
  Serial.write((uint8_t)((dbg.connectAttempts >> 8) & 0xFF));
  Serial.write(dbg.openEvents);
  Serial.write(dbg.closeEvents);
  Serial.write(dbg.lastEvent);
  Serial.write((uint8_t)((int8_t)rssi));
  Serial.write(ip[0]);
  Serial.write(ip[1]);
  Serial.write(ip[2]);
  Serial.write(ip[3]);
  Serial.write(m_lastDisconnectReason);
  Serial.write((uint8_t)(m_wifiBeginCount & 0xFF));
  Serial.write((uint8_t)((m_wifiBeginCount >> 8) & 0xFF));
  Serial.write((uint8_t)(m_wifiGotIpCount & 0xFF));
  Serial.write((uint8_t)((m_wifiGotIpCount >> 8) & 0xFF));
  Serial.write(m_wifiJoinAgeSec);
  Serial.write((uint8_t)url.length());
  if (url.length() > 0) {
    Serial.write((const uint8_t*)url.c_str(), url.length());
  }
}

void Enes100WifiModule::sendBeginPacket() {
  if (m_teamName.length() == 0) return;

  String pkt = "{";
  pkt += "\"op\":\"begin\",";
  pkt += "\"teamName\":\"" + jsonEscape(m_teamName) + "\",";
  pkt += "\"aruco\":" + String((int)m_markerId) + ",";
  pkt += "\"teamType\":\"";
  pkt += teamTypeToString(m_teamType);
  pkt += "\",";
  pkt += "\"room\":" + String((int)m_room);
  pkt += "}";

  m_ws.sendText(pkt);
}

void Enes100WifiModule::onWsConnected() {
  if (!m_hasBegin) return;

  m_lastPingMs = millis();
  m_missedPongs = 0;

  sendBeginPacket();
  requestArucoOnce();
}

void Enes100WifiModule::handleBegin() {
  uint8_t teamType;
  uint8_t fixed[4];

  if (!readByte(teamType, SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }
  if (!readBytes(fixed, 4, SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }

  String team;
  if (!readUntilNull(team, SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }
  if (!readFlush(SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }

  m_teamType = teamType;
  m_markerId = (uint16_t)((fixed[0] << 8) | fixed[1]);
  m_room     = (uint16_t)((fixed[2] << 8) | fixed[3]);
  m_teamName = team;
  m_hasBegin = true;

  m_visible = false;
  m_x = m_y = m_theta = -1.0f;
  m_arucoSeq = 0;
  m_lastCheckSeqSent = 0;

  m_lastPingMs = millis();
  m_missedPongs = 0;
  m_lastPoseReqMs = 0;

  sendBeginPacket();
  requestArucoOnce();
}

void Enes100WifiModule::handlePrint() {
  String msg;
  if (!readUntilNull(msg, SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }
  if (!readFlush(SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }

  String pkt = "{";
  pkt += "\"op\":\"print\",";
  pkt += "\"teamName\":\"" + jsonEscape(m_teamName) + "\",";
  pkt += "\"message\":\"" + jsonEscape(msg) + "\"";
  pkt += "}";

  m_ws.sendText(pkt);
}

void Enes100WifiModule::handleMission() {
  uint8_t type;
  if (!readByte(type, SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }

  String msg;
  if (!readUntilNull(msg, SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }
  if (!readFlush(SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }

  char line[192];
  mission_team_t team = (mission_team_t)m_teamType;

  if (!mission_format(team, (int)type, msg.c_str(), line, sizeof(line))) {
    snprintf(line, sizeof(line), "MISSION MESSAGE: type=%d msg=%s", (int)type, msg.c_str());
  }

  String pkt = "{";
  pkt += "\"op\":\"print\",";
  pkt += "\"teamName\":\"" + jsonEscape(m_teamName) + "\",";
  pkt += "\"message\":\"" + jsonEscape(String(line)) + "\"";
  pkt += "}";

  m_ws.sendText(pkt);
}

void Enes100WifiModule::sendCheckResponse(bool hasNew) {
  if (!hasNew) { Serial.write((uint8_t)0x00); return; }
  if (!m_visible) { Serial.write((uint8_t)0x01); return; }
  Serial.write((uint8_t)0x02);
  encodeAndSendLocation();
}

static uint8_t clampU8(int v) { if (v < 0) v = 0; if (v > 255) v = 255; return (uint8_t)v; }
static uint16_t clampU16(int v) { if (v < 0) v = 0; if (v > 65535) v = 65535; return (uint16_t)v; }

void Enes100WifiModule::encodeAndSendLocation() {
  const int y100 = (int)lroundf(m_y * 100.0f);
  const int x100 = (int)lroundf(m_x * 100.0f);
  const int t100 = (int)lroundf(m_theta * 100.0f);

  uint8_t yb = clampU8(y100);

  uint16_t xu = clampU16(x100);
  uint8_t xlo = (uint8_t)(xu & 0xFF);
  uint8_t xhi = (uint8_t)((xu >> 8) & 0xFF);

  int16_t ts = (int16_t)t100;
  uint8_t tlo = (uint8_t)(((uint16_t)ts) & 0xFF);
  uint8_t thi = (uint8_t)((((uint16_t)ts) >> 8) & 0xFF);

  Serial.write(yb);
  Serial.write(xlo);
  Serial.write(xhi);
  Serial.write(tlo);
  Serial.write(thi);
}

void Enes100WifiModule::requestArucoOnce() {
  if (!m_hasBegin) return;
  if (m_teamName.length() == 0) return;
  if (!m_ws.isConnected()) return;

  String pkt = "{";
  pkt += "\"op\":\"aruco\",";
  pkt += "\"teamName\":\"" + jsonEscape(m_teamName) + "\"";
  pkt += "}";

  m_ws.sendText(pkt);
}

void Enes100WifiModule::poseRequestLoop() {
  if (!m_hasBegin) return;
  if (m_teamName.length() == 0) return;
  if (!m_ws.isConnected()) return;

  const uint32_t now = millis();
  if (m_lastPoseReqMs != 0 && (now - m_lastPoseReqMs) < POSE_REQUEST_PERIOD_MS) return;
  m_lastPoseReqMs = now;

  requestArucoOnce();
}

void Enes100WifiModule::handleCheck() {
  requestArucoOnce();

  const bool hasNew = (m_arucoSeq != m_lastCheckSeqSent);
  if (hasNew) m_lastCheckSeqSent = m_arucoSeq;
  sendCheckResponse(hasNew);
}

void Enes100WifiModule::handleMlPrediction() {
  uint8_t modelIndex;
  if (!readByte(modelIndex, SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }
  (void)modelIndex;
  if (!readFlush(SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }
  Serial.write((uint8_t)0xFF);
  Serial.write((uint8_t)0xFF);
}

void Enes100WifiModule::handleMlCapture() {
  String label;
  if (!readUntilNull(label, SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }
  if (!readFlush(SERIAL_READ_TIMEOUT_MS)) {
    discardInputUntilIdle(SERIAL_RECOVERY_QUIET_MS, SERIAL_RECOVERY_MAX_MS);
    return;
  }

  String pkt = "{";
  pkt += "\"op\":\"ml_capture\",";
  pkt += "\"teamName\":\"" + jsonEscape(m_teamName) + "\",";
  pkt += "\"label\":\"" + jsonEscape(label) + "\"";
  pkt += "}";

  m_ws.sendText(pkt);
}

void Enes100WifiModule::handleOpcode(uint8_t op) {
  switch (op) {
    case OP_IS_CONNECTED: handleIsConnected(); break;
    case OP_BEGIN:        handleBegin(); break;
    case OP_PRINT:        handlePrint(); break;
    case OP_MISSION:      handleMission(); break;
    case OP_CHECK:        handleCheck(); break;
    case OP_ML_PRED:      handleMlPrediction(); break;
    case OP_ML_CAPTURE:   handleMlCapture(); break;
    case OP_DEBUG_STATUS: handleDebugStatus(); break;
    default: break;
  }
}

void Enes100WifiModule::sendPing(const char* status) {
  if (!m_hasBegin) return;
  if (m_teamName.length() == 0) return;
  if (!m_ws.isConnected()) return;

  String pkt = "{";
  pkt += "\"op\":\"ping\",";
  pkt += "\"teamName\":\"" + jsonEscape(m_teamName) + "\",";
  pkt += "\"status\":\"";
  pkt += status;
  pkt += "\"}";
  m_ws.sendText(pkt);
}

void Enes100WifiModule::pingLoop() {
  if (!m_hasBegin) return;

  const uint32_t now = millis();
  if (now - m_lastPingMs >= PING_PERIOD_MS) {
    m_lastPingMs = now;
    sendPing("ping");
    m_missedPongs++;

    if (m_missedPongs >= PING_MISS_LIMIT) {
      m_ws.close();
      m_missedPongs = 0;
    }
  }
}

void Enes100WifiModule::onWsText(const String& s) {
  parseArucoUpdate(s);
  parsePingUpdate(s);
}

static bool extractBool(const String& s, const char* key, bool& out) {
  int i = s.indexOf(key);
  if (i < 0) return false;
  int colon = s.indexOf(':', i);
  if (colon < 0) return false;
  String tail = s.substring(colon + 1);
  tail.trim();
  if (tail.startsWith("true")) { out = true; return true; }
  if (tail.startsWith("false")) { out = false; return true; }
  return false;
}

static bool extractFloat(const String& s, const char* key, float& out) {
  int i = s.indexOf(key);
  if (i < 0) return false;
  int colon = s.indexOf(':', i);
  if (colon < 0) return false;

  int j = colon + 1;
  while (j < (int)s.length() && (s[j] == ' ' || s[j] == '\t')) j++;
  int start = j;

  while (j < (int)s.length()) {
    char c = s[j];
    if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.') { j++; continue; }
    break;
  }
  if (j <= start) return false;
  out = s.substring(start, j).toFloat();
  return true;
}

void Enes100WifiModule::parseArucoUpdate(const String& s) {
  if (s.indexOf("\"op\"") < 0) return;
  if (s.indexOf("aruco") < 0) return;

  bool vis;
  float x, y, th;
  bool okV  = extractBool(s, "\"is_visible\"", vis) || extractBool(s, "\"visible\"", vis);
  bool okX  = extractFloat(s, "\"x\"", x);
  bool okY  = extractFloat(s, "\"y\"", y);
  bool okTh = extractFloat(s, "\"theta\"", th);

  if (!(okV && okX && okY && okTh)) return;

  m_visible = vis;
  m_x = x;
  m_y = y;
  m_theta = th;
  m_arucoSeq++;
}

void Enes100WifiModule::parsePingUpdate(const String& s) {
  if (s.indexOf("\"op\"") < 0) return;
  if (s.indexOf("ping") < 0) return;
  if (s.indexOf("\"op\":\"ping\"") < 0 && s.indexOf("\"op\": \"ping\"") < 0) return;

  int si = s.indexOf("\"status\"");
  if (si < 0) return;
  int colon = s.indexOf(':', si);
  if (colon < 0) return;

  String tail = s.substring(colon + 1);
  tail.trim();

  if (tail.startsWith("\"ping\"") || tail.startsWith("\"ping")) {
    sendPing("pong");
  } else if (tail.startsWith("\"pong\"") || tail.startsWith("\"pong")) {
    m_missedPongs = 0;
  }
}

void Enes100WifiModule::loop() {
  bootFlushLoop();
  if (!m_bootFlushed) return;

  if (m_pendingOpValid) {
    uint8_t op = m_pendingOp;
    m_pendingOpValid = false;
    handleOpcode(op);
  }

  pingLoop();
  poseRequestLoop();

  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;

    uint8_t b = (uint8_t)c;
    if (!isValidOpcode(b)) {
      continue;
    }

    handleOpcode(b);
    yield();
  }
}