// Enes100WifiModule.h
#pragma once

#include <Arduino.h>
#include "WsBridge.h"

class Enes100WifiModule {
public:
  explicit Enes100WifiModule(WsBridge& ws);

  void begin();
  void loop();

  uint16_t currentRoom() const { return m_room; }

private:
  // VisionSystemClient opcodes (kept)
  static constexpr uint8_t OP_BEGIN        = 0x01;
  static constexpr uint8_t OP_PRINT        = 0x02;
  static constexpr uint8_t OP_CHECK        = 0x03;
  static constexpr uint8_t OP_MISSION      = 0x04;
  static constexpr uint8_t OP_ML_PRED      = 0x05;
  static constexpr uint8_t OP_ML_CAPTURE   = 0x06;
  static constexpr uint8_t OP_IS_CONNECTED = 0x07;

  bool readByte(uint8_t& out, uint32_t timeoutMs);
  bool readBytes(uint8_t* out, size_t n, uint32_t timeoutMs);
  bool readUntilNull(String& out, uint32_t timeoutMs);
  bool readFlush(uint32_t timeoutMs);

  void handleOpcode(uint8_t op);
  void handleBegin();
  void handlePrint();
  void handleMission();
  void handleCheck();
  void handleIsConnected();
  void handleMlPrediction();
  void handleMlCapture();

  // WS handlers
  void onWsText(const String& s);
  void onWsConnected();
  void parseArucoUpdate(const String& s);
  void parsePingUpdate(const String& s);

  // Manual ping + pose loops
  void pingLoop();
  void sendPing(const char* status);
  void poseRequestLoop();
  void requestArucoOnce();

  // Begin resend
  void sendBeginPacket();

  // Helpers
  String jsonEscape(const String& s) const;
  void sendCheckResponse(bool hasNew);
  void encodeAndSendLocation();

  // Boot flush only (fixed so it DOESN'T discard a real opcode)
  void bootFlushLoop();
  bool isValidOpcode(uint8_t b) const;

private:
  WsBridge& m_ws;

  String  m_teamName;
  uint8_t m_teamType = 0;
  uint16_t m_markerId = 0;
  uint16_t m_room = 0;

  bool  m_visible = false;
  float m_x = -1.0f;
  float m_y = -1.0f;
  float m_theta = -1.0f;

  uint32_t m_arucoSeq = 0;
  uint32_t m_lastCheckSeqSent = 0;

  // Ping bookkeeping
  uint32_t m_lastPingMs = 0;
  uint8_t  m_missedPongs = 0;

  // Pose request bookkeeping
  uint32_t m_lastPoseReqMs = 0;

  // Begin known?
  bool m_hasBegin = false;

  // Boot flush state
  bool m_bootFlushed = false;
  uint32_t m_bootStartMs = 0;

  // If we see a valid opcode during boot flush, stash it and process immediately.
  bool m_pendingOpValid = false;
  uint8_t m_pendingOp = 0;
};