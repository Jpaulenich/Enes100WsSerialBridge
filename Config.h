// Config.h
#pragma once
#include <stdint.h>
#include <stddef.h>

// ===== Serial to Arduino UNO =====
static constexpr uint32_t SERIAL_BAUD = 19200;

// ===== Boot/Power-on robustness =====
static constexpr uint32_t BOOT_QUIET_TIME_MS = 80;
static constexpr uint32_t BOOT_FLUSH_TIME_MS = 50;

// ===== Parser recovery =====
static constexpr uint32_t SERIAL_RECOVERY_QUIET_MS = 12;
static constexpr uint32_t SERIAL_RECOVERY_MAX_MS   = 120;

// ===== WiFi =====
static constexpr const char* WIFI_SSID = "umd-iot";
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
static constexpr uint32_t WIFI_RETRY_BACKOFF_MS   = 3000;

// ===== WebSocket =====
static constexpr uint16_t WS_PORT = 7755;
static constexpr const char* WS_PATH = "/ws";
static constexpr const char* WS_DEFAULT_IP = "10.112.9.116";

// Room -> Vision server IP mapping
struct RoomIpEntry { uint16_t room; const char* ip; };
static constexpr RoomIpEntry ROOM_IP_MAP[] = {
  {1201, "10.112.9.116"},
  {1116, "10.112.9.114"},
  {1120, "10.112.9.115"},
};
static constexpr size_t ROOM_IP_MAP_LEN = sizeof(ROOM_IP_MAP) / sizeof(ROOM_IP_MAP[0]);

// ===== Reconnect =====
static constexpr uint32_t WS_RECONNECT_INTERVAL_MS = 1500;

// Keep routing pinned to the requested room.
// Set true only if your default IP is a known good generic gateway.
static constexpr bool WS_ALLOW_DEFAULT_FALLBACK = false;

// ===== Buffered outbox =====
static constexpr uint8_t WS_OUTBOX_MAX = 10;

// ===== Manual ping logic =====
static constexpr uint32_t PING_PERIOD_MS = 5000;
static constexpr uint8_t  PING_MISS_LIMIT = 5;

// ===== Pose request loop =====
static constexpr uint32_t POSE_REQUEST_PERIOD_MS = 250;

// ===== Serial protocol =====
static constexpr uint32_t SERIAL_READ_TIMEOUT_MS = 250;
static constexpr uint8_t  FLUSH_SEQ[4] = {0xFF, 0xFE, 0xFD, 0xFC};

// ===== Debug (optional) =====
// #define BRIDGE_DEBUG
#ifdef BRIDGE_DEBUG
static constexpr uint32_t DEBUG_BAUD = 115200;
#endif