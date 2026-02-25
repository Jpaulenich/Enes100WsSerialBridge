// WifiProvision.h
#pragma once

#include <Arduino.h>
#include <ESP8266WiFi.h>

class WifiProvision {
public:
  struct Credentials {
    String hostname;
    String mac;
    String password;
  };

  static String getMacString();
  static bool getCredentialsForThisDevice(Credentials& out);

  static void applyHostname(const String& hostname);

  static bool connect(const char* ssid, const String& password);
  static void ensureConnected(const char* ssid);

private:
  static bool macEqualsIgnoreCase(const String& a, const String& b);
};