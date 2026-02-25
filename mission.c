// mission.c
#include "mission.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char* qmarks(void) { return "?????"; }

static int wrap(char* out, size_t n, const char* sentence) {
  if (!out || n == 0) return 0;
  // "MISSION MESSAGE: " prefix matches MicroPython mission.py
  return (snprintf(out, n, "MISSION MESSAGE: %s\n", sentence) > 0);
}

static int to_int(const char* s, int* ok) {
  if (!s) { if (ok) *ok = 0; return 0; }
  char* endp = NULL;
  long v = strtol(s, &endp, 10);
  if (endp == s) { if (ok) *ok = 0; return 0; }
  if (ok) *ok = 1;
  return (int)v;
}

int mission_format(mission_team_t team,
                   int mtype,
                   const char* msg_ascii,
                   char* out,
                   size_t out_len) {
  if (!out || out_len == 0) return 0;

  int ok = 0;
  int msg = to_int(msg_ascii, &ok);

  // If we can't parse msg into int, we'll still print something obvious where possible.
  // Most missions use integer enums/values anyway.

  // CRASH: type: DIRECTION=0, LENGTH=1, HEIGHT=2
  if (team == MISSION_CRASH) {
    if (mtype == 0) { // DIRECTION
      const char* direction = qmarks();
      if (ok) {
        switch (msg) {
          case 0: direction = "+x"; break;
          case 1: direction = "-x"; break;
          case 2: direction = "+y"; break;
          case 3: direction = "-y"; break;
          default: direction = qmarks(); break;
        }
      }
      char s[96];
      snprintf(s, sizeof(s),
               "The direction of the abnormality is in the %s direction.",
               direction);
      return wrap(out, out_len, s);
    }
    if (mtype == 1) { // LENGTH
      char s[96];
      snprintf(s, sizeof(s),
               "The length of the side with abnormality is %dmm.",
               ok ? msg : 0);
      return wrap(out, out_len, s);
    }
    if (mtype == 2) { // HEIGHT
      char s[96];
      snprintf(s, sizeof(s),
               "The height of the side with abnormality is %dmm.",
               ok ? msg : 0);
      return wrap(out, out_len, s);
    }
    return wrap(out, out_len,
                "The direction of the abnormality is in the ????? direction.");
  }

  // DATA: type: CYCLE=0, MAGNETISM=1
  if (team == MISSION_DATA) {
    if (mtype == 0) { // CYCLE
      char s[64];
      snprintf(s, sizeof(s),
               "The duty cycle is %d%%.",
               ok ? msg : 0);
      return wrap(out, out_len, s);
    }
    if (mtype == 1) { // MAGNETISM
      const char* m = qmarks();
      if (ok) {
        if (msg == 0) m = "MAGNETIC";
        else if (msg == 1) m = "NOT MAGNETIC";
      }
      char s[64];
      snprintf(s, sizeof(s), "The disk is %s.", m);
      return wrap(out, out_len, s);
    }
    return wrap(out, out_len, "The disk is ?????.");
  }

  // MATERIAL: type: WEIGHT=0, MATERIAL_TYPE=1
  if (team == MISSION_MATERIAL) {
    if (mtype == 0) { // WEIGHT
      const char* w = qmarks();
      if (ok) {
        if (msg == 0) w = "HEAVY";
        else if (msg == 1) w = "MEDIUM";
        else if (msg == 2) w = "LIGHT";
      }
      char s[64];
      snprintf(s, sizeof(s), "The material is %s.", w);
      return wrap(out, out_len, s);
    }
    if (mtype == 1) { // MATERIAL_TYPE
      const char* t = qmarks();
      if (ok) {
        if (msg == 0) t = "FOAM";
        else if (msg == 1) t = "PLASTIC";
      }
      char s[64];
      snprintf(s, sizeof(s), "The material is %s.", t);
      return wrap(out, out_len, s);
    }
    return wrap(out, out_len, "The material is ?????.");
  }

  // FIRE: type: NUM_CANDLES=0, TOPOGRAPHY=1
  if (team == MISSION_FIRE) {
    if (mtype == 0) { // NUM_CANDLES
      char s[80];
      snprintf(s, sizeof(s),
               "The number of candles is %d.",
               ok ? msg : 0);
      return wrap(out, out_len, s);
    }
    if (mtype == 1) { // TOPOGRAPHY
      const char* t = qmarks();
      if (ok) {
        if (msg == 0) t = "A";
        else if (msg == 1) t = "B";
        else if (msg == 2) t = "C";
      }
      char s[80];
      snprintf(s, sizeof(s),
               "The topography is %s.",
               t);
      return wrap(out, out_len, s);
    }
    return wrap(out, out_len, "The topography is ?????.");
  }

  // WATER: type: DEPTH=0, WATER_TYPE=1
  if (team == MISSION_WATER) {
    if (mtype == 0) { // DEPTH
      char s[80];
      snprintf(s, sizeof(s),
               "The depth is %dmm.",
               ok ? msg : 0);
      return wrap(out, out_len, s);
    }
    if (mtype == 1) { // WATER_TYPE
      const char* t = qmarks();
      if (ok) {
        switch (msg) {
          case 0: t = "FRESH UNPOLLUTED"; break;
          case 1: t = "FRESH POLLUTED"; break;
          case 2: t = "SALTY UNPOLLUTED"; break;
          case 3: t = "SALTY POLLUTED"; break;
          default: t = qmarks(); break;
        }
      }
      char s[96];
      snprintf(s, sizeof(s), "The water is %s.", t);
      return wrap(out, out_len, s);
    }
    return wrap(out, out_len, "The water is ?????.");
  }

  // SEED: type: LOCATION=0
  if (team == MISSION_SEED) {
    if (mtype == 0) { // LOCATION
      const char* loc = qmarks();
      if (ok) {
        switch (msg) {
          case 0: loc = "BOTH"; break;
          case 1: loc = "NEITHER"; break;
          case 2: loc = "ADJACENT"; break;
          case 3: loc = "DIAGONAL"; break;
          default: loc = qmarks(); break;
        }
      }
      char s[96];
      snprintf(s, sizeof(s), "The plot locations are %s.", loc);
      return wrap(out, out_len, s);
    }
    return wrap(out, out_len, "The plot locations are ?????.");
  }

  // HYDROGEN: type: VOLTAGE_OUTPUT=0, LED_COLOR=1
  if (team == MISSION_HYDROGEN) {
    if (mtype == 0) { // VOLTAGE_OUTPUT
      char s[96];
      snprintf(s, sizeof(s),
               "The voltage output is %d.",
               ok ? msg : 0);
      return wrap(out, out_len, s);
    }

    if (mtype == 1) { // LED_COLOR
      const char* c = qmarks();
      if (ok) {
        switch (msg) {
          case 0: c = "WHITE"; break;
          case 1: c = "RED"; break;
          case 2: c = "YELLOW"; break;
          case 3: c = "GREEN"; break;
          case 4: c = "BLUE"; break;
          default: c = qmarks(); break;
        }
      }
      char s[80];
      snprintf(s, sizeof(s), "The LED color is %s.", c);
      return wrap(out, out_len, s);
    }

    return wrap(out, out_len, "The voltage output is ?????.");
  }

  // Unknown mission/teamType
  return wrap(out, out_len, "Unknown mission.");
}