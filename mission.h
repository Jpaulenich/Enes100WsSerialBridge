// mission.h
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// teamType mapping (byte from OP_BEGIN). Assumed order matches missions:
typedef enum {
  MISSION_CRASH    = 0,
  MISSION_DATA     = 1,
  MISSION_MATERIAL = 2,
  MISSION_FIRE     = 3,
  MISSION_WATER    = 4,
  MISSION_SEED     = 5,
  MISSION_HYDROGEN = 6,
  MISSION_UNKNOWN  = 255
} mission_team_t;

// Formats the mission print sentence into out (null-terminated).
// Returns 1 if a sentence was produced, 0 on failure.
int mission_format(mission_team_t team,
                   int mtype,
                   const char* msg_ascii,
                   char* out,
                   size_t out_len);

#ifdef __cplusplus
}
#endif