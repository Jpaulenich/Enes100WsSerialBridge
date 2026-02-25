#include "State.h"

static EnesState g_state;

EnesState& enesState() {
  return g_state;
}
