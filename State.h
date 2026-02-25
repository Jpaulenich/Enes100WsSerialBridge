#pragma once

#include <Arduino.h>

struct EnesState {
  char teamName[50] = {0};
  uint8_t teamType = 0;
  int arucoId = 0;
  int room = 0;

  // Latest ArUco data from server
  double aruco_x = 0.0;
  double aruco_y = 0.0;
  double aruco_theta = 0.0;
  bool aruco_visible = false;
  bool newArucoData = false;
};

EnesState& enesState();
