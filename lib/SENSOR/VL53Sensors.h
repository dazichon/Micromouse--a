#pragma once
#include <Arduino.h>
#include <VL53L0X.h>   // lib_deps: pololu/VL53L0X

struct WallDist {
  int front_mm, left_mm, right_mm;
  bool front_ok, left_ok, right_ok;
};

class VL53Sensors {
public:
  bool begin();                 // set địa chỉ I2C riêng cho từng cảm biến qua XSHUT
  WallDist read();               // đọc cả 3, đơn vị mm
  bool frontWall(int thresholdMm = -1);
  bool leftWall(int thresholdMm = -1);
  bool rightWall(int thresholdMm = -1);

private:
  VL53L0X _front, _left, _right;
  bool _ok = false;
};

extern VL53Sensors tof;