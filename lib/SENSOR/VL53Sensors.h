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
  WallDist read();               // KHÔNG chặn: lấy mẫu mới nếu có, còn lại trả giá trị cache (mm)
  WallDist readFresh(uint32_t timeoutMs = 120);  // bỏ mẫu cũ, chờ cả 3 cảm biến đo mẫu MỚI
  bool frontWall(int thresholdMm = -1);
  bool leftWall(int thresholdMm = -1);
  bool rightWall(int thresholdMm = -1);

private:
  struct Chan {
    VL53L0X* dev;
    bool inited = false;
    int mm = 8190;
    uint32_t stampMs = 0;      // thời điểm nhận mẫu gần nhất
  };
  bool poll(Chan& c);          // true nếu vừa lấy được mẫu mới
  bool fresh(const Chan& c) const;
  WallDist snapshot() const;

  VL53L0X _front, _left, _right;
  Chan _cf{&_front}, _cl{&_left}, _cr{&_right};
  bool _ok = false;
};

extern VL53Sensors tof;