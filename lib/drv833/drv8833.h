#pragma once
#include <Arduino.h>

// DRV8833: mỗi bánh dùng 2 chân IN (PWM trực tiếp trên IN, không có chân PWM riêng
// như TB6612). speed âm = lùi, speed dương = tiến, 0 = thả trôi.
class DRV8833 {
public:
  void begin();
  void run(int speedLeft, int speedRight);  // -255..255
  void stop();                               // thả trôi (coast)
  void brake();                              // phanh cứng (short brake)
  void sleep(bool on);                       // true = ngủ (tắt IC), false = thức

private:
  void driveOne(int in1Pin, int in2Pin, int speed);
};

extern DRV8833 motor;