#pragma once
#include <Arduino.h>

class Encoder {
public:
  void begin();                 // gắn interrupt cho cả 2 bánh
  void reset();                 // reset đếm xung về 0
  long left() const;            // xung tích lũy bánh trái
  long right() const;           // xung tích lũy bánh phải
  int  spinCount() const;       // trung bình |left|+|right| /2, dùng khi quay tại chỗ

  float distanceLeftCm() const;
  float distanceRightCm() const;
  float distanceAvgCm() const;

  // dùng bởi ISR — public vì attachInterrupt cần hàm rảnh (static wrapper trong .cpp)
  static Encoder* instance;
  void isrLeft();
  void isrRight();

private:
  volatile long _left = 0;
  volatile long _right = 0;
};

extern Encoder encoder;