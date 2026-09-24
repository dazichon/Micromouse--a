#pragma once
#include <Arduino.h>
#include <Adafruit_MPU6050.h>   // lib_deps: adafruit/Adafruit MPU6050, adafruit/Adafruit Unified Sensor

class MPU6050Gyro {
public:
  bool begin();
  void update();          // gọi liên tục trong loop() để tích phân góc yaw
  void calibrate(int samples = 300);  // đo offset lúc đứng yên
  float yawDeg() const;   // góc quay tích lũy quanh trục thẳng đứng (độ)
  void resetYaw();

private:
  Adafruit_MPU6050 _mpu;
  float _yaw = 0;
  float _gyroZOffset = 0;
  unsigned long _lastUs = 0;
  bool _ok = false;
};

extern MPU6050Gyro gyro;