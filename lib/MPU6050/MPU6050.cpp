#include "MPU6050.h"
#include "RobotConfig.h"
#include <Wire.h>

MPU6050Gyro gyro;

bool MPU6050Gyro::begin() {
  // Wire.begin() đã được gọi bởi VL53Sensors::begin() dùng chung bus I2C,
  // nhưng gọi lại ở đây cũng an toàn nếu MPU6050 init trước.
  Wire.begin(I2C_SDA, I2C_SCL);

  _ok = _mpu.begin();
  if (!_ok) {
    Serial.println("MPU6050 init FAIL");
    return false;
  }

  _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrate();
  _lastUs = micros();
  return true;
}

void MPU6050Gyro::calibrate(int samples) {
  if (!_ok) return;
  float sum = 0;
  sensors_event_t a, g, temp;
  for (int i = 0; i < samples; i++) {
    _mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(2);
  }
  _gyroZOffset = sum / samples;
}

void MPU6050Gyro::update() {
  if (!_ok) return;

  unsigned long now = micros();
  float dt = (now - _lastUs) * 1e-6f;
  _lastUs = now;
  if (dt <= 0 || dt > 0.5f) return;   // bỏ qua mẫu dt bất thường

  sensors_event_t a, g, temp;
  _mpu.getEvent(&a, &g, &temp);

  float gz_dps = (g.gyro.z - _gyroZOffset) * 180.0f / PI;  // rad/s -> deg/s
  _yaw += gz_dps * dt;
}

float MPU6050Gyro::yawDeg() const { return _yaw; }
void MPU6050Gyro::resetYaw() { _yaw = 0; }