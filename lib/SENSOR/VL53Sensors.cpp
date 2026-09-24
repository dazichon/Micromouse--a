#include "VL53Sensors.h"
#include "RobotConfig.h"
#include <Wire.h>

VL53Sensors tof;

// địa chỉ I2C mới gán cho từng cảm biến (mặc định của VL53L0X là 0x29)
#define ADDR_FRONT 0x30
#define ADDR_LEFT  0x31
#define ADDR_RIGHT 0x32

bool VL53Sensors::begin() {
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // tắt cả 3 (giữ ở reset) trước
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  Wire.begin(I2C_SDA, I2C_SCL);

  bool ok = true;

  // bật FRONT trước, gán địa chỉ mới
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(10);
  _front.setTimeout(500);
  if (!_front.init()) { Serial.println("VL53L0X FRONT init FAIL"); ok = false; }
  _front.setAddress(ADDR_FRONT);

  // bật LEFT
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  _left.setTimeout(500);
  if (!_left.init()) { Serial.println("VL53L0X LEFT init FAIL"); ok = false; }
  _left.setAddress(ADDR_LEFT);

  // bật RIGHT
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  _right.setTimeout(500);
  if (!_right.init()) { Serial.println("VL53L0X RIGHT init FAIL"); ok = false; }
  _right.setAddress(ADDR_RIGHT);

  _front.startContinuous();
  _left.startContinuous();
  _right.startContinuous();

  _ok = ok;
  return ok;
}

WallDist VL53Sensors::read() {
  WallDist d;

  d.front_mm = _front.readRangeContinuousMillimeters();
  d.left_mm  = _left.readRangeContinuousMillimeters();
  d.right_mm = _right.readRangeContinuousMillimeters();

  d.front_ok = !_front.timeoutOccurred() && d.front_mm < 2000;
  d.left_ok  = !_left.timeoutOccurred()  && d.left_mm  < 2000;
  d.right_ok = !_right.timeoutOccurred() && d.right_mm < 2000;

  return d;
}

bool VL53Sensors::frontWall(int thresholdMm) {
  if (thresholdMm < 0) thresholdMm = WALL_THRESHOLD_MM;
  WallDist d = read();
  return d.front_ok && d.front_mm < thresholdMm;
}

bool VL53Sensors::leftWall(int thresholdMm) {
  if (thresholdMm < 0) thresholdMm = WALL_THRESHOLD_MM;
  WallDist d = read();
  return d.left_ok && d.left_mm < thresholdMm;
}

bool VL53Sensors::rightWall(int thresholdMm) {
  if (thresholdMm < 0) thresholdMm = WALL_THRESHOLD_MM;
  WallDist d = read();
  return d.right_ok && d.right_mm < thresholdMm;
}