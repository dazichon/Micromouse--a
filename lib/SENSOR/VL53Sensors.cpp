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
  else _cf.inited = true;
  _front.setAddress(ADDR_FRONT);

  // bật LEFT
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  _left.setTimeout(500);
  if (!_left.init()) { Serial.println("VL53L0X LEFT init FAIL"); ok = false; }
  else _cl.inited = true;
  _left.setAddress(ADDR_LEFT);

  // bật RIGHT
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  _right.setTimeout(500);
  if (!_right.init()) { Serial.println("VL53L0X RIGHT init FAIL"); ok = false; }
  else _cr.inited = true;
  _right.setAddress(ADDR_RIGHT);

  Wire.setClock(400000);   // I2C nhanh hơn để đọc 3 cảm biến + MPU kịp trong vòng PID

  // 20ms/mẫu (mặc định ~33ms) => cảm biến cập nhật nhanh hơn khi đang chạy
  for (VL53L0X* s : {&_front, &_left, &_right}) {
    s->setTimeout(50);
    s->setMeasurementTimingBudget(20000);
    s->startContinuous();
  }

  _ok = ok;
  return ok;
}

static const uint32_t STALE_MS = 150;   // mẫu cũ hơn mức này coi như không tin cậy

bool VL53Sensors::poll(Chan& c) {
  if (!c.inited) return false;
  // chỉ đọc khi cảm biến báo có mẫu mới — không đứng chờ như readRangeContinuousMillimeters()
  if ((c.dev->readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return false;
  c.mm = c.dev->readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  c.dev->writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  c.stampMs = millis();
  return true;
}

bool VL53Sensors::fresh(const Chan& c) const {
  return c.inited && c.stampMs != 0 && (millis() - c.stampMs) < STALE_MS;
}

WallDist VL53Sensors::snapshot() const {
  WallDist d;
  d.front_mm = _cf.mm;
  d.left_mm  = _cl.mm;
  d.right_mm = _cr.mm;
  d.front_ok = fresh(_cf) && d.front_mm < 2000;
  d.left_ok  = fresh(_cl) && d.left_mm  < 2000;
  d.right_ok = fresh(_cr) && d.right_mm < 2000;
  return d;
}

WallDist VL53Sensors::read() {
  poll(_cf);
  poll(_cl);
  poll(_cr);
  return snapshot();
}

WallDist VL53Sensors::readFresh(uint32_t timeoutMs) {
  // Mẫu đang nằm trong thanh ghi có thể đã đo từ trước khi robot quay/dừng
  // => xóa cờ, rồi chờ mẫu đo mới hoàn toàn.
  bool gotF = !_cf.inited, gotL = !_cl.inited, gotR = !_cr.inited;
  if (_cf.inited) _front.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  if (_cl.inited) _left.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  if (_cr.inited) _right.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

  // mẫu đang đo dở lúc xóa cờ cũng có thể bắt đầu từ trước => bỏ thêm 1 mẫu
  int needF = 2, needL = 2, needR = 2;
  uint32_t start = millis();
  while (!(gotF && gotL && gotR) && millis() - start < timeoutMs) {
    if (!gotF && poll(_cf) && --needF == 0) gotF = true;
    if (!gotL && poll(_cl) && --needL == 0) gotL = true;
    if (!gotR && poll(_cr) && --needR == 0) gotR = true;
  }
  return snapshot();
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