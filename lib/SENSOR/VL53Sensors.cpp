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

  // VL53_TIMING_BUDGET_US/mẫu (thư viện mặc định ~33ms) => cảm biến cập nhật nhanh hơn khi đang chạy
  for (VL53L0X* s : {&_front, &_left, &_right}) {
    s->setTimeout(50);
    if (!s->setMeasurementTimingBudget(VL53_TIMING_BUDGET_US)) {
      Serial.printf("VL53L0X: budget %dus qua nho, dung 20000us%c", VL53_TIMING_BUDGET_US, 10);
      s->setMeasurementTimingBudget(20000);
    }
    s->startContinuous();
  }

  _ok = ok;
  return ok;
}

static const uint32_t STALE_MS = 150;   // mẫu cũ hơn mức này coi như không tin cậy

int VL53Sensors::median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

bool VL53Sensors::poll(Chan& c, bool smooth) {
  if (!c.inited) return false;
  // chỉ đọc khi cảm biến báo có mẫu mới — không đứng chờ như readRangeContinuousMillimeters()
  if ((c.dev->readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return false;

  // Bước 1: đọc giá trị raw
  int raw = c.dev->readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  c.dev->writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  c.stampMs = millis();

  if (!smooth) {
    // Cảm biến TRƯỚC: cần phản ứng NGAY để phanh kịp khi gặp vách — dùng thẳng raw, không lọc gì cả.
    c.mm = raw;
    return true;
  }

  // VL53L0X thỉnh thoảng chớp nháy giữa 1 mẫu đo đúng và 1 mẫu "ngoài tầm" (raw nhảy vọt lên
  // ~2000-8190) ngay cả khi vật đứng yên. KHÔNG được trộn số học 2 loại mẫu này qua median/EMA —
  // vd. xen kẽ đúng 100mm/8190mm sẽ cho ra một số "ổn định" giả ~3000-5000mm, tệ hơn cả raw vì
  // luôn bị hiểu nhầm là "không có tường". Nên tách trạng thái GẦN/XA có debounce riêng, EMA chỉ
  // chạy trên các mẫu GẦN với nhau.
  bool rawFar = raw >= TOF_VALID_RANGE_MAX_MM;

  if (rawFar) {
    c.nearStreak = 0;
    if (++c.farStreak >= TOF_FAR_CONFIRM_SAMPLES) {
      if (!c.isFar) {
        // vừa chuyển sang XA thật sự -> xoá bộ lọc GẦN, tránh mang giá trị cũ khi vật quay lại gần
        c.histCount = 0;
        c.emaInit = false;
      }
      c.isFar = true;
      c.mm = 8190;
    }
    // farStreak chưa đủ: mẫu "xa" đơn lẻ giữa các mẫu gần -> coi là nhiễu, GIỮ NGUYÊN c.mm hiện tại
    return true;
  }

  c.farStreak = 0;
  if (c.isFar && ++c.nearStreak < TOF_NEAR_CONFIRM_SAMPLES) {
    // mẫu "gần" đơn lẻ giữa các mẫu xa -> chưa chắc, vẫn đang coi là XA, chờ xác nhận thêm
    return true;
  }
  c.isFar = false;

  // Bước 2: median-3 trên 3 mẫu raw GẦN gần nhất — loại nhiễu xung (spike) 1 mẫu
  c.rawHist[0] = c.rawHist[1];
  c.rawHist[1] = c.rawHist[2];
  c.rawHist[2] = raw;
  if (c.histCount < 3) c.histCount++;
  if (c.histCount < 3) {
    // chưa đủ 3 mẫu (vừa khởi động / vừa hết XA) -> lấp đầy lịch sử bằng raw để median ổn định ngay
    c.rawHist[0] = raw;
    c.rawHist[1] = raw;
  }
  int med = median3(c.rawHist[0], c.rawHist[1], c.rawHist[2]);

  // Bước 3: EMA làm mượt trên giá trị đã qua median
  if (!c.emaInit) { c.ema = (float)med; c.emaInit = true; }
  else c.ema = TOF_EMA_ALPHA * (float)med + (1.0f - TOF_EMA_ALPHA) * c.ema;

  c.mm = (int)(c.ema + 0.5f);
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
  // ok = cảm biến vừa đo được mẫu mới. Ngoài tầm (8190 / >2000) vẫn là mẫu hợp lệ = KHÔNG có tường;
  // chỉ khi cảm biến không trả mẫu (lỗi / treo) thì ok = false => không được dùng để kết luận tường.
  d.front_ok = fresh(_cf);
  d.left_ok  = fresh(_cl);
  d.right_ok = fresh(_cr);
  return d;
}

WallDist VL53Sensors::read() {
  poll(_cf, false);   // trước: raw, phản ứng ngay
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
    if (!gotF && poll(_cf, false) && --needF == 0) gotF = true;
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