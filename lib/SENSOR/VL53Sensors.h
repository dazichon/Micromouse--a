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
    int mm = 8190;              // giá trị CUỐI (đã lọc) — dùng cho toàn bộ code
    uint32_t stampMs = 0;      // thời điểm nhận mẫu gần nhất
    int rawHist[3] = {8190, 8190, 8190}; // 3 mẫu raw GẦN (trong tầm) gần nhất, dùng để lọc median
    int histCount = 0;          // số mẫu raw đã có (để median ổn định ngay từ mẫu đầu)
    float ema = 8190.0f;        // trạng thái bộ lọc EMA (chỉ chạy trên mẫu GẦN, xem poll())
    bool emaInit = false;
    // Trạng thái GẦN/XA có debounce riêng — KHÔNG được trộn số học 1 mẫu "ngoài tầm" (vd. 8190)
    // với các mẫu khoảng cách thật (vd. 100mm) qua median/EMA, vì kết quả sẽ là 1 số vô nghĩa ở
    // giữa (vd. ~3000-5000mm) trong khi thực tế là "có tường sát" xen kẽ "không thấy gì".
    bool isFar = true;
    int  farStreak = 0, nearStreak = 0;
  };
  // true nếu vừa lấy được mẫu mới. smooth=false: dùng thẳng raw, không qua median/EMA/debounce
  // (cảm biến TRƯỚC cần phản ứng ngay để phanh kịp — không nên lọc, giống project mm).
  bool poll(Chan& c, bool smooth = true);
  static int median3(int a, int b, int c);
  bool fresh(const Chan& c) const;
  WallDist snapshot() const;

  VL53L0X _front, _left, _right;
  Chan _cf{&_front}, _cl{&_left}, _cr{&_right};
  bool _ok = false;
};

extern VL53Sensors tof;