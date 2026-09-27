#pragma once
#include <Arduino.h>
#include "RobotConfig.h"
#include "PIDController.h"
#include "Map.h"
#include "Floodfill.h"

struct WallDist;   // định nghĩa đầy đủ trong VL53Sensors.h — chỉ cần khai báo trước cho tham số hàm

enum NavState { NAV_IDLE, NAV_EXPLORING, NAV_GOAL_REACHED, NAV_FAST_RUN, NAV_DONE, NAV_PID_TEST, NAV_PID_TEST_CELL };

class RobotNav {
public:
  void init();     // khởi động encoder, motor, cảm biến, gyro, nạp map từ flash nếu có
  void update();   // gọi liên tục trong loop() — chạy 1 bước của state machine hiện tại

  // điều khiển từ CommandHandler (BLE) thay cho nút bấm vật lý
  bool startExplore();   // false nếu không thể bắt đầu (vd. tốc độ = 0)
  // Chế độ CẬP NHẬT MÊ CUNG: chạy lại từ (1,1) nhưng GIỮ bản đồ đã có, đo lại & sửa tường ở các ô đi qua
  bool startUpdate();
  bool isUpdating() const { return _updateMode && _state == NAV_EXPLORING; }
  void startFastRun();
  // Chế độ 2 (căn chỉnh PID): chạy PID bám tường LIÊN TỤC, TỰ DO — KHÔNG đụng tới bản đồ/vị trí ô
  // (không advanceCell, không gửi "map" lên web) — chỉ để test Kp/Ki/Kd live. Dừng bằng STOP.
  bool startPidTest();
  // Chế độ 2 - biến thể "đi từng ô": giống startPidTest() nhưng đi từng đoạn dài _cellPulses rồi
  // dừng ngắn (CELL_PAUSE_MS) như driveOneCell() — VẪN KHÔNG đụng bản đồ/vị trí ô (advanceCell bị tắt).
  bool startPidTestCell();
  void stop();
  void resetMaze();
  void resetAll();   // xoá bản đồ đang khám phá + map đã lưu trong flash, về trạng thái ban đầu

  // telemetry cho CommandHandler đóng gói JSON gửi qua BLE
  NavState state() const { return _state; }
  int  cellX() const { return _cx; }
  int  cellY() const { return _cy; }
  char headingChar() const;
  bool fastRunReady() const { return _fastRunReady; }
  int  frontMm() const { return _lastDist.front_mm; }
  int  leftMm() const  { return _lastDist.left_mm; }
  int  rightMm() const { return _lastDist.right_mm; }

  bool hasMapChanged() const { return _mapChanged; }
  void clearMapChanged() { _mapChanged = false; }

  // thông báo 1 lần cho web (vd. robot bị kẹt); trả về false nếu không có
  bool takeEvent(String& out) {
    if (!_event[0]) return false;
    out = _event; _event[0] = 0;
    return true;
  }
  String getMazeString() const;

  // ----- cấu hình mê cung (tab Bản đồ trên web) -----
  // trả về "" nếu OK, ngược lại là lý do lỗi
  String setMazeConfig(int cells, int goalRow, int goalCol, int goalSize);
  int mazeCells() const { return _map.cells(); }
  int goalRow() const   { return _map.goalRow(); }
  int goalCol() const   { return _map.goalCol(); }
  int goalSize() const  { return _map.goalSize(); }

  // gọi định kỳ bên trong các vòng chạy/quay (blocking) để vẫn gửi được telemetry qua BLE
  void setIdleHook(void (*hook)()) { _idleHook = hook; }

  // ----- Chế độ 1: test quay góc & bù quán tính (chỉnh live qua BLE) -----
  bool  testTurn(int dir, float deg);   // chỉ chạy khi robot không explore/fast-run
  void  setTurnParams(float leftComp, float rightComp, int turnSpeed);
  // góc hướng hiển thị trên đồng hồ (-180..180, dương = quay PHẢI/theo chiều kim đồng hồ)
  float displayYawDeg() const;

  // ----- Chế độ 3: điều khiển tay từng ô -----
  // action: 'F' đi thẳng 1 ô, 'L' quay trái 90° + đi 1 ô, 'R' quay phải 90° + đi 1 ô, 'B' quay đầu 180°
  // trả về "" nếu OK, ngược lại là lý do không chạy / dừng giữa chừng
  String manualStep(char action);
  float leftComp() const  { return _leftComp; }
  float rightComp() const { return _rightComp; }
  int   turnSpeed() const { return _turnSpeed; }
  // tốc độ chạy thẳng khi explore / fast run (0..DRIVE_SPEED_MAX)
  void  setDriveSpeed(int pwm) { _driveSpeed = constrain(pwm, 0, DRIVE_SPEED_MAX); }
  int   driveSpeed() const { return _driveSpeed; }
  // độ dài 1 ô tính bằng số xung encoder — chỉnh live qua BLE (SET_CELL), lưu flash
  void  setCellPulses(int pulses);
  int   cellPulses() const { return _cellPulses; }
  long  lastDrivePulses() const { return _lastDrivePulses; }   // số xung encoder đếm được ở lần chạy gần nhất
  // PID bám tường chỉnh live qua BLE (SET_KP / SET_KI / SET_KD) — giống project mm. 1 PID DUY NHẤT
  // dùng cho mọi trường hợp (có tường / không tường) — sai số đưa vào đã được trộn sẵn, xem
  // wallFollowCorrection().
  PIDController& wallPid() { return _wallPid; }
  // src: 'G' = dừng theo gyro, 'E' = theo encoder (gyro lỗi), 'T' = hết giờ (cả 2 không đọc được)
  bool  takeTurnResult(float& target, float& turned, char& src) {
    if (!_turnResultReady) return false;
    _turnResultReady = false;
    target = _lastTurnTarget; turned = _lastTurnDone; src = _lastTurnSrc;
    return true;
  }

private:
  bool _mapChanged = false;
  int  _stuckCount = 0;
  char _event[128] = "";
  void notify(const char* msg) { strncpy(_event, msg, sizeof(_event) - 1); Serial.println(msg); }
  void (*_idleHook)() = nullptr;

  float _leftComp = TURN_LEFT_COMP_DEG;
  float _rightComp = TURN_RIGHT_COMP_DEG;
  int   _turnSpeed = TURN_DEFAULT_PWM;
  int   _driveSpeed = MAX_SPEED_PWM;
  int   _cellPulses = CELL_LEN_PULSES;
  long  _lastDrivePulses = 0;
  float _lastTurnTarget = 0, _lastTurnDone = 0;
  bool  _turnResultReady = false;
  char  _lastTurnSrc = 'G';
  float _gyroSign = GYRO_Z_SIGN_DEFAULT;   // yaw gyro * _gyroSign => dương khi quay phải
  float encoderTurnDeg() const;
  void idle() { if (_idleHook) _idleHook(); }
  void advanceCell();                // tiến 1 ô theo _heading, báo map đổi để BLE gửi vị trí mới

  // cấu hình mê cung lúc chạy (chỉnh qua BLE, lưu flash) + lưới tường đã dò được
  void loadMazeConfig();
  void reloadMap();                      // reset map + nạp map đã lưu (nếu khớp cấu hình)

  Map _map;
  Floodfill _floodfill;
  bool _updateMode = false;

  int _cx = 1, _cy = 1;
  Heading _heading = EAST;
  NavState _state = NAV_IDLE;


  bool _fastRunReady = false;
  char _plannedPath[MAZE_MAX_CELLS * MAZE_MAX_CELLS * 3 + 1];   // mỗi ô tối đa 'R','R','F'
  int  _plannedLen = 0;

  struct WallDistLocal { int front_mm, left_mm, right_mm; bool front_ok, left_ok, right_ok; };
  WallDistLocal _lastDist{};

  // Hệ số PID lấy đúng từ project mm: Kd ép về 0 để triệt tiêu sốc bẻ lái do nhiễu ToF (Kd nhân
  // với tốc độ BIẾN THIÊN của sai số — cảm biến càng nhiễu thì đạo hàm càng giật).
  PIDController _wallPid{0.18f, 0.0f, 0.0f, -WALL_ERR_MAX_MM, WALL_ERR_MAX_MM};

  // ----- wall sensing -----
  void detectAndUpdateWalls(int cx, int cy, Heading h);

  // ----- motion -----
  void rotateTo(Heading target);
  void turn90(int dir);              // dir: -1 trái, +1 phải
  float turnDeg(int dir, float deg); // trả về góc thực tế đã quay (độ)
  bool driveOneCell();               // chạy 1 ô, wall-follow PID, true nếu đi hết ô
  // đi thẳng cells ô, mỗi ô = _cellPulses xung encoder; false nếu phải dừng sớm (tường / encoder / timeout)
  // trackMap=false: bỏ advanceCell() (không cập nhật _cx/_cy, không báo map đổi) — dùng cho Chế độ 2.
  bool driveCells(int cells, int pwm, bool trackMap = true);
  bool frontWallKnown() const;       // bản đồ có tường ở phía trước ô hiện tại
  // 1 bước PID bám tường dùng trong driveCells() (Explore/Fast-run/Chế độ 2/3 đều gọi qua đây):
  // trả về correction (PWM, >0 = lái sang TRÁI). Sai số đưa vào PID là hỗn hợp (xem RobotConfig.h):
  // có tường thì W1*sai_số_tường + W2*sai_số_đồng_bộ_encoder; không tường thì trộn gyro+encoder.
  // smoothL/smoothR (EMA khoảng cách), pidMode (chế độ bám hiện tại), lastEncL/lastEncR (mốc xung
  // encoder lần gọi trước, để tính CHÊNH LỆCH GIỮA 2 LẦN GỌI chứ không phải cộng dồn từ đầu),
  // smoothErr (lọc thông thấp lần 2 trên sai số đã trộn) — tất cả là state riêng của người gọi.
  float wallFollowCorrection(const WallDist& w, float& smoothL, float& smoothR, int& pidMode,
                              long& lastEncL, long& lastEncR, float& smoothErr);

  // ----- Chế độ 2: test PID — port 1:1 startPID()/updatePIDLoop() của project mm (không qua
  // wallFollowCorrection()/driveCells()). Chạy mỗi lần update() gọi tới (không rate-limit như
  // project mm chạy tự do mỗi vòng loop()), KHÔNG theo ô/bản đồ. Gặp vật cản trước mặt thì DỪNG
  // HẲN (giống mm: pidRunActive=false) — không tự chạy lại như bản driveCells()-based cũ.
  void pidTestStep();
  float _ptSmoothL = WALL_TARGET_LEFT_MM, _ptSmoothR = WALL_TARGET_RIGHT_MM;
  long  _ptLastEncL = 0, _ptLastEncR = 0;
  float _ptSmoothErr = 0.0f;
  float _ptTargetYaw = 0.0f;      // mốc góc lúc bắt đầu — dùng khi không có tường (giống _targetYaw trong mm)
  unsigned long _ptLastStepUs = 0;

  // ----- Chế độ 2 (biến thể đi từng ô): dùng driveCells(1, speed, /*trackMap=*/false) — chặn như
  // driveOneCell() nên STOP trễ tối đa ~1 đoạn, KHÔNG đụng bản đồ/vị trí ô. Có tường/vật cản trước
  // mặt thì GIỮ NGUYÊN chờ hết vật cản rồi tự đi tiếp (giống Tự Do), không bỏ cuộc.
  void pidTestCellStep();
  // Sau khi dừng cuối đoạn: đo lại (readFresh, bỏ mẫu cũ lúc đang chạy) rồi tự xoay tại chỗ theo
  // sai số bám tường tới khi thẳng giữa ô (hoặc hết ALIGN_TIMEOUT_MS / không có tường để canh).
  void alignInCell();

  // ----- fast run path planning -----
  bool buildPlannedPath(int startCx, int startCy, Heading startH);
  void executePlannedPath();
};

extern RobotNav robotNav;