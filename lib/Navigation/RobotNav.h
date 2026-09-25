#pragma once
#include <Arduino.h>
#include "RobotConfig.h"
#include "PIDController.h"

enum Heading { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
enum NavState { NAV_IDLE, NAV_EXPLORING, NAV_GOAL_REACHED, NAV_FAST_RUN, NAV_DONE };

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
  int mazeCells() const { return _cells; }
  int goalRow() const   { return _goalRow; }
  int goalCol() const   { return _goalCol; }
  int goalSize() const  { return _goalSize; }

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
  // PID bám tường chỉnh live qua BLE (SET_KP / SET_KI / SET_KD) — giống project mm
  PIDController& wallPid() { return _wallPid; }
  PIDController& gyroPid() { return _gyroPid; }
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
  bool  spinUntil(int dir, float targetYaw, bool untilAbove, int pwm, unsigned long timeoutMs);
  float _lastTurnTarget = 0, _lastTurnDone = 0;
  bool  _turnResultReady = false;
  char  _lastTurnSrc = 'G';
  float _gyroSign = GYRO_Z_SIGN_DEFAULT;   // yaw gyro * _gyroSign => dương khi quay phải
  float encoderTurnDeg() const;
  void idle() { if (_idleHook) _idleHook(); }
  void advanceCell();                // tiến 1 ô theo _heading, báo map đổi để BLE gửi vị trí mới
  static const int MAX_SIZE = MAZE_MAX_SIZE;

  // cấu hình mê cung lúc chạy (chỉnh qua BLE, lưu flash)
  int _cells = MAZE_CELLS;
  int _size = 2 * MAZE_CELLS + 1;        // kích thước lưới _maze đang dùng
  int _goalRow = GOAL_ROW, _goalCol = GOAL_COL, _goalSize = GOAL_SIZE;
  void loadMazeConfig();
  void reloadMap();                      // reset map + nạp map đã lưu (nếu khớp cấu hình)

  uint8_t _maze[MAX_SIZE][MAX_SIZE];
  bool _updateMode = false;
  bool _wallSeen[MAX_SIZE][MAX_SIZE];    // tường đã được đo lại trong lần chạy cập nhật hiện tại
  bool forgetUnseenWalls();              // tường cũ chưa đo lại => về "chưa biết"; true nếu có thay đổi
  int _dist[MAX_SIZE][MAX_SIZE];

  int _cx = 1, _cy = 1;
  Heading _heading = EAST;
  NavState _state = NAV_IDLE;


  bool _fastRunReady = false;
  char _plannedPath[MAZE_MAX_CELLS * MAZE_MAX_CELLS * 3 + 1];   // mỗi ô tối đa 'R','R','F'
  int  _plannedLen = 0;

  struct WallDistLocal { int front_mm, left_mm, right_mm; bool front_ok, left_ok, right_ok; };
  WallDistLocal _lastDist{};

  // hệ số PID lấy từ project mm (đã chỉnh trên sa hình thật)
  PIDController _wallPid{0.25f, 0.0f, 0.8f, -40, 40};   // bám tường: error(mm) -> hiệu chỉnh PWM
  PIDController _gyroPid{0.6f, 0.0f, 0.0f, -60, 60};    // không có tường 2 bên: giữ hướng theo gyro
  PIDController _syncPid{0.08f, 0.0f, 0.0f, -60, 60};   // gyro lỗi: đồng bộ 2 encoder

  // ----- flood fill -----
  // unknownIsWall = true: coi tường chưa biết là tường thật (lập đường fast run chỉ qua đường đã xác nhận)
  void floodFill(bool unknownIsWall = false);   // từ vùng đích (_goalRow/_goalCol/_goalSize)
  bool isGoalCell(int cx, int cy) const;
  bool canMove(int x, int y, int nx, int ny, bool unknownIsWall = false);
  Heading chooseBestDirection(int cx, int cy, Heading h);

  // ----- wall sensing -----
  void detectAndUpdateWalls(int cx, int cy, Heading h);
  void wallIndex(int cx, int cy, Heading side, int& wx, int& wy) const;
  void setWall(int cx, int cy, Heading side, bool present);   // side = hướng tuyệt đối

  // ----- motion -----
  void rotateTo(Heading target);
  void turn90(int dir);              // dir: -1 trái, +1 phải
  float turnDeg(int dir, float deg); // trả về góc thực tế đã quay (độ)
  bool driveOneCell();               // chạy 1 ô (18cm), wall-follow PID, true nếu ok
  void driveCells(int cells, int pwm);   // đi thẳng cells ô, mỗi ô = _cellPulses xung encoder

  // ----- fast run path planning -----
  bool buildPlannedPath(int startCx, int startCy, Heading startH);
  Heading chooseBestDirectionSolved(int mx, int my, Heading h);
  void appendPathCmd(char c);
  void executePlannedPath();

  // ----- flash persistence -----
  bool saveMazeToFlash();
  bool loadMazeFromFlash();
  void clearMazeFlash();
};

extern RobotNav robotNav;