#include "RobotNav.h"
#include "encoder.h"
#include "drv8833.h"
#include "VL53Sensors.h"
#include "MPU6050.h"
#include <Preferences.h>
#include <string.h>
#include <math.h>

RobotNav robotNav;
static Preferences prefs;
static const uint32_t MAP_VERSION = 0x20260306;

// -------- maze mặc định: 1=tường ngoài, 2/3=tường đã biết, 4/5=chưa biết(?/???), 0=trống --------
// (giữ nguyên cấu trúc bàn cờ 2 lớp ô/tường như bản gốc; sinh động bằng vòng lặp cho gọn)

// ====================== nội bộ: hàng đợi cho BFS flood fill ======================
namespace {
  struct Node { int x, y; };
  Node q[900];
  int qFront = 0, qBack = 0;
  void qReset() { qFront = qBack = 0; }
  bool qEmpty() { return qFront == qBack; }
  void qPush(int x, int y) { q[qBack] = {x, y}; qBack = (qBack + 1) % 900; }
  Node qPop() { Node n = q[qFront]; qFront = (qFront + 1) % 900; return n; }

  const int dxArr[4] = {-2, 0, 2, 0};
  const int dyArr[4] = {0, 2, 0, -2};
}

char RobotNav::headingChar() const {
  switch (_heading) {
    case NORTH: return 'N';
    case EAST:  return 'E';
    case SOUTH: return 'S';
    default:    return 'W';
  }
}

// ============================================================================
//  INIT
// ============================================================================
void RobotNav::init() {
  encoder.begin();
  motor.begin();

  if (!tof.begin())  Serial.println("[RobotNav] canh bao: VL53L0X loi");
  if (!gyro.begin()) Serial.println("[RobotNav] canh bao: MPU6050 loi");

  resetMaze();

  if (loadMazeFromFlash()) {
    finalizeUnknownWalls();
    if (buildPlannedPath(_goalX1, _goalY1, EAST, _goalX2, _goalY2)) {
      _fastRunReady = true;
      Serial.println("[RobotNav] Da nap map tu flash, fast-run san sang.");
    }
  }
}

void RobotNav::resetMaze() {
  for (int i = 0; i < SIZE; i++)
    for (int j = 0; j < SIZE; j++)
      _maze[i][j] = (i % 2 == 0) ? ((j % 2 == 0) ? 1 : ((i == 0 || i == SIZE - 1) ? 3 : 5))
                                   : ((j % 2 == 0) ? ((j == 0 || j == SIZE - 1) ? 2 : 4) : 0);

  memset(_dist, -1, sizeof(_dist));
  _cx = 1; _cy = 1;
  _heading = EAST;
  _plannedLen = 0;
  _plannedPath[0] = '\0';
  _fastRunReady = false;
  _state = NAV_IDLE;
  encoder.reset();
  _wallPid.reset();
  _syncPid.reset();
}

// ============================================================================
//  ĐIỀU KHIỂN TỪ COMMANDHANDLER (BLE)
// ============================================================================
void RobotNav::startExplore() {
  resetMaze();
  gyro.calibrate();
  _state = NAV_EXPLORING;
  Serial.println("[RobotNav] Bat dau EXPLORE");
}

void RobotNav::startFastRun() {
  if (!_fastRunReady) {
    Serial.println("[RobotNav] Chua co map/path, khong the fast-run.");
    return;
  }
  encoder.reset();
  gyro.calibrate();
  _state = NAV_FAST_RUN;
  Serial.println("[RobotNav] Bat dau FAST RUN");
}

void RobotNav::stop() {
  motor.stop();
  _state = NAV_IDLE;
}

// ============================================================================
//  UPDATE — 1 bước state machine mỗi lần loop() gọi
// ============================================================================
void RobotNav::update() {
  gyro.update();

  switch (_state) {
    case NAV_EXPLORING: {
      detectAndUpdateWalls(_cx, _cy, _heading);
      floodFill(_goalX2, _goalY2);

      Heading target = chooseBestDirection(_cx, _cy, _heading);
      rotateTo(target);

      bool moved = driveOneCell();
      if (moved) {
        if (_heading == NORTH) _cx--;
        else if (_heading == SOUTH) _cx++;
        else if (_heading == EAST)  _cy++;
        else if (_heading == WEST)  _cy--;
        _mapChanged = true;
      }

      if (_cx * 2 - 1 == _goalX2 && _cy * 2 - 1 == _goalY2) {
        motor.stop();
        finalizeUnknownWalls();
        Serial.println("[RobotNav] Toi dich! Da khoa ban do.");
        saveMazeToFlash();

        if (buildPlannedPath(_goalX1, _goalY1, EAST, _goalX2, _goalY2)) {
          _fastRunReady = true;
          Serial.println("[RobotNav] Da tao duong di nhanh.");
        }
        _state = NAV_GOAL_REACHED;
      }
      break;
    }

    case NAV_FAST_RUN:
      executePlannedPath();
      _state = NAV_DONE;
      break;

    default:
      break;  // NAV_IDLE, NAV_GOAL_REACHED, NAV_DONE: chờ lệnh mới từ BLE
  }
}

// ============================================================================
//  CẢM BIẾN TƯỜNG (dùng 3 VL53L0X: front/left/right thay cho 5 IR)
// ============================================================================
void RobotNav::detectAndUpdateWalls(int cx, int cy, Heading h) {
  WallDist d = tof.read();
  _lastDist = {d.front_mm, d.left_mm, d.right_mm, d.front_ok, d.left_ok, d.right_ok};

  bool front = d.front_ok && d.front_mm < WALL_THRESHOLD_MM;
  bool left  = d.left_ok  && d.left_mm  < WALL_THRESHOLD_MM;
  bool right = d.right_ok && d.right_mm < WALL_THRESHOLD_MM;

  if (front) updateFrontWall(cx, cy, h); else clearFrontWall(cx, cy, h);
  if (left)  updateLeftWall(cx, cy, h);  else clearLeftWall(cx, cy, h);
  if (right) updateRightWall(cx, cy, h); else clearRightWall(cx, cy, h);
  _mapChanged = true;
}

void RobotNav::updateFrontWall(int cx, int cy, Heading h) {
  int mx = cx * 2 - 1, my = cy * 2 - 1;
  if (h == NORTH) _maze[mx - 1][my] = 3;
  else if (h == SOUTH) _maze[mx + 1][my] = 3;
  else if (h == EAST)  _maze[mx][my + 1] = 2;
  else if (h == WEST)  _maze[mx][my - 1] = 2;
}
void RobotNav::updateLeftWall(int cx, int cy, Heading h) {
  Heading L = (Heading)((h + 3) % 4);
  int mx = cx * 2 - 1, my = cy * 2 - 1;
  if (L == NORTH) _maze[mx - 1][my] = 3;
  else if (L == SOUTH) _maze[mx + 1][my] = 3;
  else if (L == EAST)  _maze[mx][my + 1] = 2;
  else if (L == WEST)  _maze[mx][my - 1] = 2;
}
void RobotNav::updateRightWall(int cx, int cy, Heading h) {
  Heading R = (Heading)((h + 1) % 4);
  int mx = cx * 2 - 1, my = cy * 2 - 1;
  if (R == NORTH) _maze[mx - 1][my] = 3;
  else if (R == SOUTH) _maze[mx + 1][my] = 3;
  else if (R == EAST)  _maze[mx][my + 1] = 2;
  else if (R == WEST)  _maze[mx][my - 1] = 2;
}
void RobotNav::clearFrontWall(int cx, int cy, Heading h) {
  int mx = cx * 2 - 1, my = cy * 2 - 1;
  if (h == NORTH) { if (_maze[mx - 1][my] == 5) _maze[mx - 1][my] = 0; }
  else if (h == SOUTH) { if (_maze[mx + 1][my] == 5) _maze[mx + 1][my] = 0; }
  else if (h == EAST)  { if (_maze[mx][my + 1] == 4) _maze[mx][my + 1] = 0; }
  else if (h == WEST)  { if (_maze[mx][my - 1] == 4) _maze[mx][my - 1] = 0; }
}
void RobotNav::clearLeftWall(int cx, int cy, Heading h) {
  Heading L = (Heading)((h + 3) % 4);
  int mx = cx * 2 - 1, my = cy * 2 - 1;
  if (L == NORTH) { if (_maze[mx - 1][my] == 5) _maze[mx - 1][my] = 0; }
  else if (L == SOUTH) { if (_maze[mx + 1][my] == 5) _maze[mx + 1][my] = 0; }
  else if (L == EAST)  { if (_maze[mx][my + 1] == 4) _maze[mx][my + 1] = 0; }
  else if (L == WEST)  { if (_maze[mx][my - 1] == 4) _maze[mx][my - 1] = 0; }
}
void RobotNav::clearRightWall(int cx, int cy, Heading h) {
  Heading R = (Heading)((h + 1) % 4);
  int mx = cx * 2 - 1, my = cy * 2 - 1;
  if (R == NORTH) { if (_maze[mx - 1][my] == 5) _maze[mx - 1][my] = 0; }
  else if (R == SOUTH) { if (_maze[mx + 1][my] == 5) _maze[mx + 1][my] = 0; }
  else if (R == EAST)  { if (_maze[mx][my + 1] == 4) _maze[mx][my + 1] = 0; }
  else if (R == WEST)  { if (_maze[mx][my - 1] == 4) _maze[mx][my - 1] = 0; }
}
void RobotNav::finalizeUnknownWalls() {
  for (int i = 0; i < SIZE; i++)
    for (int j = 0; j < SIZE; j++) {
      if (_maze[i][j] == 4) _maze[i][j] = 2;
      else if (_maze[i][j] == 5) _maze[i][j] = 3;
    }
}

// ============================================================================
//  FLOOD FILL
// ============================================================================
bool RobotNav::canMove(int x, int y, int nx, int ny) {
  if (nx < 0 || ny < 0 || nx >= SIZE || ny >= SIZE) return false;
  int wx = (x + nx) / 2, wy = (y + ny) / 2;
  int w = _maze[wx][wy];
  if (w == 2 || w == 3) return false;
  if (_maze[nx][ny] != 0) return false;
  return true;
}

void RobotNav::floodFill(int goalMx, int goalMy) {
  memset(_dist, -1, sizeof(_dist));
  qReset();
  _dist[goalMx][goalMy] = 0;
  qPush(goalMx, goalMy);

  while (!qEmpty()) {
    Node cur = qPop();
    for (int d = 0; d < 4; d++) {
      int nx = cur.x + dxArr[d], ny = cur.y + dyArr[d];
      if (nx < 0 || nx >= SIZE || ny < 0 || ny >= SIZE) continue;
      if (!canMove(cur.x, cur.y, nx, ny)) continue;
      if (_dist[nx][ny] == -1) {
        _dist[nx][ny] = _dist[cur.x][cur.y] + 1;
        qPush(nx, ny);
      }
    }
  }
}

Heading RobotNav::chooseBestDirection(int cx, int cy, Heading h) {
  int mx = cx * 2 - 1, my = cy * 2 - 1;
  int best = 9999;
  Heading bestDir = h;

  for (int d = 0; d < 4; d++) {
    int nx = mx + dxArr[d], ny = my + dyArr[d];
    if (nx < 0 || ny < 0 || nx >= SIZE || ny >= SIZE) continue;
    if (!canMove(mx, my, nx, ny)) continue;
    if (_dist[nx][ny] < 0) continue;

    if (_dist[nx][ny] < best) { best = _dist[nx][ny]; bestDir = (Heading)d; }
    else if (_dist[nx][ny] == best && (Heading)d == h) bestDir = (Heading)d;
  }
  return bestDir;
}

Heading RobotNav::chooseBestDirectionSolved(int mx, int my, Heading h) {
  int best = 9999;
  Heading bestDir = h;
  for (int d = 0; d < 4; d++) {
    int nx = mx + dxArr[d], ny = my + dyArr[d];
    if (nx < 0 || ny < 0 || nx >= SIZE || ny >= SIZE) continue;
    if (!canMove(mx, my, nx, ny)) continue;
    if (_dist[nx][ny] < 0) continue;
    if (_dist[nx][ny] < best) { best = _dist[nx][ny]; bestDir = (Heading)d; }
    else if (_dist[nx][ny] == best && (Heading)d == h) bestDir = (Heading)d;
  }
  return bestDir;
}

// ============================================================================
//  QUAY (encoder + gyro để chốt góc chính xác hơn bản gốc chỉ dùng encoder)
// ============================================================================
void RobotNav::turn90(int dir) {
  encoder.reset();
  gyro.resetYaw();

  unsigned long lastUs = micros();
  float speed = 0;
  const float TARGET_DEG = 90.0f;

  while (true) {
    unsigned long now = micros();
    float dt = (now - lastUs) * 1e-6f;
    if (dt <= 0) dt = 0.001f;
    lastUs = now;

    gyro.update();
    float doneDeg = fabs(gyro.yawDeg());
    float remDeg = TARGET_DEG - doneDeg;
    if (remDeg <= 0.5f) break;

    // hồ sơ tốc độ đơn giản dựa trên góc còn lại (độ) thay vì xung encoder
    float vBrake = sqrtf(fmaxf(0, 2.0f * 300.0f * remDeg));
    float vCmd = fminf(220.0f, vBrake);
    speed = vCmd;

    int pwm = TURN_MIN_PWM + (int)((TURN_MAX_PWM - TURN_MIN_PWM) * (speed / 220.0f));
    pwm = constrain(pwm, TURN_MIN_PWM, TURN_MAX_PWM);

    int L = dir * pwm;
    int R = -dir * pwm;
    motor.run(L, R);
    delay(4);
  }

  motor.stop();
  delay(20);
  encoder.reset();
}

void RobotNav::rotateTo(Heading target) {
  int diff = (target - _heading + 4) % 4;
  if (diff == 1) turn90(+1);
  else if (diff == 3) turn90(-1);
  else if (diff == 2) { turn90(+1); turn90(+1); }
  _heading = target;
}

// ============================================================================
//  CHẠY 1 Ô — wall-follow PID dùng front/left/right ToF
// ============================================================================
bool RobotNav::driveOneCell() {
  driveDistanceCm(CELL_LEN_CM, MAX_SPEED_PWM);
  return true;
}

void RobotNav::driveDistanceCm(float targetCm, int pwmCmd) {
  encoder.reset();
  _wallPid.reset();
  _syncPid.reset();

  unsigned long startMs = millis();
  unsigned long lastStepMs = 0;

  while (true) {
    if (millis() - lastStepMs < 20) continue;   // ~50Hz
    lastStepMs = millis();

    float d = encoder.distanceAvgCm();
    if (d >= targetCm) break;
    if (millis() - startMs > 15000) { Serial.println("[RobotNav] timeout chay 1 o"); break; }

    WallDist w = tof.read();
    bool left  = w.left_ok  && w.left_mm  < 200;
    bool right = w.right_ok && w.right_mm < 200;

    float error;
    if (left && right) {
      error = (float)w.right_mm - (float)w.left_mm;   // lệch giữa 2 tường (mm)
    } else if (left) {
      error = -((float)w.left_mm - 40.0f);             // giữ cách tường trái ~40mm
    } else if (right) {
      error = ((float)w.right_mm - 40.0f);
    } else {
      long el = encoder.left(), er = encoder.right();
      error = _syncPid.compute((float)(el - er), 0.02f);
    }

    float correction = (left || right) ? _wallPid.compute(error, 0.02f) : error;

    // giảm tốc gần cuối ô
    float remain = targetCm - d;
    float speed = pwmCmd;
    if (remain < CRAWL_DIST_CM) {
      speed = CRAWL_PWM + (remain / CRAWL_DIST_CM) * (pwmCmd - CRAWL_PWM);
      if (speed < CRAWL_PWM) speed = CRAWL_PWM;
    }

    int L = constrain((int)(speed - correction), -255, 255);
    int R = constrain((int)(speed + correction), -255, 255);
    motor.run(L, R);
  }

  motor.stop();
  delay(30);
  encoder.reset();
}

// ============================================================================
//  LẬP KẾ HOẠCH ĐƯỜNG ĐI NHANH (từ map đã khám phá) + CHẠY
// ============================================================================
void RobotNav::appendPathCmd(char c) {
  if (_plannedLen < (int)sizeof(_plannedPath) - 1) {
    _plannedPath[_plannedLen++] = c;
    _plannedPath[_plannedLen] = '\0';
  }
}

bool RobotNav::buildPlannedPath(int startMx, int startMy, Heading startH, int goalMx, int goalMy) {
  _plannedLen = 0;
  _plannedPath[0] = '\0';
  floodFill(goalMx, goalMy);
  if (_dist[startMx][startMy] < 0) return false;

  int mx = startMx, my = startMy;
  Heading h = startH;
  int guard = 0;

  while (!(mx == goalMx && my == goalMy) && guard < 400) {
    Heading nextH = chooseBestDirectionSolved(mx, my, h);
    int nx = mx + dxArr[nextH], ny = my + dyArr[nextH];
    if (nx < 0 || ny < 0 || nx >= SIZE || ny >= SIZE) return false;
    if (!canMove(mx, my, nx, ny)) return false;
    if (_dist[nx][ny] < 0) return false;

    int diff = (nextH - h + 4) % 4;
    if (diff == 1) appendPathCmd('R');
    else if (diff == 3) appendPathCmd('L');
    else if (diff == 2) { appendPathCmd('R'); appendPathCmd('R'); }
    appendPathCmd('F');

    mx = nx; my = ny; h = nextH;
    guard++;
  }
  return (mx == goalMx && my == goalMy);
}

void RobotNav::executePlannedPath() {
  int i = 0;
  while (i < _plannedLen) {
    while (i < _plannedLen && (_plannedPath[i] == 'L' || _plannedPath[i] == 'R')) {
      if (_plannedPath[i] == 'L') turn90(-1); else turn90(+1);
      i++;
    }
    int fCount = 0;
    while (i < _plannedLen && _plannedPath[i] == 'F') { fCount++; i++; }
    if (fCount > 0) {
      driveDistanceCm(CELL_LEN_CM * fCount, MAX_SPEED_PWM);
    }
  }
  motor.stop();
  Serial.println("[RobotNav] FAST RUN DONE");
}

// ============================================================================
//  LƯU / NẠP MAP TỪ FLASH (Preferences — giữ nguyên ý tưởng bản gốc)
// ============================================================================
bool RobotNav::saveMazeToFlash() {
  prefs.begin("micromouse", false);
  prefs.putBool("valid", true);
  prefs.putULong("ver", MAP_VERSION);
  prefs.putInt("goalx", _goalX2);
  prefs.putInt("goaly", _goalY2);
  prefs.putInt("msize", SIZE);
  size_t written = prefs.putBytes("maze", _maze, sizeof(_maze));
  prefs.end();
  return written == sizeof(_maze);
}

bool RobotNav::loadMazeFromFlash() {
  prefs.begin("micromouse", true);
  bool valid = prefs.getBool("valid", false);
  uint32_t ver = prefs.getULong("ver", 0);
  int gx = prefs.getInt("goalx", -1);
  int gy = prefs.getInt("goaly", -1);
  int msize = prefs.getInt("msize", -1);

  if (!valid || ver != MAP_VERSION || gx != _goalX2 || gy != _goalY2 || msize != SIZE) {
    prefs.end();
    return false;
  }
  size_t readBytes = prefs.getBytes("maze", _maze, sizeof(_maze));
  prefs.end();
  return readBytes == sizeof(_maze);
}

void RobotNav::clearMazeFlash() {
  prefs.begin("micromouse", false);
  prefs.clear();
  prefs.end();
}

String RobotNav::getMazeString() const {
  String out = "";
  out.reserve(SIZE * SIZE + 1);
  for (int i = 0; i < SIZE; i++) {
    for (int j = 0; j < SIZE; j++) {
      out += String(_maze[i][j]);
    }
  }
  return out;
}