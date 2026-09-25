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
static const uint32_t MAP_VERSION = 0x20260924;   // đổi khi đổi định dạng map lưu flash

// -------- maze mặc định: 1=tường ngoài, 2/3=tường đã biết, 4/5=chưa biết(?/???), 0=trống --------
// (giữ nguyên cấu trúc bàn cờ 2 lớp ô/tường như bản gốc; sinh động bằng vòng lặp cho gọn)

// ====================== nội bộ: hàng đợi cho BFS flood fill ======================
namespace {
  struct Node { int x, y; };
  const int Q_CAP = MAZE_MAX_SIZE * MAZE_MAX_SIZE;
  Node q[Q_CAP];
  int qFront = 0, qBack = 0;
  void qReset() { qFront = qBack = 0; }
  bool qEmpty() { return qFront == qBack; }
  void qPush(int x, int y) { q[qBack] = {x, y}; qBack = (qBack + 1) % Q_CAP; }
  Node qPop() { Node n = q[qFront]; qFront = (qFront + 1) % Q_CAP; return n; }

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
  else gyro.calibrate();   // driver mới không tự hiệu chuẩn trong begin()

  loadMazeConfig();
  reloadMap();
}

void RobotNav::reloadMap() {
  resetMaze();
  if (loadMazeFromFlash()) {
    if (buildPlannedPath(1, 1, EAST)) {
      _fastRunReady = true;
      Serial.println("[RobotNav] Da nap map tu flash, fast-run san sang.");
    }
  }
  _mapChanged = true;
}

// ============================================================================
//  CẤU HÌNH MÊ CUNG (kích thước + vùng đích) — chỉnh qua BLE, lưu flash
// ============================================================================
void RobotNav::loadMazeConfig() {
  prefs.begin("mmcfg", true);
  int n  = prefs.getInt("cells", MAZE_CELLS);
  int gr = prefs.getInt("grow", GOAL_ROW);
  int gc = prefs.getInt("gcol", GOAL_COL);
  int gs = prefs.getInt("gsize", GOAL_SIZE);
  int cp = prefs.getInt("cellp", CELL_LEN_PULSES);
  prefs.end();

  _cellPulses = (cp >= CELL_LEN_MIN_PULSES && cp <= CELL_LEN_MAX_PULSES) ? cp : CELL_LEN_PULSES;

  // giá trị trong flash hỏng => dùng mặc định trong RobotConfig.h
  if (n < 2 || n > MAZE_MAX_CELLS || gs < 1 || gs > 2 || gr < 1 || gc < 1 ||
      gr + gs - 1 > n || gc + gs - 1 > n) {
    n = MAZE_CELLS; gr = GOAL_ROW; gc = GOAL_COL; gs = GOAL_SIZE;
  }
  _cells = n; _size = 2 * n + 1;
  _goalRow = gr; _goalCol = gc; _goalSize = gs;
  Serial.printf("[RobotNav] Me cung %dx%d, dich (%d,%d) %dx%d%c", n, n, gr, gc, gs, gs, 10);
}

String RobotNav::setMazeConfig(int cells, int goalRow, int goalCol, int goalSize) {
  if (_state == NAV_EXPLORING || _state == NAV_FAST_RUN) return "Robot dang chay, gui STOP truoc";
  if (cells < 2 || cells > MAZE_MAX_CELLS) return "Kich thuoc phai tu 2 den " + String(MAZE_MAX_CELLS);
  if (goalSize < 1 || goalSize > 2) return "Vung dich chi 1x1 hoac 2x2";
  if (goalRow < 1 || goalCol < 1 || goalRow + goalSize - 1 > cells || goalCol + goalSize - 1 > cells)
    return "Dich nam ngoai me cung";
  if (goalRow == 1 && goalCol == 1) return "Dich khong duoc trung o xuat phat (1,1)";

  _cells = cells; _size = 2 * cells + 1;
  _goalRow = goalRow; _goalCol = goalCol; _goalSize = goalSize;

  prefs.begin("mmcfg", false);
  prefs.putInt("cells", _cells);
  prefs.putInt("grow", _goalRow);
  prefs.putInt("gcol", _goalCol);
  prefs.putInt("gsize", _goalSize);
  prefs.end();

  // map cũ trong flash tự bị bỏ qua nếu khác kích thước / đích (xem loadMazeFromFlash)
  reloadMap();
  Serial.printf("[RobotNav] Cau hinh moi: %dx%d, dich (%d,%d) %dx%d%c", cells, cells, goalRow, goalCol, goalSize, goalSize, 10);
  return "";
}

void RobotNav::setCellPulses(int pulses) {
  _cellPulses = constrain(pulses, CELL_LEN_MIN_PULSES, CELL_LEN_MAX_PULSES);
  prefs.begin("mmcfg", false);
  prefs.putInt("cellp", _cellPulses);
  prefs.end();
  Serial.printf("[RobotNav] Do dai 1 o = %d xung%c", _cellPulses, 10);
}

void RobotNav::resetMaze() {
  for (int i = 0; i < _size; i++)
    for (int j = 0; j < _size; j++)
      _maze[i][j] = (i % 2 == 0) ? ((j % 2 == 0) ? 1 : ((i == 0 || i == _size - 1) ? 3 : 5))
                                   : ((j % 2 == 0) ? ((j == 0 || j == _size - 1) ? 2 : 4) : 0);

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
bool RobotNav::startExplore() {
  if (_driveSpeed <= 0) {
    Serial.println("[RobotNav] Toc do = 0, khong the explore.");
    return false;
  }
  resetMaze();
  gyro.calibrate();
  _stuckCount = 0;
  _updateMode = false;
  _state = NAV_EXPLORING;
  Serial.println("[RobotNav] Bat dau EXPLORE");
  return true;
}

// Chạy lại mê cung mà KHÔNG xoá bản đồ: floodfill dùng các tường đã biết nên robot đi thẳng
// theo đường cũ, còn tường ở mỗi ô đi qua vẫn được đo lại và ghi đè (thêm / xoá) như khi explore.
// Dùng khi mê cung bị sửa một phần, hoặc để xác nhận thêm tường cho fast run.
bool RobotNav::startUpdate() {
  if (_driveSpeed <= 0) {
    Serial.println("[RobotNav] Toc do = 0, khong the cap nhat me cung.");
    return false;
  }
  memset(_wallSeen, 0, sizeof(_wallSeen));
  _cx = 1; _cy = 1;
  _heading = EAST;
  encoder.reset();
  _wallPid.reset();
  _syncPid.reset();
  gyro.calibrate();
  _stuckCount = 0;
  _updateMode = true;
  _mapChanged = true;
  _state = NAV_EXPLORING;
  Serial.println("[RobotNav] Bat dau CAP NHAT ME CUNG (giu ban do cu)");
  return true;
}

void RobotNav::startFastRun() {
  if (!_fastRunReady) {
    Serial.println("[RobotNav] Chua co map/path, khong the fast-run.");
    return;
  }
  encoder.reset();
  gyro.calibrate();
  // đường đi nhanh luôn được lập từ ô xuất phát, hướng EAST
  _cx = 1; _cy = 1;
  _heading = EAST;
  _mapChanged = true;
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
      floodFill();

      // Cập nhật: mê cung có thể đã bị sửa ở chỗ robot chưa đi lại => tường cũ chặn hết đường.
      // Bỏ các tường chưa đo lại trong lần chạy này (về "chưa biết") rồi tính lại.
      if (_dist[_cx * 2 - 1][_cy * 2 - 1] < 0 && _updateMode && forgetUnseenWalls()) {
        Serial.println("[RobotNav] Ban do cu khong con duong, bo cac tuong chua xac nhan lai");
        floodFill();
      }

      // Không còn đường tới đích theo bản đồ hiện tại => KHÔNG chạy (trước đây robot lao thẳng vào tường).
      // Đo lại vài lần — tường ghi nhầm do nhiễu sẽ được xoá ở lần đo sau.
      if (_dist[_cx * 2 - 1][_cy * 2 - 1] < 0) {
        motor.stop();
        if (++_stuckCount < 3) {
          Serial.println("[RobotNav] Khong co duong toi dich, do lai tuong...");
          delay(200);
          break;
        }
        notify("Bi ket: khong con duong toi dich. Kiem tra tuong / bam RESET MAP roi thu lai.");
        _state = NAV_IDLE;
        break;
      }
      _stuckCount = 0;

      Heading target = chooseBestDirection(_cx, _cy, _heading);
      rotateTo(target);

      driveOneCell();   // _cx/_cy được cập nhật bên trong driveCells

      if (isGoalCell(_cx, _cy)) {
        motor.stop();
        Serial.println(_updateMode ? "[RobotNav] Toi dich! Da cap nhat ban do." : "[RobotNav] Toi dich! Da khoa ban do.");
        saveMazeToFlash();
        if (_updateMode) notify("Cap nhat me cung xong, da luu ban do moi.");

        if (buildPlannedPath(1, 1, EAST)) {
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
// Đo tường bằng nhiều mẫu rồi lấy trung vị: 1 mẫu nhiễu (cảm biến bên cạnh bắt nhầm tay/vật,
// VL53 nhiễu chéo nhau) không đủ để ghi 1 bức tường.
static int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

void RobotNav::detectAndUpdateWalls(int cx, int cy, Heading h) {
  const int N = 3;
  int f[N], l[N], r[N];
  int fOk = 0, lOk = 0, rOk = 0;
  for (int k = 0; k < N; k++) {
    WallDist d = tof.readFresh();   // robot vừa dừng/quay: không dùng mẫu cũ trong cảm biến
    // mẫu lỗi coi như "rất xa" để không tạo tường giả
    f[k] = d.front_ok ? d.front_mm : 8190; fOk += d.front_ok;
    l[k] = d.left_ok  ? d.left_mm  : 8190; lOk += d.left_ok;
    r[k] = d.right_ok ? d.right_mm : 8190; rOk += d.right_ok;
  }
  int fm = median3(f[0], f[1], f[2]);
  int lm = median3(l[0], l[1], l[2]);
  int rm = median3(r[0], r[1], r[2]);
  _lastDist = {fm, lm, rm, fOk >= 2, lOk >= 2, rOk >= 2};

  bool front = fm < WALL_THRESHOLD_MM;
  bool left  = lm < WALL_THRESHOLD_MM;
  bool right = rm < WALL_THRESHOLD_MM;

  // ghi ĐÈ trạng thái theo lần đo mới nhất (cả thêm lẫn xoá) — tường ghi nhầm sẽ tự sửa khi đo lại
  setWall(cx, cy, h, front);
  setWall(cx, cy, (Heading)((h + 3) % 4), left);
  setWall(cx, cy, (Heading)((h + 1) % 4), right);
  _mapChanged = true;

  Serial.printf("[RobotNav] o(%d,%d) %c  F=%d%s L=%d%s R=%d%s\n", cx, cy, headingChar(),
                fm, front ? "#" : "", lm, left ? "#" : "", rm, right ? "#" : "");
}

// Ô tường nằm giữa ô (cx,cy) và ô kề theo hướng tuyệt đối `side`
void RobotNav::wallIndex(int cx, int cy, Heading side, int& wx, int& wy) const {
  int mx = cx * 2 - 1, my = cy * 2 - 1;
  wx = mx; wy = my;
  if (side == NORTH) wx = mx - 1;
  else if (side == SOUTH) wx = mx + 1;
  else if (side == EAST)  wy = my + 1;
  else                    wy = my - 1;
}

void RobotNav::setWall(int cx, int cy, Heading side, bool present) {
  int wx, wy;
  wallIndex(cx, cy, side, wx, wy);
  if (wx <= 0 || wy <= 0 || wx >= _size - 1 || wy >= _size - 1) return;   // tường bao ngoài: luôn giữ
  _wallSeen[wx][wy] = true;
  bool horizontal = (wx % 2 == 0);   // hàng chẵn = tường ngang (3), cột chẵn = tường dọc (2)
  _maze[wx][wy] = present ? (horizontal ? 3 : 2) : 0;
}

bool RobotNav::forgetUnseenWalls() {
  bool changed = false;
  for (int i = 1; i < _size - 1; i++)
    for (int j = 1; j < _size - 1; j++) {
      if (_wallSeen[i][j]) continue;
      if (_maze[i][j] == 2) { _maze[i][j] = 4; changed = true; }
      else if (_maze[i][j] == 3) { _maze[i][j] = 5; changed = true; }
    }
  if (changed) _mapChanged = true;
  return changed;
}

// ============================================================================
//  FLOOD FILL
// ============================================================================
bool RobotNav::canMove(int x, int y, int nx, int ny, bool unknownIsWall) {
  if (nx < 0 || ny < 0 || nx >= _size || ny >= _size) return false;
  int wx = (x + nx) / 2, wy = (y + ny) / 2;
  int w = _maze[wx][wy];
  if (w == 2 || w == 3) return false;
  if (unknownIsWall && (w == 4 || w == 5)) return false;
  if (_maze[nx][ny] != 0) return false;
  return true;
}

bool RobotNav::isGoalCell(int cx, int cy) const {
  return cx >= _goalRow && cx < _goalRow + _goalSize &&
         cy >= _goalCol && cy < _goalCol + _goalSize;
}

// flood fill từ TẤT CẢ các ô đích (vùng _goalSize x _goalSize ô) => _dist = số ô tới đích gần nhất
void RobotNav::floodFill(bool unknownIsWall) {
  memset(_dist, -1, sizeof(_dist));
  qReset();
  for (int r = _goalRow; r < _goalRow + _goalSize; r++)
    for (int c = _goalCol; c < _goalCol + _goalSize; c++) {
      int mx = r * 2 - 1, my = c * 2 - 1;
      _dist[mx][my] = 0;
      qPush(mx, my);
    }

  while (!qEmpty()) {
    Node cur = qPop();
    for (int d = 0; d < 4; d++) {
      int nx = cur.x + dxArr[d], ny = cur.y + dyArr[d];
      if (nx < 0 || nx >= _size || ny < 0 || ny >= _size) continue;
      if (!canMove(cur.x, cur.y, nx, ny, unknownIsWall)) continue;
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
    if (nx < 0 || ny < 0 || nx >= _size || ny >= _size) continue;
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
    if (nx < 0 || ny < 0 || nx >= _size || ny >= _size) continue;
    if (!canMove(mx, my, nx, ny, true)) continue;
    if (_dist[nx][ny] < 0) continue;
    if (_dist[nx][ny] < best) { best = _dist[nx][ny]; bestDir = (Heading)d; }
    else if (_dist[nx][ny] == best && (Heading)d == h) bestDir = (Heading)d;
  }
  return bestDir;
}

// ============================================================================
//  QUAY (encoder + gyro để chốt góc chính xác hơn bản gốc chỉ dùng encoder)
// ============================================================================
void RobotNav::turn90(int dir) { turnDeg(dir, 90.0f); }

// Quay tại chỗ theo gyro. dir: -1 trái, +1 phải.
// Robot có quán tính nên sẽ quay lố sau khi cắt động cơ => ngắt sớm hơn một góc bù
// (_leftComp / _rightComp, chỉnh live ở Chế độ 1 trên dashboard).
float RobotNav::turnDeg(int dir, float targetDeg) {
  gyro.setMoving(true);
  encoder.reset();
  gyro.update();      // cập nhật mốc thời gian trước khi reset để không cộng dồn dt cũ
  gyro.resetYaw();

  const float comp = (dir < 0) ? _leftComp : _rightComp;
  const float stopDeg = fmaxf(targetDeg - comp, 1.0f);
  const int maxPwm = max(_turnSpeed, TURN_MIN_PWM);
  // giới hạn thời gian theo góc: 90° tối đa ~2s — không bao giờ quay mãi
  const unsigned long timeoutMs = 800 + (unsigned long)(1200.0f * targetDeg / 90.0f);
  unsigned long startMs = millis();

  // Nguồn đo góc: ưu tiên gyro; nếu gyro lỗi / không đổi trong khi bánh đã quay => dùng encoder
  bool useGyro = gyro.ok();
  if (!useGyro) Serial.println("[RobotNav] MPU6050 loi -> quay theo encoder");
  _lastTurnSrc = useGyro ? 'G' : 'E';

  // KHÔNG gọi idle() (gửi BLE) trong vòng này: gửi BLE có thể chặn vài chục ms
  // => cắt động cơ muộn => quay lố.
  while (true) {
    gyro.update();
    if (useGyro && millis() - startMs > 400 && fabs(gyro.getYaw()) < 3.0f && encoderTurnDeg() > 20.0f) {
      useGyro = false;
      _lastTurnSrc = 'E';
      Serial.println("[RobotNav] gyro khong doi goc -> chuyen sang encoder");
    }
    float doneDeg = useGyro ? fabs(gyro.getYaw()) : encoderTurnDeg();
    float remDeg = stopDeg - doneDeg;
    if (remDeg <= 0) break;
    if (millis() - startMs > timeoutMs) {
      _lastTurnSrc = 'T';
      Serial.println("[RobotNav] timeout quay: gyro va encoder deu khong bao du goc");
      break;
    }

    // giảm tốc tuyến tính trong TURN_SLOW_DEG độ cuối: maxPwm -> TURN_MIN_PWM
    float k = fminf(1.0f, remDeg / TURN_SLOW_DEG);
    int pwm = TURN_MIN_PWM + (int)((maxPwm - TURN_MIN_PWM) * k);

    motor.run(dir * pwm, -dir * pwm);
    delay(2);
  }

  motor.brake();
  delay(80);
  motor.stop();

  // chờ robot đứng yên hẳn rồi mới chốt góc thực tế (tính cả phần quay lố do quán tính)
  unsigned long t0 = millis();
  while (millis() - t0 < 150) { gyro.update(); delay(2); }

  // quay bù chậm nếu còn lố/thiếu (chỉ khi đo bằng gyro — encoder không biết chiều)
  if (_lastTurnSrc == 'G') {
    float sgn = (gyro.getYaw() >= 0) ? 1.0f : -1.0f;   // chiều dương của gyro theo hướng vừa quay
    // học dấu gyro theo chiều quay thật => kim đồng hồ luôn quay đúng chiều robot
    if (fabs(gyro.getYaw()) > 20.0f) _gyroSign = sgn * dir;
    for (int attempt = 0; attempt < 3; attempt++) {
      float done = gyro.getYaw() * sgn;
      float err = done - targetDeg;
      if (fabs(err) <= TURN_TOL_DEG) break;
      if (err > 0) spinUntil(-dir, (targetDeg + 0.5f) * sgn, false, TURN_MIN_PWM, 800);   // lố => quay ngược lại
      else         spinUntil( dir, (targetDeg - 0.5f) * sgn, true,  TURN_MIN_PWM, 800);   // thiếu => quay thêm
    }
  }

  float turned = (_lastTurnSrc == 'E') ? encoderTurnDeg() : fabs(gyro.getYaw());
  idle();   // giờ mới gửi BLE

  _lastTurnTarget = targetDeg;
  _lastTurnDone = turned;
  _turnResultReady = true;
  Serial.printf("[RobotNav] quay %s %.0f -> thuc te %.1f (lech %+.1f)%c", dir < 0 ? "trai" : "phai",
                targetDeg, turned, turned - targetDeg, 10);

  encoder.reset();
  gyro.setMoving(false);

  int steps = ((int)lroundf(targetDeg / 90.0f)) % 4;
  _heading = (Heading)((_heading + (dir > 0 ? steps : 4 - steps)) % 4);
  _mapChanged = true;
  return turned;
}

// Quay chậm theo chiều dir tới khi yaw (có dấu) vượt qua targetAbsDeg:
// untilAbove = true => chờ yaw*sgn tăng lên; false => chờ giảm xuống. Sau đó phanh + chờ đứng yên.
bool RobotNav::spinUntil(int dir, float targetYaw, bool untilAbove, int pwm, unsigned long timeoutMs) {
  const float sgn = (targetYaw >= 0) ? 1.0f : -1.0f;
  const float target = fabs(targetYaw);
  unsigned long start = millis();
  bool reached = false;
  while (millis() - start < timeoutMs) {
    gyro.update();
    float done = gyro.getYaw() * sgn;
    if (untilAbove ? (done >= target) : (done <= target)) { reached = true; break; }
    motor.run(dir * pwm, -dir * pwm);
    delay(2);
  }
  motor.brake();
  delay(60);
  motor.stop();
  unsigned long t0 = millis();
  while (millis() - t0 < 120) { gyro.update(); delay(2); }
  return reached;
}

// góc đã quay tính từ encoder: quay tại chỗ mỗi bánh đi cung = θ·(B/2)
float RobotNav::encoderTurnDeg() const {
  float arc = (fabs(encoder.distanceLeftCm()) + fabs(encoder.distanceRightCm())) / 2.0f;
  return arc * 360.0f / (PI * WHEEL_BASE_CM);
}

float RobotNav::displayYawDeg() const {
  float a = fmodf(gyro.headingDeg() * _gyroSign, 360.0f);
  if (a > 180.0f) a -= 360.0f;
  else if (a <= -180.0f) a += 360.0f;
  return a;
}

bool RobotNav::testTurn(int dir, float deg) {
  if (_state == NAV_EXPLORING || _state == NAV_FAST_RUN) return false;
  turnDeg(dir, deg);
  return true;
}

// Điều khiển tay từng ô (Chế độ 3). Sau mỗi lần quay / đi đều đo lại tường ở ô hiện tại
// để bản đồ trên web cập nhật như khi explore.
String RobotNav::manualStep(char action) {
  if (_state == NAV_EXPLORING || _state == NAV_FAST_RUN) return "Robot dang chay, gui STOP truoc";
  if (action != 'F' && action != 'L' && action != 'R' && action != 'B') return "Lenh khong hop le";
  if (action != 'B' && _driveSpeed <= 0) return "Toc do = 0, tang toc do truoc";
  _lastDrivePulses = 0;

  if (action == 'L') turn90(-1);
  else if (action == 'R') turn90(+1);
  else if (action == 'B') turnDeg(+1, 180.0f);

  detectAndUpdateWalls(_cx, _cy, _heading);
  if (action == 'B') return "";

  // có tường ngay trước mặt => không chạy (tránh đâm tường)
  if (_lastDist.front_mm < WALL_THRESHOLD_MM) return "Co tuong phia truoc (" + String(_lastDist.front_mm) + "mm), khong di";

  int cx0 = _cx, cy0 = _cy;
  driveOneCell();
  detectAndUpdateWalls(_cx, _cy, _heading);
  if (_cx == cx0 && _cy == cy0)
    return "Chua sang o moi: encoder moi dem " + String(_lastDrivePulses) + "/" + String(_cellPulses) +
           " xung (can " + String(_cellPulses / 2) + " de sang o)";
  return "";
}

void RobotNav::setTurnParams(float leftComp, float rightComp, int turnSpeed) {
  _leftComp = constrain(leftComp, 0.0f, 45.0f);
  _rightComp = constrain(rightComp, 0.0f, 45.0f);
  _turnSpeed = constrain(turnSpeed, TURN_MIN_PWM, 255);
}

void RobotNav::advanceCell() {
  if (_heading == NORTH) _cx--;
  else if (_heading == SOUTH) _cx++;
  else if (_heading == EAST)  _cy++;
  else if (_heading == WEST)  _cy--;
  // an toàn: không bao giờ để vị trí báo cáo vượt ra ngoài mê cung (encoder lỗi, timeout, v.v.)
  _cx = constrain(_cx, 1, _cells);
  _cy = constrain(_cy, 1, _cells);
  _mapChanged = true;
}

void RobotNav::rotateTo(Heading target) {
  int diff = (target - _heading + 4) % 4;
  if (diff == 1) turn90(+1);
  else if (diff == 3) turn90(-1);
  else if (diff == 2) turnDeg(+1, 180.0f);
  // turnDeg() đã tự cập nhật _heading
}

// ============================================================================
//  CHẠY 1 Ô — wall-follow PID dùng front/left/right ToF
// ============================================================================
bool RobotNav::driveOneCell() {
  driveCells(1, _driveSpeed);
  return true;
}

void RobotNav::driveCells(int cells, int pwmCmd) {
  gyro.setMoving(true);
  encoder.reset();
  _wallPid.reset();
  _gyroPid.reset();
  _syncPid.reset();
  gyro.update();
  gyro.resetYaw();   // giữ hướng lúc bắt đầu chạy khi không có tường để bám

  // lọc EMA khoảng cách 2 bên (như project mm) — giảm giật do nhiễu VL53
  float smoothL = WALL_TARGET_LEFT_MM, smoothR = WALL_TARGET_RIGHT_MM;

  unsigned long startMs = millis();
  unsigned long lastStepMs = 0;

  // cập nhật vị trí ngay khi đang chạy: qua nửa ô thì coi như đã sang ô mới
  const int totalCells = cells;
  const long target = (long)cells * _cellPulses;   // tổng số xung phải đi
  int cellsDone = 0;
  bool reachedTarget = true;   // false nếu dừng sớm (gặp tường / hết giờ) — không được báo đã đi hết quãng

  // phát hiện encoder không đếm: không có nó robot không biết đã đi bao xa => chạy mãi tới khi gặp tường
  long lastProgress = 0;
  unsigned long lastProgressMs = startMs;
  bool warnedOneSide = false;

  while (true) {
    if (millis() - lastStepMs < 20) { idle(); continue; }   // ~50Hz
    lastStepMs = millis();

    long pL = labs(encoder.left()), pR = labs(encoder.right());
    long d = (pL + pR) / 2;   // số xung đã đi (trung bình 2 bánh)
    // 1 bánh gần như không đếm => trung bình chỉ còn một nửa, robot đi gấp đôi. Dùng bánh còn đếm.
    if (max(pL, pR) > 10 && min(pL, pR) * 4 < max(pL, pR)) {
      d = max(pL, pR);
      if (!warnedOneSide) {
        warnedOneSide = true;
        notify(pL < pR ? "Encoder TRAI gan nhu khong dem, tam dung encoder PHAI" : "Encoder PHAI gan nhu khong dem, tam dung encoder TRAI");
      }
    }

    _lastDrivePulses = d;
    if (d >= lastProgress + 2) { lastProgress = d; lastProgressMs = millis(); }
    else if (millis() - lastProgressMs > ENC_STALL_MS) {
      motor.stop();
      notify("Encoder khong dem (hoac robot bi ket): dung lai. Kiem tra day encoder.");
      reachedTarget = false;
      break;
    }

    while (cellsDone < totalCells && d * 2 >= (2L * cellsDone + 1) * _cellPulses) {   // qua nửa ô
      advanceCell();
      cellsDone++;
    }
    if (d >= target) break;
    if (millis() - startMs > 15000) { Serial.println("[RobotNav] timeout chay 1 o"); reachedTarget = false; break; }

    WallDist w = tof.read();
    _lastDist = {w.front_mm, w.left_mm, w.right_mm, w.front_ok, w.left_ok, w.right_ok};

    // an toàn: có tường sát phía trước thì dừng ngay (encoder sai / trượt bánh vẫn không đâm tường)
    if (w.front_ok && w.front_mm < FRONT_STOP_MM) {
      Serial.printf("[RobotNav] dung vi tuong truoc %dmm (encoder moi %ld xung)\n", w.front_mm, d);
      reachedTarget = false;
      break;
    }

    smoothL = (w.left_ok  && w.left_mm  < 800) ? 0.5f * w.left_mm  + 0.5f * smoothL : 999.0f;
    smoothR = (w.right_ok && w.right_mm < 800) ? 0.5f * w.right_mm + 0.5f * smoothR : 999.0f;

    bool left  = smoothL > 20 && smoothL < WALL_THRESHOLD_MM;
    bool right = smoothR > 20 && smoothR < WALL_THRESHOLD_MM;

    // PID bám tường theo khoảng cách thực tế tại tâm ô (thuật toán của project mm).
    // correction > 0 => bánh phải nhanh hơn => lái sang TRÁI.
    float error, correction;
    gyro.update();
    if (left || right) {
      if (left && right) error = (smoothL - smoothR) - WALL_CENTER_OFFSET_MM;   // lệch tâm ô
      else if (left)     error = smoothL - WALL_TARGET_LEFT_MM;                 // chỉ bám tường trái
      else               error = WALL_TARGET_RIGHT_MM - smoothR;                // chỉ bám tường phải
      if (fabsf(error) < WALL_ERR_DEADBAND_MM) error = 0.0f;
      correction = _wallPid.computeError(error, 0.02f);
    } else if (gyro.ok()) {
      // ngã tư / không có tường: giữ hướng bằng gyro (yaw * _gyroSign > 0 => đang lệch PHẢI)
      correction = _gyroPid.computeError(gyro.getYaw() * _gyroSign, 0.02f);
    } else {
      long el = encoder.left(), er = encoder.right();
      correction = _syncPid.computeError((float)(el - er), 0.02f);
    }

    // giảm tốc gần cuối ô
    float remain = (float)(target - d);
    const float crawlZone = _cellPulses * CRAWL_CELL_FRAC;
    float speed = pwmCmd;
    const float crawl = fminf(CRAWL_PWM, pwmCmd);   // tốc độ đặt thấp hơn CRAWL thì không tăng lên
    if (remain < crawlZone) {
      speed = crawl + (remain / crawlZone) * (pwmCmd - crawl);
      if (speed < crawl) speed = crawl;
    }

    int L = constrain((int)(speed - correction), 0, 255);   // không cho bánh chạy lùi khi đang tiến
    int R = constrain((int)(speed + correction), 0, 255);
    motor.run(L, R);
  }

  // chỉ ép đủ totalCells khi THỰC SỰ đã đi hết quãng đường (d >= target ở vòng lặp trên);
  // dừng sớm vì gặp tường / hết giờ thì KHÔNG được báo đã sang ô mà robot chưa thực sự tới.
  if (reachedTarget) { while (cellsDone < totalCells) { advanceCell(); cellsDone++; } }

  motor.stop();
  delay(30);
  encoder.reset();
  gyro.setMoving(false);
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

bool RobotNav::buildPlannedPath(int startCx, int startCy, Heading startH) {
  _plannedLen = 0;
  _plannedPath[0] = '\0';
  floodFill(true);   // fast run chỉ đi qua tường đã xác nhận là trống
  int startMx = startCx * 2 - 1, startMy = startCy * 2 - 1;
  if (_dist[startMx][startMy] < 0) return false;

  int mx = startMx, my = startMy;
  Heading h = startH;
  int guard = 0;

  while (_dist[mx][my] != 0 && guard < _cells * _cells) {
    Heading nextH = chooseBestDirectionSolved(mx, my, h);
    int nx = mx + dxArr[nextH], ny = my + dyArr[nextH];
    if (nx < 0 || ny < 0 || nx >= _size || ny >= _size) return false;
    if (!canMove(mx, my, nx, ny, true)) return false;
    if (_dist[nx][ny] < 0) return false;

    int diff = (nextH - h + 4) % 4;
    if (diff == 1) appendPathCmd('R');
    else if (diff == 3) appendPathCmd('L');
    else if (diff == 2) { appendPathCmd('R'); appendPathCmd('R'); }
    appendPathCmd('F');

    mx = nx; my = ny; h = nextH;
    guard++;
  }
  return _dist[mx][my] == 0;
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
      driveCells(fCount, _driveSpeed);
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
  prefs.putInt("goalx", _goalRow * 100 + _goalSize);
  prefs.putInt("goaly", _goalCol);
  prefs.putInt("msize", _size);
  size_t written = prefs.putBytes("maze", _maze, sizeof(_maze));   // lưu cả mảng MAX (1089 byte)
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

  if (!valid || ver != MAP_VERSION || gx != _goalRow * 100 + _goalSize || gy != _goalCol || msize != _size) {
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

// Nút "RESET TOÀN BỘ MAP" trên web: chỉ xoá bản đồ + đường đi đã học,
// KHÔNG đụng tới cấu hình kích thước/ô đích (mmcfg) đã lưu riêng.
void RobotNav::resetAll() {
  motor.stop();
  clearMazeFlash();
  resetMaze();
  _mapChanged = true;
  Serial.println("[RobotNav] Da RESET toan bo ban do.");
}

String RobotNav::getMazeString() const {
  String out = "";
  out.reserve(_size * _size + 1);
  for (int i = 0; i < _size; i++) {
    for (int j = 0; j < _size; j++) {
      out += String(_maze[i][j]);
    }
  }
  return out;
}