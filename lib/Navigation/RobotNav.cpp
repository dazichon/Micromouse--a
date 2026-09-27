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
  if (_map.loadFromFlash()) {
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
  _map.configure(n, gr, gc, gs);
  Serial.printf("[RobotNav] Me cung %dx%d, dich (%d,%d) %dx%d%c", n, n, gr, gc, gs, gs, 10);
}

String RobotNav::setMazeConfig(int cells, int goalRow, int goalCol, int goalSize) {
  if (_state == NAV_EXPLORING || _state == NAV_FAST_RUN || _state == NAV_PID_TEST || _state == NAV_PID_TEST_CELL) return "Robot dang chay, gui STOP truoc";
  if (cells < 2 || cells > MAZE_MAX_CELLS) return "Kich thuoc phai tu 2 den " + String(MAZE_MAX_CELLS);
  if (goalSize < 1 || goalSize > 2) return "Vung dich chi 1x1 hoac 2x2";
  if (goalRow < 1 || goalCol < 1 || goalRow + goalSize - 1 > cells || goalCol + goalSize - 1 > cells)
    return "Dich nam ngoai me cung";
  if (goalRow == 1 && goalCol == 1) return "Dich khong duoc trung o xuat phat (1,1)";

  _map.configure(cells, goalRow, goalCol, goalSize);

  prefs.begin("mmcfg", false);
  prefs.putInt("cells", cells);
  prefs.putInt("grow", goalRow);
  prefs.putInt("gcol", goalCol);
  prefs.putInt("gsize", goalSize);
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
  _map.reset();
  _floodfill.reset();
  _cx = 1; _cy = 1;
  _heading = EAST;
  _plannedLen = 0;
  _plannedPath[0] = '\0';
  _fastRunReady = false;
  _state = NAV_IDLE;
  encoder.reset();
  _wallPid.reset();
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
  _map.beginWallScan();
  _cx = 1; _cy = 1;
  _heading = EAST;
  encoder.reset();
  _wallPid.reset();
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

// Chế độ 2: chạy PID bám tường LIÊN TỤC, TỰ DO — port 1:1 startPID()/updatePIDLoop() của project mm
// (xem pidTestStep()) để chỉnh Kp/Ki/Kd live (SET_KP/KI/KD) và xem xe phản ứng ra sao. KHÔNG dùng
// driveCells()/advanceCell(): không đếm ô, không đụng _cx/_cy, không gửi "map" lên web — tab Bản đồ
// đứng yên trong lúc test PID, chỉ tab PID mới có ý nghĩa.
// pidTestStep() chạy 1 nhịp PID mỗi lần gọi (không block), nên STOP có tác dụng gần như ngay lập tức.
// Gặp vách tường trước mặt: DỪNG HẲN về NAV_IDLE (giống mm), không tự chạy lại — khác PIDTEST cũ.
bool RobotNav::startPidTest() {
  if (_driveSpeed <= 0) {
    Serial.println("[RobotNav] Toc do = 0, khong the test PID.");
    return false;
  }
  encoder.reset();
  _wallPid.reset();
  gyro.calibrate();
  gyro.setMoving(true);
  gyro.update();
  _ptTargetYaw = gyro.getYaw();   // mốc góc giữ thẳng khi mất tường — giống _targetYaw trong project mm
  _ptSmoothL = WALL_TARGET_LEFT_MM;
  _ptSmoothR = WALL_TARGET_RIGHT_MM;
  _ptLastEncL = encoder.left();
  _ptLastEncR = encoder.right();
  _ptSmoothErr = 0.0f;
  _ptLastStepUs = 0;
  _state = NAV_PID_TEST;
  Serial.println("[RobotNav] Bat dau TEST PID (chay tu do, port 1:1 tu project mm)");
  return true;
}

// Chế độ 2 - biến thể "đi từng ô": mỗi lần đi đúng _cellPulses xung rồi dừng ngắn, giống
// driveOneCell() ở Explore/Chế độ 3 (đồng tốc DSPD, giảm tốc cuối đoạn) nhưng gọi driveCells()
// với trackMap=false nên KHÔNG advanceCell/không đụng _cx,_cy/không báo "map" lên web.
bool RobotNav::startPidTestCell() {
  if (_driveSpeed <= 0) {
    Serial.println("[RobotNav] Toc do = 0, khong the test PID.");
    return false;
  }
  encoder.reset();
  _wallPid.reset();
  gyro.calibrate();
  _stuckCount = 0;
  _state = NAV_PID_TEST_CELL;
  Serial.println("[RobotNav] Bat dau TEST PID TUNG O (khong theo ban do)");
  return true;
}

void RobotNav::stop() {
  motor.stop();
  gyro.setMoving(false);
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
      _floodfill.compute(_map);

      // Cập nhật: mê cung có thể đã bị sửa ở chỗ robot chưa đi lại => tường cũ chặn hết đường.
      // Bỏ các tường chưa đo lại trong lần chạy này (về "chưa biết") rồi tính lại.
      if (_floodfill.dist(_cx * 2 - 1, _cy * 2 - 1) < 0 && _updateMode && _map.forgetUnseenWalls()) {
        Serial.println("[RobotNav] Ban do cu khong con duong, bo cac tuong chua xac nhan lai");
        _floodfill.compute(_map);
      }

      // Không còn đường tới đích theo bản đồ hiện tại => KHÔNG chạy (trước đây robot lao thẳng vào tường).
      // Đo lại vài lần — tường ghi nhầm do nhiễu sẽ được xoá ở lần đo sau.
      if (_floodfill.dist(_cx * 2 - 1, _cy * 2 - 1) < 0) {
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

      Heading target = _floodfill.chooseBestDirection(_map, _cx, _cy, _heading);
      if (target != _heading) {
        rotateTo(target);
        // KẾT HỢP BẢN ĐỒ + CẢM BIẾN: hướng mới chỉ được chọn theo bản đồ (tường đo từ cảm biến bên,
        // có thể đọc nhầm). Quay xong đo lại bằng cảm biến trước — có tường thật thì ghi vào bản đồ
        // và để vòng sau tính lại đường, KHÔNG lao vào tường.
        detectAndUpdateWalls(_cx, _cy, _heading);
        if (frontWallKnown()) {
          Serial.println("[RobotNav] Quay xong thay tuong truoc mat -> tinh lai duong");
          break;
        }
      }

      driveOneCell();   // _cx/_cy được cập nhật bên trong driveCells

      // dừng hẳn 1 chút ở ô mới: cảm biến hết rung/ổn định số đo, web kịp hiển thị
      // vị trí ô mới trước khi đi tiếp (đi từng ô rõ ràng thay vì lướt liên tục)
      unsigned long tPause = millis();
      while (millis() - tPause < CELL_PAUSE_MS) idle();

      if (_map.isGoalCell(_cx, _cy)) {
        motor.stop();
        Serial.println(_updateMode ? "[RobotNav] Toi dich! Da cap nhat ban do." : "[RobotNav] Toi dich! Da khoa ban do.");
        _map.saveToFlash();
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

    case NAV_PID_TEST:
      pidTestStep();
      break;

    case NAV_PID_TEST_CELL:
      pidTestCellStep();
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

  // ghi ĐÈ trạng thái theo lần đo mới nhất (cả thêm lẫn xoá) — tường ghi nhầm sẽ tự sửa khi đo lại.
  // Cảm biến không trả đủ mẫu => KHÔNG ghi (trước đây mẫu lỗi bị coi là "trống" => đường giả => đâm tường)
  if (fOk >= 2) _map.setWall(cx, cy, h, front);
  if (lOk >= 2) _map.setWall(cx, cy, (Heading)((h + 3) % 4), left);
  if (rOk >= 2) _map.setWall(cx, cy, (Heading)((h + 1) % 4), right);
  _mapChanged = true;

  Serial.printf("[RobotNav] o(%d,%d) %c  F=%d%s L=%d%s R=%d%s\n", cx, cy, headingChar(),
                fm, front ? "#" : "", lm, left ? "#" : "", rm, right ? "#" : "");
}

bool RobotNav::frontWallKnown() const {
  return _map.wallKnown(_cx, _cy, _heading);
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

  // Quay bù nếu còn lố/thiếu (chỉ khi đo bằng gyro — encoder không biết chiều). Nhích NGẮN + phanh +
  // chờ ổn định rồi mới đo lại (giống alignInCell()) — bản cũ quay LIÊN TỤC ở TURN_MIN_PWM rồi chỉ
  // phanh 1 LẦN DUY NHẤT sau khi thoát vòng lặp, nên quán tính lúc phanh cuối luôn làm lố thêm vài độ
  // mà không được đo/sửa lại — xảy ra NHƯ NHAU dù LEFT_COMPENSATION/RIGHT_COMPENSATION là bao nhiêu
  // (comp chỉ cắt sớm pha thô ở trên, không liên quan pha bù này). Dùng _gyroSign đã học (ổn định
  // hơn đoán lại dấu từ 1 mẫu yaw có thể còn nhiễu).
  if (_lastTurnSrc == 'G') {
    // học dấu gyro theo chiều quay thật => kim đồng hồ/PID luôn dùng đúng dấu
    float sgn = (gyro.getYaw() >= 0) ? 1.0f : -1.0f;
    if (fabs(gyro.getYaw()) > 20.0f) _gyroSign = sgn * dir;

    const float compSgn = _gyroSign * dir;
    unsigned long compStart = millis();
    while (millis() - compStart < 1200) {
      gyro.update();
      float done = gyro.getYaw() * compSgn;
      float err = done - targetDeg;        // >0 = lố (quay quá), <0 = thiếu (chưa đủ)
      if (fabs(err) <= TURN_TOL_DEG) break;
      int cdir = (err > 0) ? -dir : dir;   // lố => quay ngược lại; thiếu => quay thêm đúng chiều cũ
      motor.run(cdir * ALIGN_PWM_MIN, -cdir * ALIGN_PWM_MIN);
      delay(ALIGN_NUDGE_MS);
      motor.brake();
      delay(20);
      motor.stop();
      unsigned long tS = millis();
      while (millis() - tS < ALIGN_SETTLE_MS) gyro.update();
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
  if (_state == NAV_EXPLORING || _state == NAV_FAST_RUN || _state == NAV_PID_TEST || _state == NAV_PID_TEST_CELL) return false;
  turnDeg(dir, deg);
  return true;
}

// Điều khiển tay từng ô (Chế độ 3). Sau mỗi lần quay / đi đều đo lại tường ở ô hiện tại
// để bản đồ trên web cập nhật như khi explore.
String RobotNav::manualStep(char action) {
  if (_state == NAV_EXPLORING || _state == NAV_FAST_RUN || _state == NAV_PID_TEST || _state == NAV_PID_TEST_CELL) return "Robot dang chay, gui STOP truoc";
  if (action != 'F' && action != 'L' && action != 'R' && action != 'B') return "Lenh khong hop le";
  if (action != 'B' && _driveSpeed <= 0) return "Toc do = 0, tang toc do truoc";
  _lastDrivePulses = 0;

  if (action == 'L') turn90(-1);
  else if (action == 'R') turn90(+1);
  else if (action == 'B') turnDeg(+1, 180.0f);

  detectAndUpdateWalls(_cx, _cy, _heading);
  if (action == 'B') return "";

  // có tường ngay trước mặt => không chạy (tránh đâm tường)
  if (frontWallKnown()) return "Co tuong phia truoc (" + String(_lastDist.front_mm) + "mm), khong di";

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
  _cx = constrain(_cx, 1, _map.cells());
  _cy = constrain(_cy, 1, _map.cells());
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
  return driveCells(1, _driveSpeed);
}

// 1 bước PID bám tường (dùng chung cho driveCells() và pidTestStep() ở Chế độ 2) — port đúng thuật
// toán project mm: EMA khoảng cách -> sai số thô theo từng trường hợp tường -> trộn encoder ->
// lọc thông thấp lần 2 -> kẹp trần -> 1 PID DUY NHẤT (_wallPid, Kd=0 để tránh giật do đạo hàm
// nhiễu ToF). smoothL/smoothR/pidMode/lastEncL/lastEncR/smoothErr là state của người gọi, truyền
// vào/ra qua tham chiếu để mỗi nơi gọi (driveCells/pidTestStep) tự giữ state riêng của mình.
float RobotNav::wallFollowCorrection(const WallDist& w, float& smoothL, float& smoothR, int& pidMode,
                                      long& lastEncL, long& lastEncR, float& smoothErr) {
  // EMA khoảng cách 2 bên: còn trong tầm bám nhanh (alpha lớn); mất tường/lóa thì trôi CHẬM về
  // "coi như trống" (999) để không giật khi vừa mất/vừa lại thấy tường.
  smoothL = (w.left_ok && w.left_mm < 800)
              ? WALL_SMOOTH_ALPHA_NEAR * w.left_mm + (1.0f - WALL_SMOOTH_ALPHA_NEAR) * smoothL
              : WALL_SMOOTH_ALPHA_FAR * WALL_SMOOTH_FAR_MM + (1.0f - WALL_SMOOTH_ALPHA_FAR) * smoothL;
  smoothR = (w.right_ok && w.right_mm < 800)
              ? WALL_SMOOTH_ALPHA_NEAR * w.right_mm + (1.0f - WALL_SMOOTH_ALPHA_NEAR) * smoothR
              : WALL_SMOOTH_ALPHA_FAR * WALL_SMOOTH_FAR_MM + (1.0f - WALL_SMOOTH_ALPHA_FAR) * smoothR;

  bool left  = smoothL > 20 && smoothL < WALL_THRESHOLD_MM;
  bool right = smoothR > 20 && smoothR < WALL_THRESHOLD_MM;

  // Sai số đồng bộ 2 bánh tính theo CHÊNH LỆCH GIỮA 2 LẦN GỌI GẦN NHẤT (không cộng dồn từ đầu đoạn
  // chạy) — đi thẳng đều thì lệch này ≈0 ở mỗi vòng lặp.
  long curL = encoder.left(), curR = encoder.right();
  float enc_error = (float)((curL - lastEncL) - (curR - lastEncR));
  lastEncL = curL;
  lastEncR = curR;

  gyro.update();
  const int mode = (left && right) ? 0 : left ? 1 : right ? 2 : 3;   // 3 = không có tường (ngã tư)
  if (mode != pidMode) {
    _wallPid.reset();
    // mất tường để bám => giữ hướng hiện tại bằng gyro (không kéo về hướng lúc bắt đầu chạy)
    if (mode == 3) { gyro.update(); gyro.resetYaw(); }
    pidMode = mode;
  }

  float raw_error;
  if (mode <= 2) {
    float wall_error;
    if (mode == 0)      wall_error = (smoothL - smoothR) - WALL_CENTER_OFFSET_MM;   // lệch tâm ô
    else if (mode == 1) wall_error = smoothL - WALL_TARGET_LEFT_MM;                 // chỉ bám tường trái
    else                wall_error = WALL_TARGET_RIGHT_MM - smoothR;                // chỉ bám tường phải
    raw_error = WALL_ENC_BLEND_W1 * wall_error + WALL_ENC_BLEND_W2 * enc_error;
  } else if (gyro.ok()) {
    // ngã tư/ô trống: giữ thẳng bằng gyro + encoder (yaw * _gyroSign > 0 => đang lệch PHẢI)
    float gyro_error = gyro.getYaw() * _gyroSign;
    raw_error = NOWALL_GYRO_WEIGHT * gyro_error + NOWALL_ENC_WEIGHT * enc_error;
  } else {
    // gyro lỗi: chỉ còn encoder để giữ thẳng
    raw_error = enc_error;
  }

  // Lọc thông thấp lần 2 trên sai số ĐÃ TRỘN (chống giật do gai nhọn/khe nứt tường tức thời),
  // rồi kẹp trần trước khi đưa vào PID.
  smoothErr = WALL_ERR_LOWPASS_ALPHA * raw_error + (1.0f - WALL_ERR_LOWPASS_ALPHA) * smoothErr;
  float final_error = constrain(smoothErr, -WALL_ERR_MAX_MM, WALL_ERR_MAX_MM);

  // correction > 0 => bánh phải nhanh hơn => lái sang TRÁI.
  float correction = _wallPid.computeError(final_error, DRIVE_LOOP_MS / 1000.0f);

  return correction;
}

bool RobotNav::driveCells(int cells, int pwmCmd, bool trackMap) {
  gyro.setMoving(true);
  encoder.reset();
  _wallPid.reset();
  gyro.update();
  gyro.resetYaw();   // giữ hướng lúc bắt đầu chạy khi không có tường để bám

  // lọc EMA khoảng cách 2 bên (như project mm) — giảm giật do nhiễu VL53
  float smoothL = WALL_TARGET_LEFT_MM, smoothR = WALL_TARGET_RIGHT_MM;
  long lastEncL = encoder.left(), lastEncR = encoder.right();
  float smoothErr = 0.0f;

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
  int  pidMode = -1;   // đổi kiểu bám (2 tường / 1 tường / gyro) => reset PID, tránh giật do D và I cũ

  while (true) {
    if (millis() - lastStepMs < DRIVE_LOOP_MS) { idle(); continue; }   // theo tốc độ đo của VL53
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
      if (trackMap) advanceCell();
      cellsDone++;
    }
    if (d >= target) break;
    if (millis() - startMs > 15000) { Serial.println("[RobotNav] timeout chay 1 o"); reachedTarget = false; break; }

    WallDist w = tof.read();
    _lastDist = {w.front_mm, w.left_mm, w.right_mm, w.front_ok, w.left_ok, w.right_ok};

    // an toàn: có tường sát phía trước thì dừng ngay (encoder sai / trượt bánh vẫn không đâm tường)
    if (w.front_ok && w.front_mm < FRONT_STOP_MM) {   // kể cả che sát cảm biến (~0mm) cũng phải dừng
      // motor.stop() ở cuối hàm chỉ THẢ TRÔI — với quán tính hiện tại xe vẫn trôi tiếp và đâm tường
      // trong quãng FRONT_STOP_MM còn lại. Phanh cứng ngay tại đây rồi mới thả trôi.
      motor.brake();
      delay(60);
      // vị trí đã tự sang ô mới khi qua nửa ô => ô đang đứng là đúng, chỉ không ép thêm các ô còn lại
      Serial.printf("[RobotNav] dung vi tuong truoc %dmm (encoder %ld/%ld xung)\n", w.front_mm, d, target);
      reachedTarget = false;
      break;
    }

    float correction = wallFollowCorrection(w, smoothL, smoothR, pidMode, lastEncL, lastEncR, smoothErr);

    // giảm tốc gần cuối ô
    float remain = (float)(target - d);
    const float crawlZone = _cellPulses * CRAWL_CELL_FRAC;
    float speed = pwmCmd;
    const float crawl = fminf(CRAWL_PWM, pwmCmd);   // tốc độ đặt thấp hơn CRAWL thì không tăng lên
    if (remain < crawlZone) {
      speed = crawl + (remain / crawlZone) * (pwmCmd - crawl);
      if (speed < crawl) speed = crawl;
    }
    // tường trước đã gần => chạy chậm để dừng kịp ở FRONT_STOP_MM, không lao vào do quán tính
    if (w.front_ok && w.front_mm < FRONT_SLOW_MM) speed = fminf(speed, crawl);

    // sàn PWM 50 giống project mm: không bao giờ để 1 bánh gần như dừng (correction lớn) khiến xe
    // xoay ngoặt tại chỗ thay vì bám tường mượt — bánh còn lại vẫn bị kẹp trần 255 như cũ.
    int L = constrain((int)(speed - correction), 50, 255);
    int R = constrain((int)(speed + correction), 50, 255);
    motor.run(L, R);
  }

  // chỉ ép đủ totalCells khi THỰC SỰ đã đi hết quãng đường (d >= target ở vòng lặp trên);
  // dừng sớm vì gặp tường / hết giờ thì KHÔNG được báo đã sang ô mà robot chưa thực sự tới.
  if (reachedTarget) { while (cellsDone < totalCells) { if (trackMap) advanceCell(); cellsDone++; } }

  motor.stop();
  delay(30);
  encoder.reset();
  gyro.setMoving(false);
  return reachedTarget;
}

// Chế độ 2 (test PID) — chạy 1 nhịp PID bám tường mỗi lần update() gọi tới (nhịp theo
// DRIVE_LOOP_MS, giống tốc độ đo VL53), KHÔNG block như driveCells() => STOP có tác dụng
// gần như ngay lập tức thay vì trễ hết 1 ô. Không đếm ô, không đụng bản đồ/vị trí.
// Gặp vật cản/tường sát trước mặt: phanh & giữ nguyên, vật cản rời đi thì tự chạy tiếp
// (tiện để đưa tay lại gần test phản ứng PID/an toàn mà không cần STOP/PIDTEST lại).
// Port 1:1 updatePIDLoop() của project mm — KHÔNG dùng wallFollowCorrection()/driveCells() (2 hàm
// đó phục vụ Explore/FastRun/Chế độ 3, có thêm giảm tốc cuối ô/reset PID theo mode... không có
// trong mm). Thứ tự tính toán, hằng số, công thức sai số, dt thực đo bằng micros(), sàn/trần PWM
// [50,255] — tất cả giống hệt updatePIDLoop() trong RobotNav.cpp của mm.
void RobotNav::pidTestStep() {
  unsigned long nowUs = micros();
  float dt = (_ptLastStepUs == 0) ? 0.01f : (nowUs - _ptLastStepUs) / 1000000.0f;
  if (dt > 0.5f || dt <= 0.0f) dt = 0.01f;
  _ptLastStepUs = nowUs;

  WallDist w = tof.read();
  _lastDist = {w.front_mm, w.left_mm, w.right_mm, w.front_ok, w.left_ok, w.right_ok};
  _lastDrivePulses = (labs(encoder.left()) + labs(encoder.right())) / 2;

  // 1. Phanh dừng an toàn khi gặp vách tường trước — giống mm: DỪNG HẲN, không tự chạy lại.
  if (w.front_ok && w.front_mm < FRONT_STOP_MM) {   // kể cả che sát cảm biến (~0mm) cũng phải dừng
    motor.stop();
    gyro.setMoving(false);
    _state = NAV_IDLE;
    notify(("Test PID: dung, gap vach tuong truoc (" + String(w.front_mm) + "mm)").c_str());
    return;
  }

  bool hasLeftWall  = w.left_ok  && w.left_mm  > 20 && w.left_mm  < WALL_THRESHOLD_MM;
  bool hasRightWall = w.right_ok && w.right_mm > 20 && w.right_mm < WALL_THRESHOLD_MM;

  // EMA làm mượt khoảng cách 2 bên (khởi tạo = target lúc startPidTest())
  _ptSmoothL = (w.left_ok && w.left_mm < 800)
                 ? WALL_SMOOTH_ALPHA_NEAR * w.left_mm + (1.0f - WALL_SMOOTH_ALPHA_NEAR) * _ptSmoothL
                 : WALL_SMOOTH_ALPHA_FAR * WALL_SMOOTH_FAR_MM + (1.0f - WALL_SMOOTH_ALPHA_FAR) * _ptSmoothL;
  _ptSmoothR = (w.right_ok && w.right_mm < 800)
                 ? WALL_SMOOTH_ALPHA_NEAR * w.right_mm + (1.0f - WALL_SMOOTH_ALPHA_NEAR) * _ptSmoothR
                 : WALL_SMOOTH_ALPHA_FAR * WALL_SMOOTH_FAR_MM + (1.0f - WALL_SMOOTH_ALPHA_FAR) * _ptSmoothR;

  // Sai số đồng bộ 2 bánh: chênh lệch xung GIỮA 2 LẦN GỌI GẦN NHẤT
  long curL = encoder.left(), curR = encoder.right();
  float enc_error = (float)((curL - _ptLastEncL) - (curR - _ptLastEncR));
  _ptLastEncL = curL;
  _ptLastEncR = curR;

  float raw_error;
  if (hasLeftWall && hasRightWall) {
    float wall_error = (_ptSmoothL - _ptSmoothR) - WALL_CENTER_OFFSET_MM;
    raw_error = 0.7f * wall_error + 0.35f * enc_error;
  } else if (hasLeftWall) {
    float wall_error = _ptSmoothL - WALL_TARGET_LEFT_MM;
    raw_error = 0.7f * wall_error + 0.35f * enc_error;
  } else if (hasRightWall) {
    float wall_error = WALL_TARGET_RIGHT_MM - _ptSmoothR;
    raw_error = 0.7f * wall_error + 0.35f * enc_error;
  } else {
    // Không có tường (ngã tư): phối hợp Encoder & Gyro giữ thẳng tuyệt đối, giống mm
    gyro.update();
    float gyro_error = _ptTargetYaw - gyro.getYaw();
    raw_error = 0.6f * gyro_error + 0.4f * enc_error;
  }

  // Lọc thông thấp lần 2 (ALPHA = 0.3) rồi kẹp trần trước khi đưa vào PID — giống mm
  _ptSmoothErr = WALL_ERR_LOWPASS_ALPHA * raw_error + (1.0f - WALL_ERR_LOWPASS_ALPHA) * _ptSmoothErr;
  float final_error = constrain(_ptSmoothErr, -WALL_ERR_MAX_MM, WALL_ERR_MAX_MM);
  float pidOut = _wallPid.computeError(final_error, dt);

  int L = constrain((int)(_driveSpeed - pidOut), 50, 255);
  int R = constrain((int)(_driveSpeed + pidOut), 50, 255);
  motor.run(L, R);
}

// Chế độ 2 (biến thể đi từng ô) — đi đúng _cellPulses xung (wall-follow PID + giảm tốc cuối đoạn,
// dùng chung driveCells(1, speed, trackMap=false) nên KHÔNG advanceCell/không đụng bản đồ), dừng
// hẳn, rồi tự CANH GIỮA Ô (alignInCell) trước khi đi tiếp — chứ không đi liên tục nối đoạn.
// driveCells() CHẶN tới khi đi hết 1 đoạn nên STOP trễ tối đa ~1 đoạn (_cellPulses).
void RobotNav::pidTestCellStep() {
  if (!driveCells(1, _driveSpeed, false)) {
    motor.stop();
    WallDist w = tof.read();
    // Tường/vật cản trước mặt: GIỮ NGUYÊN, không bỏ cuộc — hết vật cản tự đi tiếp, giống Tự Do.
    if (w.front_ok && w.front_mm < FRONT_STOP_MM) {   // kể cả che sát cảm biến (~0mm) cũng phải dừng
      Serial.println("[RobotNav] Test PID tung o: co vat can truoc mat, dung cho...");
      delay(150);
      return;
    }
    // Lý do khác (encoder khong dem / ket co khi): thu vai lan roi moi bo cuoc, tranh ket cung mai.
    if (++_stuckCount < 3) {
      Serial.println("[RobotNav] Test PID tung o: dung som, thu lai...");
      delay(200);
      return;
    }
    notify("Test PID tung o: dung lai (kiem tra encoder/co khi). Bam STOP roi thu lai.");
    _state = NAV_IDLE;
    return;
  }
  _stuckCount = 0;

  // đứng yên 1 chút cho cảm biến hết rung rồi mới đo để canh giữa ô
  unsigned long tPause = millis();
  while (millis() - tPause < CELL_PAUSE_MS) idle();

  alignInCell();   // tự xoay chỉnh tại chỗ theo sai số bám tường tới khi thẳng giữa ô
}

// Sau khi dừng ở cuối đoạn: đo lại bằng readFresh() (bỏ mẫu cũ lúc đang chạy/rung, chờ mẫu MỚI
// hoàn toàn) rồi tự xoay tại chỗ theo đúng sai số bám tường (như wallFollowCorrection() lúc đang
// đi) cho tới khi thẳng giữa ô (hoặc chỉ bám 1 tường) mới thôi — bù trôi hướng tích luỹ trong đoạn
// vừa đi. Không có tường 2 bên để bám (giữa ngã tư, ô trống...) thì không có gì để canh, bỏ qua.
void RobotNav::alignInCell() {
  unsigned long t0 = millis();
  float smoothL = WALL_TARGET_LEFT_MM, smoothR = WALL_TARGET_RIGHT_MM;
  bool haveL = false, haveR = false;

  while (millis() - t0 < ALIGN_TIMEOUT_MS) {
    WallDist w = tof.readFresh();
    _lastDist = {w.front_mm, w.left_mm, w.right_mm, w.front_ok, w.left_ok, w.right_ok};

    bool left  = w.left_ok  && w.left_mm  < WALL_THRESHOLD_MM;
    bool right = w.right_ok && w.right_mm < WALL_THRESHOLD_MM;
    if (!left && !right) return;   // không có tường bên nào để canh -> bỏ qua bước này

    if (w.left_ok)  smoothL = haveL ? 0.5f * w.left_mm  + 0.5f * smoothL : (float)w.left_mm;
    if (w.right_ok) smoothR = haveR ? 0.5f * w.right_mm + 0.5f * smoothR : (float)w.right_mm;
    haveL = haveL || w.left_ok;
    haveR = haveR || w.right_ok;

    float error;
    if (left && right)  error = (smoothL - smoothR) - WALL_CENTER_OFFSET_MM;   // lệch tâm ô
    else if (left)       error = smoothL - WALL_TARGET_LEFT_MM;                // chỉ bám tường trái
    else                 error = WALL_TARGET_RIGHT_MM - smoothR;               // chỉ bám tường phải

    if (fabsf(error) < WALL_ERR_DEADBAND_MM) { motor.stop(); return; }   // đã thẳng giữa ô

    // error > 0 => đang cần lái sang TRÁI (giống wallFollowCorrection) => quay trái tại chỗ 1 nhích
    int pwm = constrain((int)(fabsf(error) * 0.8f) + ALIGN_PWM_MIN, ALIGN_PWM_MIN, ALIGN_PWM_MAX);
    int dir = (error > 0) ? -1 : +1;
    motor.run(dir * pwm, -dir * pwm);
    delay(ALIGN_NUDGE_MS);
    motor.brake();
    delay(20);
    motor.stop();

    unsigned long tS = millis();
    while (millis() - tS < ALIGN_SETTLE_MS) idle();   // đứng yên hẳn trước khi đo lại
  }
  motor.stop();
  Serial.println("[RobotNav] Test PID tung o: het gio can chinh, di tiep");
}

// ============================================================================
//  LẬP KẾ HOẠCH ĐƯỜNG ĐI NHANH (từ map đã khám phá, dùng thuật toán trong Floodfill) + CHẠY
// ============================================================================
bool RobotNav::buildPlannedPath(int startCx, int startCy, Heading startH) {
  bool ok = _floodfill.buildPath(_map, startCx, startCy, startH, _plannedPath, sizeof(_plannedPath));
  _plannedLen = strlen(_plannedPath);
  return ok;
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
      // bản đồ nói trống nhưng cảm biến thấy tường => mê cung khác bản đồ, dừng thay vì đâm
      WallDist f = tof.readFresh();
      if (f.front_ok && f.front_mm < WALL_THRESHOLD_MM) {
        motor.stop();
        notify("Fast run: gap tuong khong co trong ban do. Hay chay CAP NHAT ME CUNG.");
        return;
      }
      if (!driveCells(fCount, _driveSpeed)) {
        motor.stop();
        notify("Fast run dung giua duong (tuong / encoder). Hay chay CAP NHAT ME CUNG.");
        return;
      }
    }
  }
  motor.stop();
  Serial.println("[RobotNav] FAST RUN DONE");
}

// Nút "RESET TOÀN BỘ MAP" trên web: chỉ xoá bản đồ + đường đi đã học,
// KHÔNG đụng tới cấu hình kích thước/ô đích (mmcfg) đã lưu riêng.
void RobotNav::resetAll() {
  motor.stop();
  _map.clearFlash();
  resetMaze();
  _mapChanged = true;
  Serial.println("[RobotNav] Da RESET toan bo ban do.");
}

String RobotNav::getMazeString() const {
  return _map.toString();
}