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
  void startExplore();
  void startFastRun();
  void stop();
  void resetMaze();

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
  String getMazeString() const;

private:
  bool _mapChanged = false;
  static const int SIZE = MAZE_SIZE;

  int _maze[SIZE][SIZE];
  int _dist[SIZE][SIZE];

  int _cx = 1, _cy = 1;
  Heading _heading = EAST;
  NavState _state = NAV_IDLE;

  int _goalX1 = 1, _goalY1 = 1;
  int _goalX2 = 15, _goalY2 = 15;

  bool _fastRunReady = false;
  char _plannedPath[256];
  int  _plannedLen = 0;

  struct WallDistLocal { int front_mm, left_mm, right_mm; bool front_ok, left_ok, right_ok; };
  WallDistLocal _lastDist{};

  PIDController _wallPid{12.0f, 0.0f, 4.0f, -60, 60};  // bám tường: error(mm) -> hiệu chỉnh PWM
  PIDController _syncPid{0.08f, 0.0f, 0.0f, -60, 60};

  // ----- flood fill -----
  void floodFill(int goalMx, int goalMy);
  bool canMove(int x, int y, int nx, int ny);
  Heading chooseBestDirection(int cx, int cy, Heading h);

  // ----- wall sensing -----
  void detectAndUpdateWalls(int cx, int cy, Heading h);
  void updateFrontWall(int cx, int cy, Heading h);
  void updateLeftWall(int cx, int cy, Heading h);
  void updateRightWall(int cx, int cy, Heading h);
  void clearFrontWall(int cx, int cy, Heading h);
  void clearLeftWall(int cx, int cy, Heading h);
  void clearRightWall(int cx, int cy, Heading h);
  void finalizeUnknownWalls();

  // ----- motion -----
  void rotateTo(Heading target);
  void turn90(int dir);              // dir: -1 trái, +1 phải
  bool driveOneCell();               // chạy 1 ô (18cm), wall-follow PID, true nếu ok
  void driveDistanceCm(float cm, int pwm);

  // ----- fast run path planning -----
  bool buildPlannedPath(int startMx, int startMy, Heading startH, int goalMx, int goalMy);
  Heading chooseBestDirectionSolved(int mx, int my, Heading h);
  void appendPathCmd(char c);
  void executePlannedPath();

  // ----- flash persistence -----
  bool saveMazeToFlash();
  bool loadMazeFromFlash();
  void clearMazeFlash();
};

extern RobotNav robotNav;