#pragma once
#include <Arduino.h>
#include <string.h>
#include "RobotConfig.h"

enum Heading { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// Lưới bàn cờ 2 lớp ô/tường: 1=tường ngoài, 2/3=tường đã biết (dọc/ngang),
// 4/5=tường chưa biết (dọc/ngang, hiển thị "?"), 0=trống.
// Class này chỉ lo lưu trữ + truy vấn bản đồ (tường, cấu hình kích thước/đích, lưu/nạp flash).
// Thuật toán tìm đường (flood fill) nằm ở class Floodfill, dùng Map làm dữ liệu vào.
class Map {
public:
  static const int MAX_SIZE = MAZE_MAX_SIZE;

  // cấu hình kích thước mê cung + vùng đích — KHÔNG đụng tới lưới tường (gọi reset() sau nếu cần)
  void configure(int cells, int goalRow, int goalCol, int goalSize);
  void reset();   // lưới tường về trạng thái ban đầu: chưa biết tường trong, biết tường ngoài

  int cells() const { return _cells; }
  int size() const  { return _size; }
  int goalRow() const  { return _goalRow; }
  int goalCol() const  { return _goalCol; }
  int goalSize() const { return _goalSize; }
  bool isGoalCell(int cx, int cy) const;

  // toạ độ lưới (0.._size-1): số chẵn = ô/nút tường bao, số lẻ = tâm ô hoặc tường giữa 2 ô
  uint8_t cellRaw(int x, int y) const { return _maze[x][y]; }
  bool isOpen(int x, int y) const { return _maze[x][y] == 0; }

  // true nếu di chuyển từ ô lưới (x,y) sang ô liền kề (nx,ny) không bị chặn.
  // unknownIsWall = true: coi tường chưa biết (4/5) là tường thật (dùng khi lập đường fast run).
  bool canMove(int x, int y, int nx, int ny, bool unknownIsWall = false) const;

  // Ô tường nằm giữa ô (cx,cy) và ô kề theo hướng tuyệt đối `side`
  void wallIndex(int cx, int cy, Heading side, int& wx, int& wy) const;
  void setWall(int cx, int cy, Heading side, bool present);   // ghi đè theo lần đo mới nhất
  bool wallKnown(int cx, int cy, Heading side) const;         // đã xác nhận có tường ở phía đó

  // theo dõi tường đã đo lại trong 1 lượt chạy (dùng cho chế độ CẬP NHẬT MÊ CUNG)
  void beginWallScan() { memset(_wallSeen, 0, sizeof(_wallSeen)); }
  bool forgetUnseenWalls();   // tường cũ chưa đo lại => về "chưa biết"; true nếu có thay đổi

  // ----- lưu/nạp bản đồ đã dò được từ flash -----
  bool saveToFlash();
  bool loadFromFlash();
  void clearFlash();

  String toString() const;   // mỗi ô lưới 1 ký tự số — dùng gửi bản đồ qua BLE

private:
  int _cells = MAZE_CELLS;
  int _size = 2 * MAZE_CELLS + 1;
  int _goalRow = GOAL_ROW, _goalCol = GOAL_COL, _goalSize = GOAL_SIZE;

  uint8_t _maze[MAX_SIZE][MAX_SIZE];
  bool _wallSeen[MAX_SIZE][MAX_SIZE];
};
