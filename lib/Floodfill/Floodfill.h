#pragma once
#include <Arduino.h>
#include "Map.h"

// Thuật toán flood fill (BFS ngược từ đích) trên lưới của Map: tính số ô ngắn nhất
// tới đích cho mọi ô, chọn hướng đi tốt nhất mỗi bước, và lập sẵn đường đi nhanh (fast run).
class Floodfill {
public:
  void reset();   // _dist về "chưa biết" (-1) — gọi khi reset mê cung

  // flood fill từ TẤT CẢ ô đích (map.goalRow/goalCol/goalSize) => dist(x,y) = số ô tới đích gần nhất.
  // unknownIsWall = true: coi tường chưa biết là tường thật (lập đường fast run chỉ qua đường đã xác nhận).
  void compute(const Map& map, bool unknownIsWall = false);
  int dist(int x, int y) const { return _dist[x][y]; }

  // hướng đi tốt nhất từ ô (cx,cy) [toạ độ ô, 1.._cells] theo _dist đã compute() — dùng khi explore
  Heading chooseBestDirection(const Map& map, int cx, int cy, Heading h) const;
  // giống trên nhưng toạ độ lưới (mx,my) và chỉ đi qua đường đã xác nhận — dùng khi lập đường fast run
  Heading chooseBestDirectionSolved(const Map& map, int mx, int my, Heading h) const;

  // lập đường đi nhanh từ (startCx,startCy,startH) tới đích: tự compute(map, true) rồi dò theo _dist.
  // Ghi chuỗi lệnh 'L'/'R'/'F' (quay trái/phải 90°, đi thẳng 1 ô; quay đầu = 'R','R') vào outPath
  // (đủ chỗ null-terminator, tối đa outCap ký tự). Trả về false nếu không có đường tới đích.
  bool buildPath(const Map& map, int startCx, int startCy, Heading startH, char* outPath, int outCap);

private:
  int _dist[Map::MAX_SIZE][Map::MAX_SIZE];
};
