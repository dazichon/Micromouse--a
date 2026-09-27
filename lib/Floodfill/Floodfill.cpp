#include "Floodfill.h"
#include <string.h>

// ====================== nội bộ: hàng đợi cho BFS flood fill ======================
namespace {
  struct Node { int x, y; };
  const int Q_CAP = Map::MAX_SIZE * Map::MAX_SIZE;
  Node q[Q_CAP];
  int qFront = 0, qBack = 0;
  void qReset() { qFront = qBack = 0; }
  bool qEmpty() { return qFront == qBack; }
  void qPush(int x, int y) { q[qBack] = {x, y}; qBack = (qBack + 1) % Q_CAP; }
  Node qPop() { Node n = q[qFront]; qFront = (qFront + 1) % Q_CAP; return n; }

  const int dxArr[4] = {-2, 0, 2, 0};
  const int dyArr[4] = {0, 2, 0, -2};

  void appendCmd(char* path, int& len, int cap, char c) {
    if (len < cap - 1) { path[len++] = c; path[len] = '\0'; }
  }
}

void Floodfill::reset() {
  memset(_dist, -1, sizeof(_dist));
}

void Floodfill::compute(const Map& map, bool unknownIsWall) {
  memset(_dist, -1, sizeof(_dist));
  qReset();
  int size = map.size();
  for (int r = map.goalRow(); r < map.goalRow() + map.goalSize(); r++)
    for (int c = map.goalCol(); c < map.goalCol() + map.goalSize(); c++) {
      int mx = r * 2 - 1, my = c * 2 - 1;
      _dist[mx][my] = 0;
      qPush(mx, my);
    }

  while (!qEmpty()) {
    Node cur = qPop();
    for (int d = 0; d < 4; d++) {
      int nx = cur.x + dxArr[d], ny = cur.y + dyArr[d];
      if (nx < 0 || nx >= size || ny < 0 || ny >= size) continue;
      if (!map.canMove(cur.x, cur.y, nx, ny, unknownIsWall)) continue;
      if (_dist[nx][ny] == -1) {
        _dist[nx][ny] = _dist[cur.x][cur.y] + 1;
        qPush(nx, ny);
      }
    }
  }
}

Heading Floodfill::chooseBestDirection(const Map& map, int cx, int cy, Heading h) const {
  int mx = cx * 2 - 1, my = cy * 2 - 1;
  int size = map.size();
  int best = 9999;
  Heading bestDir = h;

  for (int d = 0; d < 4; d++) {
    int nx = mx + dxArr[d], ny = my + dyArr[d];
    if (nx < 0 || ny < 0 || nx >= size || ny >= size) continue;
    if (!map.canMove(mx, my, nx, ny)) continue;
    if (_dist[nx][ny] < 0) continue;

    if (_dist[nx][ny] < best) { best = _dist[nx][ny]; bestDir = (Heading)d; }
    else if (_dist[nx][ny] == best && (Heading)d == h) bestDir = (Heading)d;
  }
  return bestDir;
}

Heading Floodfill::chooseBestDirectionSolved(const Map& map, int mx, int my, Heading h) const {
  int size = map.size();
  int best = 9999;
  Heading bestDir = h;
  for (int d = 0; d < 4; d++) {
    int nx = mx + dxArr[d], ny = my + dyArr[d];
    if (nx < 0 || ny < 0 || nx >= size || ny >= size) continue;
    if (!map.canMove(mx, my, nx, ny, true)) continue;
    if (_dist[nx][ny] < 0) continue;
    if (_dist[nx][ny] < best) { best = _dist[nx][ny]; bestDir = (Heading)d; }
    else if (_dist[nx][ny] == best && (Heading)d == h) bestDir = (Heading)d;
  }
  return bestDir;
}

bool Floodfill::buildPath(const Map& map, int startCx, int startCy, Heading startH, char* outPath, int outCap) {
  int len = 0;
  if (outCap > 0) outPath[0] = '\0';

  compute(map, true);   // fast run chỉ đi qua tường đã xác nhận là trống
  int startMx = startCx * 2 - 1, startMy = startCy * 2 - 1;
  if (_dist[startMx][startMy] < 0) return false;

  int size = map.size();
  int mx = startMx, my = startMy;
  Heading h = startH;
  int guard = 0;
  int guardMax = map.cells() * map.cells();

  while (_dist[mx][my] != 0 && guard < guardMax) {
    Heading nextH = chooseBestDirectionSolved(map, mx, my, h);
    int nx = mx + dxArr[nextH], ny = my + dyArr[nextH];
    if (nx < 0 || ny < 0 || nx >= size || ny >= size) return false;
    if (!map.canMove(mx, my, nx, ny, true)) return false;
    if (_dist[nx][ny] < 0) return false;

    int diff = (nextH - h + 4) % 4;
    if (diff == 1) appendCmd(outPath, len, outCap, 'R');
    else if (diff == 3) appendCmd(outPath, len, outCap, 'L');
    else if (diff == 2) { appendCmd(outPath, len, outCap, 'R'); appendCmd(outPath, len, outCap, 'R'); }
    appendCmd(outPath, len, outCap, 'F');

    mx = nx; my = ny; h = nextH;
    guard++;
  }
  return _dist[mx][my] == 0;
}
