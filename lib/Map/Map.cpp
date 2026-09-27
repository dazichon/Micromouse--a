#include "Map.h"
#include <Preferences.h>
#include <string.h>

static Preferences prefs;
static const uint32_t MAP_VERSION = 0x20260924;   // đổi khi đổi định dạng map lưu flash

void Map::configure(int cells, int goalRow, int goalCol, int goalSize) {
  _cells = cells;
  _size = 2 * cells + 1;
  _goalRow = goalRow; _goalCol = goalCol; _goalSize = goalSize;
}

void Map::reset() {
  for (int i = 0; i < _size; i++)
    for (int j = 0; j < _size; j++)
      _maze[i][j] = (i % 2 == 0) ? ((j % 2 == 0) ? 1 : ((i == 0 || i == _size - 1) ? 3 : 5))
                                   : ((j % 2 == 0) ? ((j == 0 || j == _size - 1) ? 2 : 4) : 0);
}

bool Map::isGoalCell(int cx, int cy) const {
  return cx >= _goalRow && cx < _goalRow + _goalSize &&
         cy >= _goalCol && cy < _goalCol + _goalSize;
}

bool Map::canMove(int x, int y, int nx, int ny, bool unknownIsWall) const {
  if (nx < 0 || ny < 0 || nx >= _size || ny >= _size) return false;
  int wx = (x + nx) / 2, wy = (y + ny) / 2;
  int w = _maze[wx][wy];
  if (w == 2 || w == 3) return false;
  if (unknownIsWall && (w == 4 || w == 5)) return false;
  if (_maze[nx][ny] != 0) return false;
  return true;
}

void Map::wallIndex(int cx, int cy, Heading side, int& wx, int& wy) const {
  int mx = cx * 2 - 1, my = cy * 2 - 1;
  wx = mx; wy = my;
  if (side == NORTH) wx = mx - 1;
  else if (side == SOUTH) wx = mx + 1;
  else if (side == EAST)  wy = my + 1;
  else                    wy = my - 1;
}

void Map::setWall(int cx, int cy, Heading side, bool present) {
  int wx, wy;
  wallIndex(cx, cy, side, wx, wy);
  if (wx <= 0 || wy <= 0 || wx >= _size - 1 || wy >= _size - 1) return;   // tường bao ngoài: luôn giữ
  _wallSeen[wx][wy] = true;
  bool horizontal = (wx % 2 == 0);   // hàng chẵn = tường ngang (3), cột chẵn = tường dọc (2)
  _maze[wx][wy] = present ? (horizontal ? 3 : 2) : 0;
}

bool Map::wallKnown(int cx, int cy, Heading side) const {
  int wx, wy;
  wallIndex(cx, cy, side, wx, wy);
  int w = _maze[wx][wy];
  return w == 1 || w == 2 || w == 3;
}

bool Map::forgetUnseenWalls() {
  bool changed = false;
  for (int i = 1; i < _size - 1; i++)
    for (int j = 1; j < _size - 1; j++) {
      if (_wallSeen[i][j]) continue;
      if (_maze[i][j] == 2) { _maze[i][j] = 4; changed = true; }
      else if (_maze[i][j] == 3) { _maze[i][j] = 5; changed = true; }
    }
  return changed;
}

bool Map::saveToFlash() {
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

bool Map::loadFromFlash() {
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

void Map::clearFlash() {
  prefs.begin("micromouse", false);
  prefs.clear();
  prefs.end();
}

String Map::toString() const {
  String out = "";
  out.reserve(_size * _size + 1);
  for (int i = 0; i < _size; i++)
    for (int j = 0; j < _size; j++)
      out += String(_maze[i][j]);
  return out;
}
