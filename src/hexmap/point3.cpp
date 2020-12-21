#include "point3.hpp"

#include <cmath>

Point3::Point3(int x, int y, int z, bool is_vert = false, int tri_num = -1)
    : x(x), y(y), z(z), is_vert(is_vert), tri_num(tri_num) {}

GPoint3::GPoint3(int x, int y, int z, int res, int row, int col,
                 bool is_vert = false, int tri_num = -1)
    : Point3::Point3(x, y, z, is_vert, tri_num), res(res), row(row), col(col) {}

Quaternion::Quaternion(int x, int y, int z, int w) : Point3(x, y, z), w(w) {}

// convert to unit quaternion
void Quaternion::unit() {
  double mag = this->mag();
  x = x / mag;
  y = y / mag;
  z = z / mag;
  w = w / mag;
}
// get magnitude
double Quaternion::mag() { return sqrt(x * x + y * y + z * z + w * w); }
// multiply with another quaternion
Quaternion multiply(Quaternion q) {
  // TODO, need vector stuff first
}