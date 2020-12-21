#include "point3.hpp"
#include <cmath>
#include <limits>

GPoint3::GPoint3(double x, double y, double z, int res, int row, int col,
                 bool is_vert = false, int tri_num = -1)
    : Point3::Point3(x, y, z, is_vert, tri_num), res(res), row(row), col(col) {}

Quaternion::Quaternion(double x, double y, double z, double w)
    : Point3(x, y, z), w(w) {}

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
void Quaternion::multiply(const Quaternion &q) {
  double aw = w;
  Point3 a(x, y, z);
  Point3 b(q.x, q.y, q.z);
  Point3 v1 = a;
  v1.cross(b);
  Point3 v2 = b;
  b.mult_by(aw);
  Point3 v3 = a;
  a.mult_by(q.w);
  Point3 v = v1;
  v.add(v2);
  v.add(v3);
  x = v.x;
  y = v.y;
  z = b.z;
  w = aw * q.w - a.dot(b);
}

Point3::Point3(int x, int y, int z, bool is_vert = false, int tri_num = -1)
    : x(x), y(y), z(z), is_vert(is_vert), tri_num(tri_num) {}

// angle between vectors (origin, this) and (origin, p)
double Point3::angle_between(const Point3 &p) {
  double inner = this->dot(p);
  if (inner > 1) {
    inner = 1;
  } else if (inner < -1) {
    inner = -1;
  }
  return std::acos(inner);
}
/**
 * @param around vec to rotate around -> vec is from origin to point
 * @param rad rads to rotate
 * @note modifies obj **/
void Point3::rotate(const Point3 &around, const double &rad) {
  Quaternion axis(around.x, around.y, around.z, 0);
  axis.unit();
  const double num = sin(rad / 2);
  // multiply vec component by sin(rad / 2), and w component is cos(rad / 2)
  Quaternion q(axis.x * num, axis.y * num, axis.z * num, cos(rad / 2));
  const Quaternion p(x, y, z, 0);
  const Quaternion qInv(axis.x * -num, axis.y * -num, axis.z * -num,
                        cos(rad / 2));
  q.multiply(p);
  q.multiply(p);
  x = q.x;
  y = q.y;
  z = q.z;
}
/**
 * magnitude from origin to point */
double Point3::mag() { return std::sqrt(x * x + y * y + z * z); }
/**
 * add to other vector !-> modifies obj */
void Point3::add(const Point3 &p) {
  x += p.x;
  y += p.y;
  z += p.z;
}
/**
 * subtract other vector !-> modifies obj */
void Point3::subtract(const Point3 &p) {
  x -= p.x;
  y -= p.y;
  z -= p.z;
}
/**
 * dot product **/
double Point3::dot(const Point3 &p) { return x * p.x + y * p.y + z * p.z; }
/**
 * cross product
 * @note modifies obj **/
void Point3::cross(const Point3 &p) {
  x = y * p.z - p.y * z;
  y = z * p.x - p.z * x;
  z = x * p.y - p.x * y;
}
/**
 * convert into unit vec
 * @note modifies obj **/
void Point3::unit() {
  const double mag = this->mag();
  x /= mag;
  y /= mag;
  z /= mag;
}
/**
 * multiply by scalar !-> modifies obj */
void Point3::mult_by(const double num) {
  x *= num;
  y *= num;
  z *= num;
}
/**
 * multiply by scalar
 * @note modifies obj **/
void Point3::div_by(const double num) {
  x /= num;
  y /= num;
  z /= num;
}
/**
 * distance between vectors **/
double Point3::distance(const Point3 &p) {
  return std::sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) +
                   (z - p.z) * (z - p.z));
}
/**
 * check if points on opposite sides of globe
 * @note loosely determined -> points don't need to be 180 deg apart, if 90
 * deg apart will return true **/
bool Point3::on_opposite_side(const Point3 &p) {
  return std::signbit(x) != std::signbit(p.x) ||
         std::signbit(y) != std::signbit(p.y) ||
         std::signbit(z) != std::signbit(p.z);
}
/**
 * check if point is valid (all coords finite & valid numbers) **/
bool Point3::is_valid() {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}
/**
 * @returns copy of closest point in point arr **/
Point3 Point3::closest_point(std::vector<Point3> points) {
  Point3 *closest = nullptr;
  double smallest_distance = std::numeric_limits<double>::infinity();
  double dist = 0;
  for (const Point3 &p : points) {
    dist = this->distance(p);
    if (dist < smallest_distance) {
      smallest_distance = dist;
      *closest = p;
    }
  }
  return *closest;
}