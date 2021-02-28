#include "point3.hpp"
#include "triangle.hpp"
#include <cmath>
#include <functional>
#include <limits>
#include <string>

#include <iostream>

using std::acos;
using std::atan2;
using std::sqrt;

GPoint3::GPoint3(long double x, long double y, long double z, int res, int row,
                 int col, ico::map_orientation mo, ico::rotation_method rm,
                 bool is_pc, int tri_num)
    : Point3::Point3(x, y, z, is_pc, tri_num), res(res), row(row), col(col),
      mo(mo), rm(rm) {}

GPoint3::GPoint3()
    : Point3::Point3(std::numeric_limits<long double>::infinity(),
                     std::numeric_limits<long double>::infinity(),
                     std::numeric_limits<long double>::infinity(), false, -1),
      res(-1), row(-1), col(-1) {}

bool GPoint3::is_phex_center(const int res, const int row, const int col) {
  const int nd = hexmapf::num_divisions(res);
  if (row == 0 || row == nd * 3) {
    return true;
  }
  int new_row = -1;
  int new_col = -1;
  if (row > nd * 2) {
    // if in bottom tri
    new_row = row - nd * 2;
    new_col = col % (nd - new_row);
    return (new_col - (new_row % 3)) % 3 == 0;
  } else if (row >= nd) {
    // if in center tri
    new_row = row;
    new_col = col % row;
    return (new_col - (new_row % 3)) % 3 == 0;
  } else {
    // if in top tri
    new_row = row;
    new_col = col % row;
    return (new_col - (3 - (new_row % 3))) % 3 == 0;
  }
}

Quaternion::Quaternion(long double x, long double y, long double z,
                       long double w)
    : Point3(x, y, z), w(w) {}

void Quaternion::unit() {
  long double mag = this->mag();
  x = x / mag;
  y = y / mag;
  z = z / mag;
  w = w / mag;
}

long double Quaternion::mag() { return sqrt(x * x + y * y + z * z + w * w); }

void Quaternion::multiply(const Quaternion &q) {
  Point3 a(x, y, z);
  Point3 b(q.x, q.y, q.z);
  Point3 v1 = a;
  v1.cross(b);
  Point3 v2 = b;
  v2.mult_by(w);
  Point3 v3 = a;
  v3.mult_by(q.w);
  Point3 v = v1;
  v.add(v2);
  v.add(v3);
  x = v.x;
  y = v.y;
  z = v.z;
  w = w * q.w - a.dot(b);
}

Point3::Point3(long double x, long double y, long double z, bool is_pc,
               int tri_num)
    : x(x), y(y), z(z), tri_num(tri_num), is_pc(is_pc) {}

Point3::~Point3() {}

long double Point3::get_radius() {
  return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
}

/**
 * from here: [https://gis.stackexchange.com/a/120685]
 *
 * (longitude)
 * λ = [arctan(y / x) * 360 / 2π]
 *   = [atan2(y, x) * 360 / 2π] to account for quadrant or div-by-zero issues
 *
 * (latitude)
 * φ = 90 - arccos(z / r) * 360 / 2π
 *   = arcsin(z / r) * 360 / 2π
 **/

long double Point3::get_lat() {
  const long double r = this->get_radius();
  if (r == 0) {
    return 0;
  }
  return 90 - acos(this->z / r) * (180 / constants::PI);
}

long double Point3::get_lon() {
  return atan2(this->y, this->x) * (180 / constants::PI);
}

long double Point3::angle_between(const Point3 &p) const {
  long double inner = this->dot(p) / (this->mag() * p.mag());
  if (inner > 1.0) {
    inner = 1.0;
  } else if (inner < -1.0) {
    inner = -1.0;
  }
  return std::acos(inner);
}
/**
 * @param around vec to rotate around -> vec is from origin to point
 * @param rad rads to rotate
 * @note modifies obj **/
void Point3::rotate(const Point3 &around, const long double &rad) {
  Quaternion axis(around.x, around.y, around.z, 0.0);
  axis.unit();
  const long double num = sin(rad / 2.0);
  // multiply vec component by sin(rad / 2.0), and w component is cos(rad / 2.0)
  Quaternion q(axis.x * num, axis.y * num, axis.z * num, cos(rad / 2.0));
  const Quaternion p(x, y, z, 0);
  const Quaternion qInv(axis.x * -num, axis.y * -num, axis.z * -num,
                        cos(rad / 2.0));
  q.multiply(p);
  q.multiply(p);
  x = q.x;
  y = q.y;
  z = q.z;
}

long double Point3::mag() const {
  const long double sq =
      std::sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
  return sq;
}

void Point3::add(const Point3 &p) {
  x += p.x;
  y += p.y;
  z += p.z;
}

void Point3::subtract(const Point3 &p) {
  x -= p.x;
  y -= p.y;
  z -= p.z;
}

long double Point3::dot(const Point3 &p) const {
  return x * p.x + y * p.y + z * p.z;
}

void Point3::cross(const Point3 &p) {
  const long double t_x = this->x;
  const long double t_y = this->y;
  const long double t_z = this->z;
  this->x = t_y * p.z - p.y * t_z;
  this->y = t_z * p.x - p.z * t_x;
  this->z = t_x * p.y - p.x * t_y;
}

void Point3::unit() {
  const long double mag = this->mag();
  if (mag == 0) {
    x = 0;
    y = 0;
    z = 0;
  } else {
    x /= mag;
    y /= mag;
    z /= mag;
  }
}

void Point3::mult_by(const long double num) {
  x *= num;
  y *= num;
  z *= num;
}

void Point3::div_by(const long double num) {
  x /= num;
  y /= num;
  z /= num;
}

long double Point3::distance(const Point3 &p) const {
  return std::sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) +
                   (z - p.z) * (z - p.z));
}

void Point3::spheriphy() {
  this->unit();
  this->mult_by(constants::radius);
}

std::vector<Point3> Point3::spherify1D(std::vector<Point3> points) {
  for (Point3 &p : points) {
    p.spheriphy();
  }
  return points;
}

bool Point3::on_opposite_side(const Point3 &p) const {
  return std::signbit(x) != std::signbit(p.x) ||
         std::signbit(y) != std::signbit(p.y) ||
         std::signbit(z) != std::signbit(p.z);
  // // FIXME: TODO: returns false for all, still, small optimization
  // return false;
}

bool Point3::is_valid() const {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

GPoint3 Point3::closest_point(std::vector<GPoint3> &points) const {
  std::unique_ptr<GPoint3> closest;
  long double smallest_distance = std::numeric_limits<long double>::infinity();
  long double dist = 0;
  for (GPoint3 &p : points) {
    dist = this->distance(p);
    if (dist < smallest_distance) {
      smallest_distance = dist;
      closest.reset(&p);
      // *closest = p;
    }
  }
  return *closest;
}

GPoint3
Point3::closest_point_2d(std::vector<std::vector<GPoint3>> &points_2d) const {
  GPoint3 *closest = new GPoint3();
  long double smallest_distance =
      constants::radius * 20; // std::numeric_limits<long double>::infinity();
  long double dist = 0;

  for (std::vector<GPoint3> &points : points_2d) {
    for (GPoint3 &p : points) {
      if (p.is_pc) {
        dist = this->distance(p);
        if (dist < smallest_distance) {
          smallest_distance = dist;
          *closest = p;
        }
      }
    }
  }

  // bit hacky, but for deleting allocated mem before leaving function
  GPoint3 p_copy = *closest;
  delete closest;
  return p_copy;
}

void Point3::rotate_around_y(long double rads) {
  const int o_x = this->x;
  this->x = this->x * cos(rads) + this->z * sin(rads);
  this->z = -o_x * sin(rads) + this->z * cos(rads);
}

/**
 * GNOMONIC POINT GENERATION
 **/

std::vector<Point3> Point3::all_side_points_gnomonic(const Point3 &above,
                                                     const Point3 &below,
                                                     int res) {
  const int nd = hexmapf::num_divisions(res);
  std::vector<Point3> point_arr;
  // vec from [above] to [below]
  Point3 AB = below;
  AB.subtract(above);
  // unit AB
  Point3 uAB = AB;
  uAB.unit();
  // vec between [above] and [below]
  Point3 C = uAB;
  C.mult_by(above.distance(below) / 2.0);
  C.add(above);
  // spherify
  C.spheriphy(); // replaces below:
  // C.unit();
  // C.mult_by(r);
  const long double magC = C.mag();
  const long double alpha = above.angle_between(C);
  const long double magH = magC / std::cos(alpha);
  Point3 A = above;
  A.unit();
  A.mult_by(magH);
  A.is_pc = true;
  Point3 B = below;
  B.unit();
  B.mult_by(magH);
  B.is_pc = true;
  // distance between A and B
  const long double dist = A.distance(B);
  const long double dist_unit = dist / nd;
  // add first point
  point_arr.push_back(A);
  // add points between, loop vars set here so don't create more each iteration
  // TODO: local vars are cached, value at pointer might actually take longer to
  // get -> test speed of this vs creating new instances/vars for each iteration
  long double d;
  // std::unique_ptr<Point3> rotated;
  for (int i = 1; i < nd; i++) {
    d = dist_unit * i;
    // set pointer's value
    // rotated.reset(&uAB);
    Point3 rotated = uAB;
    rotated.mult_by(d);
    rotated.add(A);
    // spheriphy point since gnomonic generates points between tri sides (along
    // line, not circle)
    // rotated.spheriphy();
    point_arr.push_back(rotated);
  }
  // add last point
  point_arr.push_back(B);
  return point_arr;
}

Point3::lazy_side_points_result
Point3::lazy_side_points_gnomonic(const Triangle &tri, const int center,
                                  const int res, const int lazy_range,
                                  int lower, int upper) {

  const int nd = hexmapf::num_divisions(res);
  if (lower == -1) {
    lower = center - lazy_range;
  }
  if (lower < 0) {
    lower = 0;
  }
  if (upper == -1) {
    upper = center + lazy_range;
  }
  if (upper > nd) {
    upper = nd;
  }

  const std::function generate_side_points =
      [&nd, &lower, &upper](const Point3 &top,
                            const Point3 &bot) -> std::vector<Point3> {
    std::vector<Point3> point_arr;
    Point3 AB = bot;
    AB.subtract(top);
    Point3 uAB = AB;
    uAB.unit();
    // vec between above and below
    Point3 C = uAB;
    C.mult_by(top.distance(bot) / 2.0);
    C.add(top);
    C.spheriphy();

    const long double magC = C.mag();
    const long double alpha = top.angle_between(C);
    const long double magH = magC / std::cos(alpha);
    Point3 A = top;
    A.unit();
    A.mult_by(magH);
    A.is_pc = true;
    Point3 B = bot;
    B.unit();
    B.mult_by(magH);
    B.is_pc = true;
    const long double dist = A.distance(B);
    const long double dist_unit = dist / nd;

    long double d;
    for (int c = lower; c <= upper; c++) {
      d = dist_unit * c;
      Point3 rotated = uAB;
      rotated.mult_by(d);
      rotated.add(A);
      // spheriphy point since gnomonic generates points between tri sides
      // (along line, not circle)
      // rotated.spheriphy();
      point_arr.push_back(rotated);
    }

    return point_arr;
  };

  const Point3 topL = tri.direction == tri::pointing::UP ? tri.A : tri.B;
  const Point3 botL = tri.direction == tri::pointing::UP ? tri.C : tri.A;
  const Point3 topR = tri.direction == tri::pointing::UP ? tri.A : tri.C;
  const Point3 botR = tri.direction == tri::pointing::UP ? tri.B : tri.A;
  const std::vector<Point3> pointsL = generate_side_points(topL, botL);
  const std::vector<Point3> pointsR = generate_side_points(topR, botR);

  return {.pointsL = pointsL, .pointsR = pointsR, .lower_indx = lower};
}

std::vector<Point3> Point3::all_row_points_gnomonic(const Point3 &left,
                                                    const Point3 &right,
                                                    int num_divisions) {
  if (num_divisions <= 0) {
    return std::vector<Point3>({left});
  }
  std::vector<Point3> points;
  const long double dist_unit = left.distance(right) / num_divisions;

  Point3 uLR = right;
  uLR.subtract(left);
  uLR.unit();
  // add first point
  points.push_back(left);
  // add points between
  long double d;
  // std::unique_ptr<Point3> rotated;
  for (int c = 1; c < num_divisions; c++) {
    d = dist_unit * c;
    // rotated.reset(&uLR);
    Point3 rotated = uLR;
    rotated.mult_by(d);
    rotated.add(left);
    // rotated.spheriphy();
    points.push_back(rotated);
  }
  // add last point
  points.push_back(right);
  return points;
}

Point3::lazy_row_points_result
Point3::lazy_row_points_gnomonic(const int center, const Point3 &left,
                                 const Point3 &right, int num_divisions,
                                 const int lazy_range, int lower, int upper) {

  if (num_divisions <= 0) {
    return {.row_points = std::vector<Point3>({left}), .lower_indx = 0};
  }
  if (lower == -1) {
    lower = center - lazy_range;
  }
  if (lower < 0) {
    lower = 0;
  }

  if (upper == -1) {
    upper = center + lazy_range;
  }
  if (upper > num_divisions) {
    upper = num_divisions;
  }

  std::vector<Point3> point_arr;
  const long double dist = left.distance(right);
  const long double dist_unit = dist / num_divisions;
  Point3 uLR = right;
  uLR.subtract(left);
  uLR.unit();
  // add points in lazy range
  long double d;
  // std::unique_ptr<Point3> rotated;
  for (int c = lower; c <= upper; c++) {
    d = dist_unit * c;
    // rotated.reset(&uLR);
    Point3 rotated = uLR;
    rotated.mult_by(d);
    rotated.add(left);
    // rotated.spheriphy();
    point_arr.push_back(rotated);
  }

  return {.row_points = point_arr, .lower_indx = lower};
}

/**
 * QUATERNION POINT GENERATION
 **/

std::vector<Point3> Point3::all_side_points_quaternion(const Point3 &above,
                                                       const Point3 &below,
                                                       int res) {
  const long double nd = hexmapf::num_divisions(res);
  std::vector<Point3> point_arr;
  const long double angle = above.angle_between(below);
  const long double angle_unit = angle / nd;
  Point3 axis = above;
  axis.cross(below);
  // add first point
  point_arr.push_back(above);
  // add points between
  long double ang;
  // std::unique_ptr<Point3> rotated;
  for (int c = 1; c < nd; c++) {
    ang = angle_unit * c;
    // Point3 above_copy = above;
    Point3 rotated = above;
    rotated.rotate(axis, ang);
    point_arr.push_back(rotated);
  }
  // add last point
  point_arr.push_back(below);
  return point_arr;
};

Point3::lazy_side_points_result
Point3::lazy_side_points_quaternion(const Triangle &tri, const int center,
                                    const int res, const int lazy_range,
                                    int lower, int upper) {
  const int nd = hexmapf::num_divisions(res);
  if (lower == -1) {
    lower = center - lazy_range;
  }
  if (lower < 0) {
    lower = 0;
  }

  if (upper == -1) {
    upper = center + lazy_range;
  }
  if (upper > nd) {
    upper = nd;
  }

  // generate list of points
  const std::function generate_side_points =
      [&nd, &lower, &upper](const Point3 top,
                            const Point3 bot) -> std::vector<Point3> {
    std::vector<Point3> arr;
    const long double angle = top.angle_between(bot);
    const long double angle_unit = angle / nd;
    Point3 axis = top;
    axis.cross(bot);
    long double ang;
    // std::unique_ptr<Point3> rotated;
    for (int c = lower; c <= upper; c++) {
      ang = angle_unit * c;
      Point3 rotated = top;
      rotated.rotate(axis, ang);
      arr.push_back(rotated);
    }
    return arr;
  };
  // setup points
  const Point3 topL = tri.direction == tri::pointing::UP ? tri.A : tri.B;
  const Point3 botL = tri.direction == tri::pointing::UP ? tri.C : tri.A;
  const Point3 topR = tri.direction == tri::pointing::UP ? tri.A : tri.C;
  const Point3 botR = tri.direction == tri::pointing::UP ? tri.B : tri.A;
  // generate side points
  const std::vector<Point3> pointsL = generate_side_points(topL, botL);
  const std::vector<Point3> pointsR = generate_side_points(topR, botR);

  return {.pointsL = pointsL, .pointsR = pointsR, .lower_indx = lower};
}

std::vector<Point3> Point3::all_row_points_quaternion(const Point3 &left,
                                                      const Point3 &right,
                                                      int num_divisions) {
  if (num_divisions <= 0) {
    return std::vector<Point3>({left});
  }
  std::vector<Point3> point_arr;
  const long double angle = left.angle_between(right);
  const long double angle_unit = angle / num_divisions;
  Point3 axis = left;
  axis.cross(right);
  // add first point
  point_arr.push_back(left);
  // add points between
  long double ang;
  Point3 *rotated = nullptr;
  for (int c = 1; c < num_divisions; c++) {
    ang = angle_unit * c;
    *rotated = left;
    rotated->rotate(axis, ang);
    point_arr.push_back(*rotated);
  }
  delete rotated;
  // add last point
  point_arr.push_back(right);
  return point_arr;
}

Point3::lazy_row_points_result
Point3::lazy_row_points_quaternion(const int center, const Point3 &left,
                                   const Point3 &right, int num_divisions,
                                   const int lazy_range, int lower, int upper) {
  if (num_divisions <= 0) {
    return {.row_points = std::vector<Point3>({left}), .lower_indx = 0};
  }

  if (lower == -1) {
    lower = center - lazy_range;
  }
  if (lower < 0) {
    lower = 0;
  }

  if (upper == -1) {
    upper = center + lazy_range;
  }
  if (upper > num_divisions || upper < 0) {
    upper = num_divisions;
  }

  std::vector<Point3> point_arr;
  const long double angle = left.angle_between(right);
  const long double angle_unit = angle / num_divisions;
  Point3 axis = left;
  axis.cross(right);
  // add points in lazy range
  long double ang;
  // std::unique_ptr<Point3> rotated;
  for (int c = lower; c <= upper; c++) {
    ang = angle_unit * c;
    Point3 rotated = left;
    rotated.rotate(left, ang);
    point_arr.push_back(rotated);
  }
  return {.row_points = point_arr, .lower_indx = lower};
};