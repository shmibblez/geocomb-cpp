#include "point3.hpp"
#include "constants.hpp"
#include "triangle.hpp"
#include <cmath>
#include <limits>

GPoint3::GPoint3(double x, double y, double z, int res, int row, int col,
                 bool is_vert = false, int tri_num = -1)
    : Point3::Point3(x, y, z, is_vert, tri_num), res(res), row(row), col(col) {}

Quaternion::Quaternion(double x, double y, double z, double w)
    : Point3(x, y, z), w(w) {}

void Quaternion::unit() {
  double mag = this->mag();
  x = x / mag;
  y = y / mag;
  z = z / mag;
  w = w / mag;
}

double Quaternion::mag() { return sqrt(x * x + y * y + z * z + w * w); }

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

Point3::Point3(int x, int y, int z, bool is_vert = false, int tri_num = -1)
    : x(x), y(y), z(z), is_vert(is_vert), tri_num(tri_num) {}

double Point3::angle_between(const Point3 &p) const {
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
  const double num = sin(rad / 2.0);
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

double Point3::mag() const { return std::sqrt(x * x + y * y + z * z); }

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
/**
 * dot product **/
double Point3::dot(const Point3 &p) const {
  return x * p.x + y * p.y + z * p.z;
}

void Point3::cross(const Point3 &p) {
  x = y * p.z - p.y * z;
  y = z * p.x - p.z * x;
  z = x * p.y - p.x * y;
}

void Point3::unit() {
  const double mag = this->mag();
  x /= mag;
  y /= mag;
  z /= mag;
}

void Point3::mult_by(const double num) {
  x *= num;
  y *= num;
  z *= num;
}

void Point3::div_by(const double num) {
  x /= num;
  y /= num;
  z /= num;
}

double Point3::distance(const Point3 &p) const {
  return std::sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) +
                   (z - p.z) * (z - p.z));
}

void Point3::spheriphy() {
  this->unit();
  this->mult_by(constants::radius);
}

void Point3::spherify1D(std::vector<Point3> &points) {
  for (Point3 &p : points) {
    p.spheriphy();
  }
}

bool Point3::on_opposite_side(const Point3 &p) const {
  return std::signbit(x) != std::signbit(p.x) ||
         std::signbit(y) != std::signbit(p.y) ||
         std::signbit(z) != std::signbit(p.z);
}

bool Point3::is_valid() const {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

Point3 Point3::closest_point(std::vector<Point3> &points) const {
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

/**
 * GNOMONIC POINT GENERATION
 **/

std::vector<Point3> Point3::all_side_points_gnomonic(const Point3 &above,
                                                     const Point3 &below,
                                                     int res) {
  const int nd = hexmapf::num_divisions(res);
  const double r = constants::radius;
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
  C.spheriphy();
  const double magC = C.mag();
  const double alpha = above.angle_between(C);
  const double magH = magC / std::cos(alpha);
  Point3 A = above;
  A.unit();
  A.mult_by(magH);
  A.is_vert = true;
  Point3 B = below;
  B.unit();
  B.mult_by(magH);
  B.is_vert = true;
  // distance between A and B
  const double dist = A.distance(B);
  const double dist_unit = dist / nd;
  // add first point
  point_arr.push_back(A);
  // add points between, loop vars set here so don't create more each iteration
  // TODO: local vars are cached, value at pointer might actually take longer to
  // get -> test speed of this vs creating new instances/vars for each iteration
  double d;
  Point3 *rotated = nullptr;
  for (int i = 1; i < nd; i++) {
    d = dist_unit * i;
    // set pointer's value
    *rotated = uAB;
    rotated->mult_by(d);
    rotated->add(A);
    // spheriphy point since gnomonic generates points between tri sides (along
    // line, not circle)
    rotated->spheriphy();
    point_arr.push_back(*rotated);
  }
  delete rotated;
  // add last point
  point_arr.push_back(B);
  return point_arr;
}

Point3::lazy_side_points_result
Point3::lazy_side_points_gnomonic(const Triangle &tri, const int center,
                                  const int res,
                                  const int lazy_range = constants::lazy_range,
                                  int lower = -1, int upper = -1) {
  const int nd = hexmapf::num_divisions(res);
  if (lower == -1) {
    lower = center - lazy_range;
  }
  if (lower < 0)
    lower = 0;
  if (upper == -1) {
    upper = center + lazy_range;
  }
  if (upper > nd)
    upper = nd;

  const int r = constants::radius;

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

    const double magC = C.mag();
    const double alpha = top.angle_between(C);
    const double magH = magC / std::cos(alpha);
    Point3 A = top;
    A.unit();
    A.mult_by(magH);
    A.is_vert = true;
    Point3 B = bot;
    B.unit();
    B.mult_by(magH);
    B.is_vert = true;
    const double dist = A.distance(B);
    const double dist_unit = dist / nd;

    double d;
    Point3 *rotated = nullptr;
    for (int c = lower; c <= upper; c++) {
      d = dist_unit * c;
      *rotated = uAB;
      rotated->mult_by(d);
      rotated->add(A);
      // spheriphy point since gnomonic generates points between tri sides
      // (along line, not circle)
      rotated->spheriphy();
      point_arr.push_back(*rotated);
    }
    delete rotated;
    return point_arr;
  };

  const Point3 topL = tri.direction == pointing::UP ? tri.A : tri.B;
  const Point3 botL = tri.direction == pointing::UP ? tri.C : tri.A;
  const Point3 topR = tri.direction == pointing::UP ? tri.A : tri.C;
  const Point3 botR = tri.direction == pointing::UP ? tri.B : tri.A;
  const std::vector<Point3> pointsL = generate_side_points(topL, botL);
  const std::vector<Point3> pointsR = generate_side_points(topR, botR);

  const Point3::lazy_side_points_result result(pointsL, pointsR, lower);
  return result;
}

std::vector<Point3> Point3::all_row_points_gnomonic(const Point3 &left,
                                                    const Point3 &right,
                                                    int num_divisions) {
  if (num_divisions <= 0) {
    return std::vector<Point3>({left});
  }
  std::vector<Point3> points;
  const double dist_unit = left.distance(right) / num_divisions;

  Point3 uLR = right;
  uLR.subtract(left);
  uLR.unit();
  // add first point
  points.push_back(left);
  // add points between
  double d;
  Point3 *rotated = nullptr;
  for (int c = 1; c < num_divisions; c++) {
    d = dist_unit * c;
    *rotated = uLR;
    rotated->mult_by(d);
    rotated->add(left);
    rotated->spheriphy();
    points.push_back(*rotated);
  }
  delete rotated;
  // add last point
  points.push_back(right);
  return points;
}

Point3::lazy_row_points_result
Point3::lazy_row_points_gnomonic(const int center, const Point3 &left,
                                 const Point3 &right, int num_divisions,
                                 const int lazy_range = constants::lazy_range,
                                 int lower = -1, int upper = -1) {

  if (num_divisions <= 0) {
    return Point3::lazy_row_points_result(std::vector<Point3>({left}), 0);
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
  const double dist = left.distance(right);
  const double dist_unit = dist / num_divisions;
  Point3 uLR = right;
  uLR.subtract(left);
  uLR.unit();
  // add points in lazy range
  double d;
  Point3 *rotated = nullptr;
  for (int c = lower; c <= upper; c++) {
    d = dist_unit * c;
    *rotated = uLR;
    rotated->mult_by(d);
    rotated->add(left);
    rotated->spheriphy();
    point_arr.push_back(*rotated);
  }
  delete rotated;
  return Point3::lazy_row_points_result(point_arr, lower);
}

/**
 * QUATERNION POINT GENERATION
 **/

std::vector<Point3> Point3::all_side_points_quaternion(const Point3 &above,
                                                       const Point3 &below,
                                                       int res) {
  const double nd = hexmapf::num_divisions(res);
  std::vector<Point3> point_arr;
  const double angle = above.angle_between(below);
  const double angle_unit = angle / nd;
  Point3 axis = above;
  axis.cross(below);
  // add first point
  point_arr.push_back(above);
  // add points between
  double ang;
  Point3 *rotated = nullptr;
  for (int c = 1; c < nd; c++) {
    ang = angle_unit * c;
    *rotated = above;
    rotated->rotate(axis, ang);
    point_arr.push_back(*rotated);
  }
  delete rotated;
  // add last point
  point_arr.push_back(below);
  return point_arr;
};

Point3::lazy_side_points_result Point3::lazy_side_points_quaternion(
    const Triangle &tri, const int center, const int res,
    const int lazy_range = constants::lazy_range, int lower = -1,
    int upper = -1) {
  const int nd = hexmapf::num_divisions(res);
  if (lower == -1) {
    lower = 0;
  }
  if (lower < 0) {
    lower = 0;
  }

  if (upper == -1) {
    upper = nd;
  }
  if (upper > nd) {
    upper = nd;
  }

  // generate list of points
  const std::function generate_side_points =
      [&nd, &lower, &upper](const Point3 top,
                            const Point3 bot) -> std::vector<Point3> {
    std::vector<Point3> arr;
    const double angle = top.angle_between(bot);
    const double angle_unit = angle / nd;
    Point3 axis = top;
    axis.cross(bot);
    double ang;
    Point3 *rotated = nullptr;
    for (int c = lower; c <= upper; c++) {
      ang = angle_unit * c;
      *rotated = top;
      rotated->rotate(axis, ang);
      arr.push_back(*rotated);
    }
    delete rotated;
    return arr;
  };
  // setup points
  const Point3 topL = tri.direction == pointing::UP ? tri.A : tri.B;
  const Point3 botL = tri.direction == pointing::UP ? tri.C : tri.A;
  const Point3 topR = tri.direction == pointing::UP ? tri.A : tri.C;
  const Point3 botR = tri.direction == pointing::UP ? tri.B : tri.A;
  // generate side points
  const std::vector<Point3> pointsL = generate_side_points(topL, botL);
  const std::vector<Point3> pointsR = generate_side_points(topR, botR);

  return Point3::lazy_side_points_result(pointsL, pointsR, lower);
}

std::vector<Point3> Point3::all_row_points_quaternion(const Point3 &left,
                                                      const Point3 &right,
                                                      int num_divisions) {
  if (num_divisions <= 0) {
    return std::vector<Point3>({left});
  }
  std::vector<Point3> point_arr;
  const double angle = left.angle_between(right);
  const double angle_unit = angle / num_divisions;
  Point3 axis = left;
  axis.cross(right);
  // add first point
  point_arr.push_back(left);
  // add points between
  double ang;
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
                                   const int lazy_range = constants::lazy_range,
                                   int lower = -1, int upper = -1) {
  if (num_divisions <= 0) {
    return Point3::lazy_row_points_result(std::vector<Point3>({left}), 0);
  }

  if (lower == -1) {
    lower = 0;
  }
  if (lower < 0) {
    lower = 0;
  }

  if (upper == -1) {
    upper = num_divisions;
  }
  if (upper > num_divisions || upper < 0) {
    upper = num_divisions;
  }

  std::vector<Point3> point_arr;
  const double angle = left.angle_between(right);
  const double angle_unit = angle / num_divisions;
  Point3 axis = left;
  axis.cross(right);
  // add points in lazy range
  double ang;
  Point3 *rotated = nullptr;
  for (int c = lower; c <= upper; c++) {
    ang = angle_unit * c;
    *rotated = left;
    rotated->rotate(left, ang);
    point_arr.push_back(*rotated);
  }
  delete rotated;
  return Point3::lazy_row_points_result(point_arr, lower);
};