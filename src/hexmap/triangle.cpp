#include "triangle.hpp"
#include "enums.hpp"

using std::round;

Triangle::Triangle(Point3 A, Point3 B, Point3 C, tri::pointing direction,
                   tri::position pos, int num, int toAB, int toBC, int toCA)
    : A(A), B(B), C(C), direction(direction), pos(pos), num(num), toAB(toAB),
      toBC(toBC), toCA(toCA){};

std::vector<std::vector<Point3>>
Triangle::all_points(int res, ico::map_orientation mo,
                     ico::rotation_method rm) const {
  // empty 2d vec
  std::vector<std::vector<Point3>> points;

  const int max_divisions = hexmapf::num_divisions(res);
  const Point3 left_above = this->direction == tri::pointing::UP ? A : B;
  const Point3 left_below = this->direction == tri::pointing::UP ? C : A;
  std::vector<Point3> left_points =
      rm == ico::rotation_method::gnomonic
          ? Point3::all_side_points_gnomonic(left_above, left_below, res)
          : Point3::all_side_points_quaternion(left_above, left_below, res);

  const Point3 right_above = this->direction == tri::pointing::UP ? A : C;
  const Point3 right_below = this->direction == tri::pointing::UP ? B : A;
  std::vector<Point3> right_points =
      rm == ico::rotation_method::gnomonic
          ? Point3::all_side_points_gnomonic(right_above, right_below, res)
          : Point3::all_side_points_quaternion(right_above, right_below, res);

  for (int x = 0; x <= max_divisions; x++) {
    int num_divs = this->direction == tri::pointing::UP ? x : max_divisions - x;
    std::vector<Point3> new_points =
        rm == ico::rotation_method::gnomonic
            ? Point3::all_row_points_gnomonic(left_points[x], right_points[x],
                                              num_divs)
            : Point3::all_row_points_quaternion(left_points[x], right_points[x],
                                                num_divs);
    points.push_back(new_points);
  }
  return points;
}

Triangle::lazy_points_around_result
Triangle::lazy_points_around(Point3 &p, int res,
                             ico::rotation_method rotation) const {

  const int nd = hexmapf::num_divisions(res);
  // calc side percents
  const CalcPercent::calc_percent_result percents =
      rotation == ico::rotation_method::gnomonic
          ? CalcPercent::gnomonic(*this, p)
          : CalcPercent::quaternion(*this, p);
  // calculate percent of intersect component from C to A
  const int estimated_vert_center = this->direction == tri::pointing::UP
                                        ? round(nd - percents.percent_CA * nd)
                                        : round(percents.percent_CA * nd);
  // lazy calculate points
  Point3::lazy_side_points_result side_point_result =
      rotation == ico::rotation_method::gnomonic
          ? Point3::lazy_side_points_gnomonic(*this, estimated_vert_center, res)
          : Point3::lazy_side_points_quaternion(*this, estimated_vert_center,
                                                res);

  // replaced n with i
  // int n = 0;

  std::vector<std::vector<Point3>> points;

  const int estimated_horz_center = round(percents.percent_CB * nd);
  // while vertical points exist, generate points for their rows in range
  int lower_horz_bound;
  // not hit or miss like js hexmap, here lazy range starts from vec[0]
  // accompanied by indx
  // TODO: need to test (check above comment)
  for (int i = 0; i < side_point_result.pointsL.size(); i++) {
    // generates points for range between left and right points along vertical
    // triangle sides (AB an AC)
    Point3 left = this->direction == tri::pointing::UP
                      ? side_point_result.pointsL[i]
                      : side_point_result.pointsR[i];
    Point3 right = this->direction == tri::pointing::UP
                       ? side_point_result.pointsR[i]
                       : side_point_result.pointsL[i];
    int num_div = this->direction == tri::pointing::UP
                      ? side_point_result.lower_indx + i
                      : nd - (side_point_result.lower_indx + i);

    Point3::lazy_row_points_result row_points_result =
        rotation == ico::rotation_method::gnomonic
            ? Point3::lazy_row_points_gnomonic(estimated_horz_center, left,
                                               right, num_div)
            : Point3::lazy_row_points_quaternion(estimated_horz_center, left,
                                                 right, num_div);
    points.push_back(row_points_result.row_points);
    // better way to set lower_horz_bound? only need last value...
    lower_horz_bound = row_points_result.lower_indx;
  }
  return {.points = points,
          .start_vert = side_point_result.lower_indx,
          .start_horz = lower_horz_bound};
}

Point3 Triangle::generate_point(int res, int lower_vert, int lower_horz,
                                ico::rotation_method rotation) const {
  const int nd = hexmapf::num_divisions(res);
  // kind of hacky but works
  Point3::lazy_side_points_result vert_result =
      rotation == ico::rotation_method::gnomonic
          ? Point3::lazy_side_points_gnomonic(*this, lower_vert, res,
                                              constants::lazy_range, lower_vert,
                                              lower_vert)
          : Point3::lazy_side_points_quaternion(*this, lower_vert, res,
                                                constants::lazy_range,
                                                lower_vert, lower_vert);

  const Point3 left = this->direction == tri::pointing::UP
                          ? vert_result.pointsL[0]
                          : vert_result.pointsR[0];
  const Point3 right = this->direction == tri::pointing::UP
                           ? vert_result.pointsR[0]
                           : vert_result.pointsL[0];
  const Point3::lazy_row_points_result horz_result =
      rotation == ico::rotation_method::gnomonic
          ? Point3::lazy_row_points_gnomonic(lower_horz, left, right, nd,
                                             constants::lazy_range, lower_horz,
                                             lower_horz)
          : Point3::lazy_row_points_quaternion(lower_horz, left, right, nd,
                                               constants::lazy_range,
                                               lower_horz, lower_horz);

  return horz_result.row_points[0];
};

bool Triangle::contains_point(Point3 &point) const {
  // vec and tri intersection point
  const Point3 intersection = this->plane_intersection(point);
  // check if intersection on opposite side
  if (intersection.on_opposite_side(point)) {
    return false;
  }
  // check if coords ok
  if (!intersection.is_valid()) {
    return false;
  }
  // calc tri area
  const double tri_area = this->area();
  // if any sub tri area is bigger than thisArea it means point outside of
  // triangle
  const double pAB_area = Triangle(this->A, this->B, intersection).area();
  if (pAB_area > tri_area + 0.01) {
    return false;
  }
  const double pBC_area = Triangle(intersection, this->B, this->C).area();
  if (pBC_area > tri_area + 0.01) {
    return false;
  }
  const double pCA_area = Triangle(this->A, intersection, this->C).area();
  if (pCA_area > tri_area + 0.01) {
    return false;
  }
  // round and check if equal enough
  const double combined_area = pAB_area + pBC_area + pCA_area;
  return hexmapf::equal_enough(tri_area, combined_area);
};

Point3 Triangle::plane_intersection(Point3 vec) const {
  // x component
  const double l = (this->A.y - this->B.y) * (this->C.z - this->B.z) -
                   (this->A.z - this->B.z) * (this->C.y - this->B.y);
  // y component
  const double m = (this->A.z - this->B.z) * (this->C.x - this->B.x) -
                   (this->A.x - this->B.x) * (this->C.z - this->B.z);
  // z component
  const double n = (this->A.x - this->B.x) * (this->C.y - this->B.y) -
                   (this->C.x - this->B.x) * (this->A.y - this->B.y);
  // finds v - variable used to find point along vector from origin to p on
  // plane
  const double v_numer = l * this->A.x + m * this->A.y + n * this->A.z;
  const double v_denom = l * vec.x + m * vec.y + n * vec.z;
  const double v = v_numer / v_denom;
  // parametric equation for line along vec (from origin)
  const double x = vec.x * v;
  const double y = vec.y * v;
  const double z = vec.z * v;
  return Point3(x, y, z);
};

double Triangle::area() const {
  Point3 AB = this->B;
  AB.subtract(this->A);
  Point3 BC = this->C;
  BC.subtract(this->B);
  Point3 temp = AB;
  AB.cross(BC);
  return temp.mag();
}