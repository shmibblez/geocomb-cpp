#include "triangle.hpp"
#include "constants.hpp"
#include "icosahedron.hpp"
#include "point3.hpp"

Triangle::Triangle(Point3 A, Point3 B, Point3 C,
                   pointing direction = pointing::NA,
                   position pos = position::NA, int num = -1, int toAB = -1,
                   int toBC = -1, int toCA = -1)
    : A(A), B(B), C(C), direction(direction), pos(pos), num(num), toAB(toAB),
      toBC(toBC), toCA(toCA){};

/**
 * @param res resolution
 * @return 2d std::vector of triangle's points for [res] */
std::vector<std::vector<Point3>>
Triangle::generate_all_points(int res, rotation_method rotation) const {
  // empty 2d vec
  std::vector<std::vector<Point3>> points;

  const int max_divisions = hexmapf::num_divisions(res);
  const Point3 left_above = this->direction == pointing::UP ? A : B;
  const Point3 left_below = this->direction == pointing::UP ? C : A;
  std::vector<Point3> left_points =
      rotation == rotation_method::gnomonic
          ? Point3::all_side_points_gnomonic(left_above, left_below, res)
          : Point3::all_side_points_quaternion(left_above, left_below, res);

  const Point3 right_above = this->direction == pointing::UP ? A : C;
  const Point3 right_below = this->direction == pointing::UP ? B : A;
  std::vector<Point3> right_points =
      rotation == rotation_method::gnomonic
          ? Point3::all_side_points_gnomonic(right_above, right_below, res)
          : Point3::all_side_points_quaternion(right_above, right_below, res);

  for (int x = 0; x <= max_divisions; x++) {
    int num_divs = this->direction == pointing::UP ? x : max_divisions - x;
    std::vector<Point3> new_points =
        rotation == rotation_method::gnomonic
            ? Point3::all_row_points_gnomonic(left_points[x], right_points[x],
                                              num_divs)
            : Point3::all_row_points_quaternion(left_points[x], right_points[x],
                                                num_divs);
    points.push_back(new_points);
  }
  return points;
}
/**
 * @param p point to generate points arounmd
 * @param res resolution
 * @param rotation rotation method
 * @returns (std::vector vec) where
 * - vec[0] = 2d vector (rows & cols)
 * - vec[1] = starting vertical index
 * - vec[2] = starting horizontal index
 * - starting indexes are in relation to tri.C -> pointing direction
 * influences row and col num calculation */
std::vector<std::any>
Triangle::generate_lazy_points_around(Point3 &p, int res,
                                      rotation_method rotation) const {}

/**
 * @param res resolutiom
 * @param lower_vert lower vertical index of point
 * @param lower_horz lower horizontal index of point
 * @param rotation rotation method
 * @returns point from lower indices */
Point3 Triangle::generate_point(int res, int lower_vert, int lower_horz,
                                rotation_method rotation) const {
  const int nd = hexmapf::num_divisions(res);
  // kind of hacky but works
  const Point3::lazy_side_points_result vert_result =
      rotation == rotation_method::gnomonic
          ? Point3::lazy_side_points_gnomonic(*this, lower_vert, res,
                                              constants::lazy_range, lower_vert,
                                              lower_vert)
          : Point3::lazy_side_points_quaternion(*this, lower_vert, res,
                                                constants::lazy_range,
                                                lower_vert, lower_vert);

  const Point3 left = this->direction == pointing::UP ? vert_result.pointsL[0]
                                                      : vert_result.pointsR[0];
  const Point3 right = this->direction == pointing::UP ? vert_result.pointsR[0]
                                                       : vert_result.pointsL[0];
  const Point3::lazy_row_points_result horz_result =
      rotation == rotation_method::gnomonic
          ? Point3::lazy_row_points_gnomonic(lower_horz, left, right, nd,
                                             constants::lazy_range, lower_horz,
                                             lower_horz)
          : Point3::lazy_row_points_quaternion(lower_horz, left, right, nd,
                                               constants::lazy_range,
                                               lower_horz, lower_horz);

  return horz_result.row_points[0];
};

/**
 * @param point point to test
 * @returns whether triangle contains point */
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

/**
 * @param vec vector from origin to point
 * @returns point where [vec] intersects with this triangle's plane */
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

/**
 * @returns triangle area */
double Triangle::area() const {
  Point3 AB = this->B;
  AB.subtract(this->A);
  Point3 BC = this->C;
  BC.subtract(this->B);
  Point3 temp = AB;
  AB.cross(BC);
  return temp.mag();
}