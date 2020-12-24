#include "triangle.hpp"
#include "constants.hpp"
#include "icosahedron.hpp"
#include "point3.hpp"

Triangle::Triangle(Point3 A, Point3 B, Point3 C, pointing direction,
                   position pos, int num = -1, int toAB = -1, int toBC = -1,
                   int toCA = -1)
    : A(A), B(B), C(C), direction(direction), pos(pos), num(num), toAB(toAB),
      toBC(toBC), toCA(toCA) {}

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
Point3 generate_point(int res, int lower_vert, int lower_horz,
                      rotation_method rotation) const;

/**
 * @param point point to test
 * @returns whether triangle contains point */
bool contains_point(Point3 &point) const;

/**
 * @param vec vector from origin to point
 * @returns point where [vec] intersects with this triangle's plane */
Point3 plane_intersection(Point3 vec) const;

/**
 * @returns triangle area */
double area() const;