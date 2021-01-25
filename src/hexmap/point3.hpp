#ifndef POINT3_HPP
#define POINT3_HPP

#include "constants.hpp"
#include "enums.hpp"
#include "icosahedron.hpp"
#include <cmath>
#include <limits>
#include <vector>

class Triangle;
class GPoint3;

class Point3 {

public:
  double x;
  double y;
  double z;
  int tri_num;
  bool is_vert;
  Point3(int x, int y, int z, bool is_vert = false, int tri_num = -1);
  ~Point3();

  struct lazy_side_points_result {
    std::vector<Point3> pointsL;
    std::vector<Point3> pointsR;
    int lower_indx;

    // TODO: how to save values to struct? (constructor)
    // and if possible any way to make them const?
    //.
    // CHORE: IMPORTANT need to push to main
  };

  struct lazy_row_points_result {
    std::vector<Point3> row_points;
    int lower_indx;
    lazy_row_points_result(std::vector<Point3> row_points, int lower_indx)
        : row_points(row_points), lower_indx(lower_indx){};
    ~lazy_row_points_result();
  };

  /**
   * VECTOR ARITHMETIC, NOTHING TOO SPECIAL HERE
   * next section is where the action happens
   */

  /**
   * angle between vectors (origin, this) and (origin, p) */
  double angle_between(const Point3 &p) const;
  /**
   * @param around vec to rotate around -> vec is from origin to point
   * @param rad rads to rotate
   * @note modifies obj */
  void rotate(const Point3 &around, const double &rad);
  /**
   * magnitude from origin to point */
  double mag() const;
  /**
   * add another vector
   * !-> modifies obj */
  void add(const Point3 &p);
  /**
   * subtract another vector
   * !-> modifies obj */
  void subtract(const Point3 &p);
  /**
   * dot product */
  double dot(const Point3 &p) const;
  /**
   * cross product
   * !-> modifies obj */
  void cross(const Point3 &p);
  /**
   * convert into unit vec
   * !-> modifies obj */
  void unit();
  /**
   * multiply by scalar
   * !-> modifies obj */
  void mult_by(const double num);
  /**
   * divide by scalar
   * !-> modifies obj */
  void div_by(const double num);
  /**
   * distance between vectors */
  double distance(const Point3 &p) const;
  /**
   * move point to be on sphere (multiply unit vec by radius) */
  void spheriphy();
  /**
   * check if points on opposite sides of globe
   * !-> loosely determined -> points don't need to be 180 deg apart, in some
   * cases points 90 deg apart will return true */
  bool on_opposite_side(const Point3 &p) const;
  /**
   * check if point is valid (all coords finite & valid numbers) */
  bool is_valid() const;
  /**
   * @returns copy of closest point in point arr */
  GPoint3 closest_point(std::vector<GPoint3> &points) const;
  /**
   * @returns copy of closest point in 2d point arr */
  GPoint3 closest_point_2d(std::vector<std::vector<GPoint3>> &points_2d) const;

  /**
   * HEXMAP STUFF, THIS IS WHERE IT GETS INTERESTING
   */
  /**
   * rotates point around y axis !-> modifies obj
   * @param rads how many radians to rotate point */
  void rotate_around_y(double rads);

  /**
   * GNOMONIC POINT GENERATION
   */

  /**
   * generates all points between [above] and [below]
   * @param above point above
   * @param below point below
   * @param res resolution
   * @returns vector (array) of points between above and below (inclusive),
   * above is first point, below is last */
  static std::vector<Point3>
  all_side_points_gnomonic(const Point3 &above, const Point3 &below, int res);
  /**
   * generates only necessary points along right and left triangle sides
   * @param tri triangle (for referencing points)
   * @param center point # from top to bottom (top is B or C if tri points down,
   * and A if tri points up)
   * @param res resolution
   * @param lazy_range? # of points to generate around center
   * @param lower? lower point # (lower limit of points to generate from first
   * point)
   * @param upper? upper point # (upper limit of points to generate from first
   * point)
   * @returns vector -> [left side points, right side points, lower bound
   * offset]
   * - (std::vector<Point3>)left side points are points from B to A if tri
   * pointing down, and from A to C if tri pointing up
   * - (std::vector<Point3>) right side points are points from C to A if tri
   * pointing down, and from A to B if tri pointing up
   * - (int) lower bound offset is first generated point # from first to last */
  static lazy_side_points_result
  lazy_side_points_gnomonic(const Triangle &tri, const int center,
                            const int res,
                            const int lazy_range = constants::lazy_range,
                            int lower = -1, int upper = -1);
  /**
   * @param left left bound
   * @param right right bound
   * @param num_dividions number of divisions on triangle row
   * @returns list of points between left and right points inclusive, if
   * num_divisions is 0, it means at triangle tip and returns list with 1 point
   */
  static std::vector<Point3> all_row_points_gnomonic(const Point3 &left,
                                                     const Point3 &right,
                                                     int num_divisions);
  /**
   * generates only necessary points between [left] and [right] points
   * @param center point # from left to right to generate points around
   * @param left left bound
   * @param right right bound
   * @param num_divisions number of divisions on row
   * @param lazy_range? # of points to generate around center point
   * @param lower? lower point # from left (lower limit of points to generate)
   * @param upper? upper point # from left (upper limit of points to generate)
   * @returns vector -> [points, lower bound offset]
   * - (std::vector<Point3>) points are generated points !-> vector only
   * includes points (means vector[0] is lower_bound_offset -> vector[0] != left
   * point)
   * - (int) lower bound offset is first generated point # from left to right */
  static lazy_row_points_result
  lazy_row_points_gnomonic(const int center, const Point3 &left,
                           const Point3 &right, int num_divisions,
                           const int lazy_range = constants::lazy_range,
                           int lower = -1, int upper = -1);

  /**
   * QUATERNION POINT GENERATION
   */

  /**
   * generates all points between [above] and [below]
   * @param above point above
   * @param below point below
   * @param res resolution
   * @returns vector (array) of points between above and below (inclusive),
   * above is first point, below is last */
  static std::vector<Point3>
  all_side_points_quaternion(const Point3 &above, const Point3 &below, int res);
  /**
   * generates only necessary points along right and left triangle sides
   * @param tri triangle (for referencing points)
   * @param center point # from top to bottom (top is B or C if tri points down,
   * and A if tri points up)
   * @param res resolution
   * @param lazy_range? # of points to generate around center
   * @param lower? lower point # (lower limit of points to generate from first
   * point)
   * @param upper? upper point # (upper limit of points to generate from first
   * point)
   * @returns vector -> [left side points, right side points, lower bound
   * offset]
   * - (std::vector<Point3>)left side points are points from B to A if tri
   * pointing down, and from A to C if tri pointing up
   * - (std::vector<Point3>) right side points are points from C to A if tri
   * pointing down, and from A to B if tri pointing up
   * - (int) lower bound offset is first generated point # from first to last */
  static lazy_side_points_result
  lazy_side_points_quaternion(const Triangle &tri, const int center,
                              const int res,
                              const int lazy_range = constants::lazy_range,
                              int lower = -1, int upper = -1);
  /**
   * @param left left bound
   * @param right right bound
   * @param num_dividions number of divisions on triangle row
   * @returns list of points between left and right points inclusive, if
   * num_divisions is 0, it means at triangle tip and returns list with 1 point
   */
  static std::vector<Point3> all_row_points_quaternion(const Point3 &left,
                                                       const Point3 &right,
                                                       int num_divisions);
  /**
   * generates only necessary points between [left] and [right] points
   * @param center point # from left to right to generate points around
   * @param left left bound
   * @param right right bound
   * @param num_divisions number of divisions on row
   * @param lazy_range? # of points to generate around center point
   * @param lower? lower point # from left (lower limit of points to generate)
   * @param upper? upper point # from left (upper limit of points to generate)
   * @returns vector -> [points, lower bound offset]
   * - (std::vector<Point3>) points are generated points !-> vector only
   * includes points (means vector[0] is lower_bound_offset -> vector[0] != left
   * point)
   * - (int) lower bound offset is first generated point # from left to right */
  static lazy_row_points_result
  lazy_row_points_quaternion(const int center, const Point3 &left,
                             const Point3 &right, int num_divisions,
                             const int lazy_range = constants::lazy_range,
                             int lower = -1, int upper = -1);

private:
  /**
   * @param points points to spheriphy
   * !-> modifies obj */
  static void spherify1D(std::vector<Point3> &points);

  /**
   * NOT IMPLEMENED (list functions that haven't been implemented yet & are in
   * hexmap js)
   *
   * pointNum(GPoint3 p)
   */
};

class GPoint3 : public Point3 {

public:
  int res;
  int row;
  int col;
  GPoint3(double x, double y, double z, int res, int row, int col,
          ico::map_orientation mo, ico::rotation_method rm,
          bool is_vert = false, int tri_num = -1);

  /**
   * checks if point is phex center
   * @param res resolution
   * @param row row
   * @param col column
   * @return whether point is phex center
   **/
  static bool is_phex_center(const int res, const int row, const int col);

  /**
   * generate point from coordinates
   * @param lat latitude in degrees
   * @param lon longitude in degrees */
  static GPoint3 from_coordinates(double lat, double lon);
};

class Quaternion : public Point3 {

public:
  int w;
  Quaternion(double x, double y, double z, double w);

  /**
   * convert to unit quaternion */
  void unit();
  /**
   * get quaternion magnitude */
  double mag();
  /**
   * multiply with another quaternion
   * @note modifies obj */
  void multiply(const Quaternion &q);
};

#endif