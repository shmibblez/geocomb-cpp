#include "point3.hpp"
#include <any>
#include <vector>

enum pointing { UP, DOWN };
enum position { TOP, CENTER, BOT };

class Triangle {

public:
  Point3 A;
  Point3 B;
  Point3 C;
  pointing direction;
  position pos;
  int num;
  // triangle number next to this one (shares side)
  int toAB;
  int toBC;
  int toCA;
  Triangle(Point3 A, Point3 B, Point3 C, pointing direction, position pos,
           int num = -1, int toAB = -1, int toBC = -1, int toCA = -1);

  /**
   * @param res resolution
   * @param rotation rotation method to generate points with
   * @return 2d std::vector of triangle's points for [res] */
  std::vector<std::vector<Point3>>
  generate_all_points(int res, rotation_method rotation) const;
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
  generate_lazy_points_around(Point3 &p, int res,
                              rotation_method rotation) const;

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

private:
  /**
   * @returns triangle area */
  double area() const;
};