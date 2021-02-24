#pragma once

#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP

#include "enums.hpp"
#include "point3.hpp"

class Triangle {

public:
  const Point3 A;
  const Point3 B;
  const Point3 C;
  const tri::pointing direction;
  const tri::position pos;
  const int num;
  // triangle number next to this one (shares side)
  const int toAB;
  const int toBC;
  const int toCA;
  Triangle(Point3 A, Point3 B, Point3 C,
           tri::pointing direction = tri::pointing::NA,
           tri::position pos = tri::position::NA, int num = -1, int toAB = -1,
           int toBC = -1, int toCA = -1);

  struct lazy_points_around_result {
    std::vector<std::vector<Point3>> points;
    int start_vert;
    int start_horz;
  };

  struct calc_percent_result {
    // vertical percent
    long double percent_CA;
    // horizontal percent
    long double percent_CB;
  };

  struct vec_side_components_result {
    Point3 vec_CA;
    Point3 vec_CB;
  };

  /**
   * calculates percent of point along tri sides CA and CB
   **/
  static vec_side_components_result vec_side_components(const Triangle &tri,
                                                        const Point3 &i);

  /**
   * calc percent of p along tri sides
   * @param tri triangle
   * @param p point inside [tri]
   * @returns percents along CA and CB
   **/
  calc_percent_result calc_percent_gnomonic(const Point3 &p) const;

  /**
   * not ready yet!!!!!!!!
   * @param tri triangle
   * @param p point inside [tri]
   * @returns percents along CA and CB
   **/
  calc_percent_result calc_percent_quaternion(const Point3 &p) const;

  /**
   * @param res resolution
   * @param rotation rotation method to generate points with
   * @return 2d std::vector of triangle's points for [res] */
  std::vector<std::vector<Point3>> all_points(int res,
                                              ico::rotation_method rm) const;
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
  lazy_points_around_result
  lazy_points_around(Point3 &p, int res, ico::rotation_method rotation) const;

  /**
   * @param res resolutiom
   * @param lower_vert lower vertical index of point
   * @param lower_horz lower horizontal index of point
   * @param rotation rotation method
   * @returns point from lower indices */
  Point3 generate_point(int res, int lower_vert, int lower_horz,
                        ico::rotation_method rotation) const;

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
  long double area() const;
};

#endif