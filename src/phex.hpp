#pragma once

#ifndef PHEX_HPP
#define PHEX_HPP

#include "icosahedron.hpp"
#include "point3.hpp"
#include <vector>

class Phex {

public:
  std::vector<GPoint3> points;
  Point3 center;

  Phex(std::vector<GPoint3> points, GPoint3 center);
  ~Phex();

  /**
   * @param all_points all icosahedron points, for generating phexes
   * @return vector of all icosahedron phexes
   **/
  static std::vector<Phex>
  all_phexes(Icosahedron::all_icosahedron_points all_points);

  /**
   * @param all_points all icosahedron points for res
   * @return vector of all phex centers
   **/
  static std::vector<GPoint3>
  all_phex_centers(Icosahedron::all_icosahedron_points all_points);

  /**
   * @param all_points all icosahedron points for res
   * @param p point to generate phex points for
   * @returns phex whose center is p
   **/
  static std::vector<GPoint3>
  not_lazy_surrounding_points(Icosahedron::all_icosahedron_points all_points,
                              GPoint3 p);
};

#endif