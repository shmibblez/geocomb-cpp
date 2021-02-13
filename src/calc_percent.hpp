#pragma once
#ifndef CALC_PERCENT_HPP
#define CALC_PERCENT_HPP

#include "icosahedron.hpp"
#include "point3.hpp"

class Triangle;

namespace CalcPercent {

struct calc_percent_result {
  // vertical percent
  double percent_CA;
  // horizontal percent
  double percent_CB;
};

struct vec_side_components_result {
  Point3 vec_CA;
  Point3 vec_CB;
};

// TODO: extern functions instead of making them static (will get copied a lot,
// even though only used in Triangle.cpp - i think)

static vec_side_components_result vec_side_components(const Triangle &tri,
                                                      const Point3 &i);

/**
 * calc percent of p along tri sides
 * @param tri triangle
 * @param p point inside [tri]
 * @returns percents along CA and CB
 **/
static calc_percent_result gnomonic(const Triangle &tri, const Point3 &p);

/**
 * not ready yet!!!!!!!!
 * @param tri triangle
 * @param p point inside [tri]
 * @returns percents along CA and CB
 **/
static calc_percent_result quaternion(const Triangle &tri, const Point3 &p);

}; // namespace CalcPercent

#endif