#pragma once
// #ifndef CALC_PERCENT_HPP
// #define CALC_PERCENT_HPP

#include "icosahedron.hpp"
#include "point3.hpp"

class Triangle;

namespace CalcPercent {

// public:
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
static CalcPercent::vec_side_components_result
vec_side_components(const Triangle &tri, const Point3 &i);

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

// #endif