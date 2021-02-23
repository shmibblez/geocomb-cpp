#pragma once

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>

namespace constants {

// for generating points around another point
inline int lazy_range = 2;
// earth's radius
inline long double radius = 250.0;
// golden ratio
inline long double golden_ratio = 1.618033988749894848205;
// pi
inline long double PI = 3.141592653589793238463;
}; // namespace constants

// hexmap functions
namespace hexmapf {

// calculates number of divisions from resolution, always res * 3
inline int num_divisions(int resolution) { return resolution * 3; };
// round 2 nums and check if equal enough
inline bool equal_enough(long double n1, long double n2, int places = 6) {
  int mult = pow(10, places);
  long left = round(n1 * mult);
  long right = round(n2 * mult);
  bool result = left == right;
  return result;
}
// rounds up to next multiple
inline int round_up(const int num, const int mult) {
  // (A + B - 1) / B
  if (num % mult == 0) {
    return num;
  }
  return (trunc(num / mult) + 1) * mult;
}
// convert from deg to rad
inline long double deg_2_rad(long double deg) {
  return deg * (constants::PI / 180.0);
};
// gets closest even number by subtracting 1 if odd
inline int closest_even_num(const int n) {
  const bool is_even = n % 2 == 0;
  return is_even ? n : n - 1;
}

}; // namespace hexmapf

#endif