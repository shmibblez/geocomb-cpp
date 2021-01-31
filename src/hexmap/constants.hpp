#pragma once

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>

namespace constants {
// for generating points around another point
inline const int lazy_range = 2;
// earth's radius
inline const double radius = 200;
// golden ratio
inline const double golden_ratio = (1 + sqrt(5)) / 2;
// pi
inline const double PI = 3.141592653589793238463;
}; // namespace constants

// hexmap functions
class hexmapf {
public:
  // calculates number of divisions from resolution, always res * 3
  static int num_divisions(int resolution);
  // round 2 nums and check if equal enough
  static bool equal_enough(double n1, double n2);

  // rounds up to next int i think
  static int round_up(const int num, const int mult);
  // convert from deg to rad
  static double deg_2_rad(double deg);
  /**
   * gets closest even number by subtracting 1 if odd
   * @param n number
   * @returns [closest even number, isEven]
   */
  static int closest_even_num(const int n);
};

#endif