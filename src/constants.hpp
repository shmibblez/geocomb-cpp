#pragma once

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>

namespace constants {
// for generating points around another point
inline const int lazy_range = 2;
// earth's radius
inline const long double radius = 250.0;
// golden ratio
inline const long double golden_ratio = (1.0 + sqrt(5.0)) / 2.0;
// pi
inline const long double PI = 3.141592653589793238463;
}; // namespace constants

// hexmap functions
namespace hexmapf {

// calculates number of divisions from resolution, always res * 3
static int num_divisions(int resolution);
// round 2 nums and check if equal enough
static bool equal_enough(long double n1, long double n2, int places = 6);
// rounds up to next multiple
static int round_up(const int num, const int mult);
// convert from deg to rad
static long double deg_2_rad(long double deg);
// gets closest even number by subtracting 1 if odd
static int closest_even_num(const int n);
}; // namespace hexmapf

#endif