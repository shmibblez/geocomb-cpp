#include "constants.hpp"

using std::round;
using std::sqrt;

// static const int constants::lazy_range = 2;

// static const double constants::radius = 200;

// static const double constants::golden_ratio = (1 + sqrt(5)) / 2;

// static const double constants::PI = 3.141592653589793238463;

int hexmapf::num_divisions(int resolution) { return resolution * 3; };

bool hexmapf::equal_enough(double n1, double n2) {
  // TODO: round num here
  throw "need to figure out how to round nums";
};

int hexmapf::round_up(const int num, const int mult) {
  // (A + B - 1) / B
  if (num % mult == 0) {
    return num;
  }
  return (trunc(num / mult) + 1) * mult;
};

double hexmapf::deg_2_rad(double deg) { return deg * (constants::PI / 180); };

int hexmapf::closest_even_num(const int n) {
  const bool is_even = n % 2 == 0;
  return is_even ? n : n - 1;
};
