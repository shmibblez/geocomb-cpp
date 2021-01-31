#include "constants.hpp"

using std::round;
using std::sqrt;

int hexmapf::num_divisions(int resolution) { return resolution * 3; };

bool hexmapf::equal_enough(double n1, double n2) {
  // TODO: round num here
  throw "need to figure out how to round nums";
}

int hexmapf::round_up(const int num, const int mult) {
  // (A + B - 1) / B
  if (num % mult == 0) {
    return num;
  }
  return (trunc(num / mult) + 1) * mult;
}

double hexmapf::deg_2_rad(double deg) { return deg * (constants::PI / 180); };

int hexmapf::closest_even_num(const int n) {
  const bool is_even = n % 2 == 0;
  return is_even ? n : n - 1;
}
