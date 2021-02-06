#include "constants.hpp"

using std::pow;
using std::round;
using std::sqrt;

int hexmapf::num_divisions(int resolution) { return resolution * 3; };

bool hexmapf::equal_enough(double n1, double n2, int places) {
  int mult = pow(10, places);
  long left = round(n1 * mult);
  long right = round(n2 * mult);
  bool result = left == right;
  return result;
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
