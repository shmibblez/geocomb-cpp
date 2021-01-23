#include <cmath>

using std::round;

namespace constants {
// for generating points around another point
static const int lazy_range = 2;
// earth's radius
static const double radius = 200;
// golden ratio
// TODO: how to take sqrt?
static const double golden_ratio = (1 + sqrt(5)) / 2;

static const double PI = 3.141592653589793238463;
} // namespace constants

// hexmap functions
class hexmapf {
public:
  // calculates number of divisions from resolution, always res * 3
  static int num_divisions(int resolution) { return resolution * 3; };
  // round 2 nums and check if equal enough
  static bool equal_enough(double n1, double n2) {
    // TODO: round num here
    throw "need to figure out how to round nums";
  };
  // rounds up to next int i think
  static int round_up(const int num, const int mult) {
    // (A + B - 1) / B
    if (num % mult == 0) {
      return num;
    }
    return (trunc(num / mult) + 1) * mult;
  };
  // convert from deg to rad
  static double deg_2_rad(double deg) { return deg * (constants::PI / 180); };
  /**
   * gets closest even number by subtracting 1 if odd
   * @param n number
   * @returns [closest even number, isEven]
   */
  static int closest_even_num(const int n) {
    const bool is_even = n % 2 == 0;
    return is_even ? n : n - 1;
  };
};