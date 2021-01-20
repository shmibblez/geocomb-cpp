namespace constants {
// for generating points around another point
static const int lazy_range = 2;
// earth's radius
static const double radius = 200;
// golden ratio
// TODO: how to take sqrt?
static const double golden_ratio = (1 + sqrt(5)) / 2;
} // namespace constants

// hexmap functions
namespace hexmapf {
// calculates number of divisions from resolution, always res * 3
static int num_divisions(int resolution) { return resolution * 3; }
// round 2 nums and check if equal enough
static bool equal_enough(double n1, double n2) {
  // TODO: round num here
  throw "need to figure out how to round nums";
}

} // namespace hexmapf