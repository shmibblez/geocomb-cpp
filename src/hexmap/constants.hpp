namespace constants {
// for generating points around another point
static int lazy_range = 2;
// earth's radius
static double radius = 200;
} // namespace constants

// hexmap functions
namespace hexmapf {
// calculates number of divisions from resolution, always res * 3
static int num_divisions(int resolution) { return resolution * 3; }
} // namespace hexmapf