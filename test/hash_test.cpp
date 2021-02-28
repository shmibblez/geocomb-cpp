#include "../src/enums.hpp"
#include "../src/icosahedron.hpp"
#include "../src/point3.hpp"
#include <iostream>
#include <limits>

// TODO: tidy up (make console output more readable), add ico.random_point(),
// and add function for testing lots of points for hash generation & hash
// parsing (copy Icosahedron::hash implementation to get gpoint3, and compare
// with parsed point)
//
// add below to hash test for generated point & comparison
// std::cout << "\nhashed point:\n  x: " << cp.x << "\n  y: " << cp.y
//           << "\n  z: " << cp.z << "\n  lat: " << cp.get_lat()
//           << "\n  lon: " << cp.get_lon() << "\n  row: " << cp.row
//           << "\n  col: " << cp.col << "\n";

int main() {
  Icosahedron ico =
      Icosahedron(ico::map_orientation::ECEF, ico::rotation_method::gnomonic);

  long double lat, lon;
  unsigned int res;

  // std::cout << "\ninsert point coords, lat: ";
  // std::cin >> lat;
  // std::cout << "lon: ";
  // std::cin >> lon;
  // std::cout << "res: ";
  // std::cin >> res;

  lat = (long double)71;
  lon = (long double)27;
  res = 10;

  std::cout.precision(std::numeric_limits<long double>::max_digits10);

  // std::cout << "sizeof double: " << sizeof(double) << "\n";
  // std::cout << "sizeof long double: " << sizeof(long double) << "\n";

  std::cout << "\npoint coords:\n  lat: " << lat << "\n  lon: " << lon
            << "\n  res: " << res << "\n";

  // generated point
  Point3 p = ico.point_from_coords(lat, lon);

  std::cout << "\ngenerated point:\n  x: " << p.x << "\n  y: " << p.y
            << "\n  z: " << p.z << "\n  lat: " << p.get_lat()
            << "\n  lon: " << p.get_lon() << "\n";

  Icosahedron::hash_properties props = ico.hash(p, res);

  std::cout << "\npoint hash properties below";
  std::cout << "\n  res: " << props.res << "\n  row: " << props.row
            << "\n  col: " << props.col << "\n";

  // parsed point
  GPoint3 pp = ico.parse_hash(props);

  std::cout << "\nparsed point:\n  x: " << pp.x << "\n  y: " << pp.y
            << "\n  z: " << pp.z << "\n  lat: " << pp.get_lat()
            << "\n  lon: " << pp.get_lon() << "\n  row: " << pp.row
            << "\n  col: " << pp.col << "\n";
}

// TODO: for high res hashes don't match with hexmap node, try compiling with
// no-rounding flags, or use long double

//.
// FIXME: not working for coords (doesn't match with hexmap-node hash), most
// likely due to rounding errors
//
// lat  |  lon  |  res -70  |  179  |  144000 & 144