#include "../src/enums.hpp"
#include "../src/icosahedron.hpp"
#include "../src/point3.hpp"
#include <iostream>
#include <limits>
#include <string>

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

  lat = 47;
  lon = 47;
  res = 777;

  // std::cout.precision(std::numeric_limits<long double>::max_digits10);

  // std::cout << "sizeof double: " << sizeof(double) << "\n";
  // std::cout << "sizeof long double: " << sizeof(long double) << "\n";

  // std::cout << "\npoint coords:\n  lat: " << lat << "\n  lon: " << lon
  //           << "\n  res: " << res << "\n";

  // generated point
  Point3 p = ico.point_from_coords(lat, lon);

  // std::cout << "\ngenerated point:\n  x: " << p.x << "\n  y: " << p.y
  //           << "\n  z: " << p.z << "\n  lat: " << p.get_lat()
  //           << "\n  lon: " << p.get_lon() << "\n";

  Icosahedron::hash_properties props = ico.hash(p, res);

  // std::cout << "\npoint hash properties below";
  // std::cout << "\n  res: " << props.res << "\n  row: " << props.row
  //           << "\n  col: " << props.col << "\n";

  // parsed point
  GPoint3 pp = ico.parse_hash(props);

  // std::cout << "\nparsed point:\n  x: " << pp.x << "\n  y: " << pp.y
  //           << "\n  z: " << pp.z << "\n  lat: " << pp.get_lat()
  //           << "\n  lon: " << pp.get_lon() << "\n  row: " << pp.row
  //           << "\n  col: " << pp.col << "\n";

  printf("hash: e|g|%d|%d|%d", props.res, props.row, props.col);
}

// TODO: for high res hashes don't match with hexmap node, try compiling with
// no-rounding flags, or use long double

//.
// FIXME: not working for coords (doesn't match with hexmap-node hash), most
// likely due to rounding errors
//
// lat  |  lon  |  res -70  |  179  |  144000 & 144