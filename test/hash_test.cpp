#include "../src/enums.hpp"
#include "../src/icosahedron.hpp"
#include "../src/point3.hpp"
#include <iostream>

int main() {
  Icosahedron ico =
      Icosahedron(ico::map_orientation::ECEF, ico::rotation_method::gnomonic);

  int lat, lon;
  unsigned int res;

  std::cout << "\ninsert point coords, lat: ";
  std::cin >> lat;
  std::cout << "lon: ";
  std::cin >> lon;
  std::cout << "res: ";
  std::cin >> res;

  lat = -70;
  lon = 179;
  res = 144000;

  std::cout << "sizeof double: " << sizeof(double) << "\n";
  std::cout << "sizeof long double: " << sizeof(long double) << "\n";

  std::cout << "\n\nlat: " + std::to_string(lat)
            << ", lon: " + std::to_string(lon)
            << ", res: " << std::to_string(res) << "\n\n";

  Point3 p = ico.point_from_coords(lat, lon);

  std::cout << "point_from_coords: "
            << "\n  x: " << std::to_string(p.x)
            << "\n  y: " << std::to_string(p.y)
            << "\n  z: " << std::to_string(p.z) << "\n";

  Icosahedron::hash_properties props = ico.hash(p, res);

  std::cout << "\ncreated point, hash properties below";
  std::cout << "\nres: " + std::to_string(props.res)
            << ",\nrow: " + std::to_string(props.row)
            << ",\ncol: " + std::to_string(props.col) << "\n";
}

// TODO: for high res hashes don't match with hexmap node, try compiling with
// no-rounding flags, or use long double

//.
// FIXME: not working for coords (doesn't match with hexmap-node hash), most
// likely due to rounding errors
//
// lat  |  lon  |  res -70  |  179  |  144000 & 144