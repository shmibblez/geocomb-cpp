#include "../src/hexmap/enums.hpp"
#include "../src/hexmap/icosahedron.hpp"
#include "../src/hexmap/point3.hpp"
#include <iostream>

int main() {
  Icosahedron ico =
      Icosahedron(ico::map_orientation::ECEF, ico::rotation_method::gnomonic,
                  ico::hash_type::rowCol);

  int lat, lon;
  unsigned int res;

  std::cout << "\ninsert point coords, lat: ";
  std::cin >> lat;
  std::cout << "lon: ";
  std::cin >> lon;
  std::cout << "res: ";
  std::cin >> res;

  Point3 p = ico.point_from_coords(lat, lon);
  Icosahedron::hash_properties props = ico.hash(p, res);

  std::cout << "\ncreated point, hash properties belowss";
  std::cout << "\nres: " + std::to_string(props.res)
            << ",\nrow: " + std::to_string(props.row)
            << ",\ncol: " + std::to_string(props.col) << "\n";
}

// TODO: fix bugz