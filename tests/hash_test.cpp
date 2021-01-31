#include "../src/hexmap/enums.hpp"
#include "../src/hexmap/icosahedron.hpp"
#include "../src/hexmap/point3.hpp"
#include <iostream>

int main() {
  Icosahedron ico =
      Icosahedron(ico::map_orientation::ECEF, ico::rotation_method::gnomonic,
                  ico::hash_type::rowCol);

  Point3 p = ico.point_from_coords(45, 45);
  Icosahedron::hash_properties props = ico.hash(p, 7);

  std::cout << "created point, hash properties below\n";
  std::cout << "res: " + std::to_string(props.res)
            << ", row: " + std::to_string(props.row)
            << ", col: " + std::to_string(props.col);
}

// TODO: have multiple files, and have main() have options, and depending on
// which test call from main()