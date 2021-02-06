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

  std::cout << "\ncreated point, hash properties belowss";
  std::cout << "\nres: " + std::to_string(props.res)
            << ",\nrow: " + std::to_string(props.row)
            << ",\ncol: " + std::to_string(props.col) << "\n";
}

// TODO: fix bugz