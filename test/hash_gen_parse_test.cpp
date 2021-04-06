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

void test_generated_parsed_equal(int times, int res) {
  const Icosahedron ico =
      Icosahedron(ico::map_orientation::ECEF, ico::rotation_method::gnomonic);

  for (int i = 0; i <= times; i++) {
    Point3 p = ico.random_point();
    // Icosahedron::hash implementation below
    Icosahedron::all_icosahedron_points lazy_points =
        ico.lazy_points_around(p, res);

    // closest point that is also phex center
    GPoint3 cp = p.closest_point_2d(lazy_points);

    Icosahedron::hash_properties props = Icosahedron::hash_properties{
        .res = res,
        .row = cp.row,
        .col = cp.col,
        .rm = ico.rm,
        .mo = ico.mo,
    };

    // parsed point
    GPoint3 pp = ico.parse_hash(props);

    if (cp.x != pp.x || cp.y != pp.y || cp.z != pp.z) {
      std::cout
          << "\n generated & parsed points for hash don't match, random point "
             "info:\n  lat: "
          << p.get_lat() << "\n  lon: " << p.get_lon()
          << "\ngenerated point info: " << cp.get_lat()
          << "\n  lon: " << cp.get_lon() << "\n  x: " << cp.x
          << "\n  y: " << cp.y << "\n  z: " << cp.z << "\n  is_pc: " << cp.is_pc
          << "\n  tri_num: " << cp.tri_num
          << "\nparsed point info: " << pp.get_lat()
          << "\n  lon: " << pp.get_lon() << "\n  x: " << pp.x
          << "\n  y: " << pp.y << "\n  z: " << pp.z << "\n  is_pc: " << pp.is_pc
          << "\n  tri_num: " << pp.tri_num << "\n";
    } else {
      std::cout << "success\n";
    }
  }

  std::cout << "\nmade sure generated & parsed points referenced by hash "
               "properties were equal "
            << times << " times, all succeed"
            << "\n";
};

int main() {
  std::cout << "\nhow many tests?\n";
  int times = 0;
  std::cin >> times;

  std::cout << "\nresolution?\n";
  int res = 0;
  std::cin >> res;

  test_generated_parsed_equal(times, res);

  return 1;
}