#include "phex.hpp"
#include "point3.hpp"
#include "triangle.hpp"
#include <vector>

enum hash_type { rowCol, nested };

enum map_orientation { ECEF, dymaxion };

enum rotation_method { gnomonic, quaternion };

class Icosahedron {

public:
  std::vector<Triangle> triangles;
  map_orientation orientation;
  rotation_method rotation;
  Icosahedron(map_orientation orientation = map_orientation::ECEF,
              rotation_method rotation = rotation_method::gnomonic);

  typedef std::vector<std::vector<GPoint3>> all_points;

  struct hash_properties {
    int res;
    int row;
    int col;
  };

  /**
   * generate point from coordinates (degrees)
   * @param lat latitude
   * @param lon longitude */
  Point3 point_from_coords(double lat, double lon) const;

  // TODO: generate hash here, in js version it's in Point3, point_from_coords
  // is also in Point3 in js version
  hash_properties hash(Point3 p);

  std::vector<std::vector<Point3>> lazy_points_around(Point3 p, int res) const;

  Phex not_lazy_get_containing_phex(Point3 p, int res) const;

  Triangle get_containing_triangle(Point3 p) const;
};